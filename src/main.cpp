#include <Arduino.h>
#include <SI470X.h>
#include <WiFi.h>
#include <Wire.h>

#include "audio_source.h"
#include "config.h"
#include "streamer.h"

// ============================== Состояние ==============================
static SI470X tuner;
static AudioStreamer streamer;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint64_t s_sampleIndex = 0;      // абсолютная шкала сэмплов (для статистики)
static uint32_t s_framesDropped = 0;    // кадры, не ушедшие на сервер
static volatile uint32_t s_lastDataMs = 0;   // когда последний раз были сэмплы
static volatile int32_t s_peak = 0;          // пик последнего кадра

// ================================ Wi-Fi ===============================
static void wifiConnect() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                  // иначе периодические просадки throughput
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(DEVICE_ID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[wifi] подключение к \"%s\"", WIFI_SSID);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] IP %s, RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("[wifi] пока не подключился, пробую в фоне");
  }
}

static void wifiKeepAlive() {
  if (WiFi.status() == WL_CONNECTED) return;

  static uint32_t nextTry = 0;
  const uint32_t now = millis();
  if ((int32_t)(now - nextTry) < 0) return;
  nextTry = now + 10000;

  Serial.println("[wifi] переподключение");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// =============================== Тюнер ================================
// Si4703 в 2-wire режиме живёт на адресе 0x10 (SEN -> GND).
// Если чип не отвечает, библиотека PU2CLR зависает навсегда в
// waitAndFinishTune(): неудачные чтения дают 0xFFFF, а STC (0Ah[14]) в этом
// случае равен 1, и цикл "ждём сброса STC" не заканчивается. Поэтому наличие
// устройства на шине проверяем до настройки частоты.
static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;   // 0 = ACK
}

static void i2cScan() {
  Serial.print("[i2c] устройства на шине:");
  bool any = false;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    if (i2cProbe(addr)) {
      Serial.printf(" 0x%02X", addr);
      any = true;
    }
  }
  if (!any) Serial.print(" нет");
  Serial.println();
}

static void tunerInit() {
  Serial.println("[tuner] инициализация Si470x...");

  // SI470X::setup() вызывает Wire.begin() без аргументов и взял бы пины по
  // умолчанию из варианта платы, поэтому задаём их через setPins() до него.
  Wire.setPins(PIN_TUNER_SDA, PIN_TUNER_SCL);
  Wire.setTimeOut(50);

  // Пока Si4703 не прошёл reset, его I2C-интерфейс выключен и он не отвечает.
  pinMode(PIN_TUNER_RESET, OUTPUT);
  digitalWrite(PIN_TUNER_RESET, HIGH);
  delay(10);

  tuner.setup(PIN_TUNER_RESET, PIN_TUNER_SDA);

  if (!i2cProbe(0x10)) {
    Serial.printf("[tuner] нет ответа на 0x10: SDA=GPIO%d, SCL=GPIO%d, RST=GPIO%d, "
                  "SEN -> GND, питание 3.3 В\n",
                  PIN_TUNER_SDA, PIN_TUNER_SCL, PIN_TUNER_RESET);
    i2cScan();
    Serial.println("[tuner] настройка пропущена (иначе setFrequency() зависнет)");
    return;
  }

  tuner.setVolume(TUNER_VOLUME);
  tuner.setMono(TUNER_MONO);
  tuner.setFrequency(TUNER_FREQUENCY);

  Serial.printf("[tuner] %.1f МГц (mono=%d, volume=%d)\n",
                TUNER_FREQUENCY / 100.0, (int)TUNER_MONO, TUNER_VOLUME);
}

// ============================ Аудио-задача ============================
// Читает I2S блоками по FRAME_MS и отправляет кадры на сервер.
// Номер сэмпла монотонно растёт всё время работы, поэтому после обрыва связи
// сервер видит разрыв first_sample и добивает его тишиной.
static void audioTask(void *) {
  static int16_t pcm[FRAME_SAMPLES * CHANNELS];

  uint64_t sampleIndex = 0;
  uint32_t nextConnectAt = 0;

  for (;;) {
    const size_t frames = audio_read(pcm, FRAME_SAMPLES, 100);
    if (frames == 0) continue;                       // нет клока / ещё нет данных

    int32_t peak = 0;
    const size_t total = frames * CHANNELS;
    for (size_t i = 0; i < total; ++i) {
      const int32_t v = pcm[i] < 0 ? -pcm[i] : pcm[i];
      if (v > peak) peak = v;
    }
    s_peak = peak;
    s_lastDataMs = millis();

    const uint64_t firstSample = sampleIndex;
    sampleIndex += frames;

    portENTER_CRITICAL(&s_mux);
    s_sampleIndex = sampleIndex;
    portEXIT_CRITICAL(&s_mux);

    if (!streamer.connected()) {
      if (WiFi.status() != WL_CONNECTED) continue;    // сеть упала — просто копим время

      const uint32_t now = millis();
      if ((int32_t)(now - nextConnectAt) < 0) continue;

      if (!streamer.connect()) {
        nextConnectAt = now + RECONNECT_DELAY_MS;
        ++s_framesDropped;
        continue;
      }
    }

    if (!streamer.sendFrame(pcm, frames, firstSample)) {
      Serial.println("[net] соединение потеряно, переподключаюсь");
      streamer.disconnect();
      nextConnectAt = millis() + RECONNECT_DELAY_MS;
      ++s_framesDropped;
    }
  }
}

// ============================== Статистика ============================
static void printStats() {
  portENTER_CRITICAL(&s_mux);
  const uint64_t samples = s_sampleIndex;
  portEXIT_CRITICAL(&s_mux);

  static uint32_t lastMs = 0;
  static uint32_t lastFrames = 0;

  const uint32_t now = millis();
  const uint32_t dt = now - lastMs;
  const uint32_t df = streamer.framesSent() - lastFrames;
  lastMs = now;
  lastFrames = streamer.framesSent();

  Serial.printf("[stat] кадров/с %lu | отправлено %lu | потеряно %lu | "
                "сэмплов %lu | пик %ld | WiFi %d | heap %u\n",
                dt ? (unsigned long)((uint64_t)df * 1000 / dt) : 0UL,
                (unsigned long)streamer.framesSent(),
                (unsigned long)s_framesDropped,
                (unsigned long)(samples & 0xFFFFFFFFULL),
                (long)s_peak,
                WiFi.RSSI(),
                (unsigned)ESP.getFreeHeap());

  const uint32_t age = now - s_lastDataMs;
  // s_lastDataMs пишет аудио-задача: без знака гонка даёт «данных нет 4294967288 мс».
  const int32_t ageSigned = (int32_t)age;

  if (ageSigned > 3000) {
#if AUDIO_SOURCE_TEST_TONE
    Serial.printf("[i2s] данных нет %lu мс\n", (unsigned long)ageSigned);
#else
#if AUDIO_CLOCK_DIAG
    // PCNT переводит измеряемые GPIO в режим входа, то есть на время замера
    // глушит выходы BCK/LRCK/MCLK — по умолчанию замер отключён.
    const AudioClocks clk = audio_measure_clocks(2);
    Serial.printf("[i2s] данных нет %ld мс | MCLK %lu | BCK %lu | LRCK %lu Гц\n",
                  (long)ageSigned,
                  (unsigned long)clk.mclk_hz,
                  (unsigned long)clk.bck_hz, (unsigned long)clk.lrck_hz);
#else
    Serial.printf("[i2s] данных нет %ld мс\n", (long)ageSigned);
#endif
#endif
  } else if (s_peak == 0) {
    Serial.printf("[i2s] кадры идут, но все сэмплы нулевые: DOUT -> GPIO%d", PIN_I2S_DATA);
#if I2S_ESP32_MASTER
    Serial.printf(", SCKI -> GPIO%d", PIN_I2S_MCLK);
#endif
    Serial.println(", проверьте вход АЦП");

    // Первые три раза показываем сырые слоты: 00000000 — линия молчит
    // (DOUT/SCKI не туда), иные значения — АЦП отдаёт данные, но тишину.
    static uint8_t zeroDumps = 0;
    if (zeroDumps < 3) {
      ++zeroDumps;
      audio_dump_raw(8);
    }
  }
}

// ========================= Проверка тактирования =======================
#if !AUDIO_SOURCE_TEST_TONE && AUDIO_CLOCK_DIAG
static void audioPrintClocks() {
  const AudioClocks clk = audio_measure_clocks(2);
  Serial.printf("[i2s] MCLK %lu Гц, BCK %lu Гц, LRCK %lu Гц\n",
                (unsigned long)clk.mclk_hz,
                (unsigned long)clk.bck_hz, (unsigned long)clk.lrck_hz);

  if (clk.lrck_hz == 0) {
#if I2S_ESP32_MASTER
    Serial.printf("[i2s] ESP32 не выдал BCK/LRCK (MCLK -> GPIO%d) — см. ошибки выше\n",
                  PIN_I2S_MCLK);
#else
    Serial.println("[i2s] НЕТ тактирования (BCK/LRCK молчат):");
    Serial.printf("      BCK -> GPIO%d, LRCK -> GPIO%d, DOUT -> GPIO%d, GND общий\n",
                  PIN_I2S_BCK, PIN_I2S_LRCK, PIN_I2S_DATA);
    Serial.println("      PCM1808 должен быть МАСТЕРОМ шины: кварц на модуле,");
    Serial.println("      перемычки MD0/MD1 — master, FMT — I2S.");
    Serial.println("      Если модуль в slave — I2S_ESP32_MASTER 1 и MCLK -> SCKI.");
#endif
    return;
  }

  const uint32_t bitsPerFrame = clk.bck_hz / clk.lrck_hz;
  Serial.printf("[i2s] фактическая fs %lu Гц, %lu бит/кадр\n",
                (unsigned long)clk.lrck_hz, (unsigned long)bitsPerFrame);

  if (clk.mclk_hz != 0) {
    const uint32_t sckiRatio = clk.mclk_hz / clk.lrck_hz;
    Serial.printf("[i2s] SCKI = %lu fs\n", (unsigned long)sckiRatio);
    if (sckiRatio != 256 && sckiRatio != 384 && sckiRatio != 512) {
      Serial.println("[i2s] ВНИМАНИЕ: PCM1808 ждёт SCKI 256/384/512 fs");
    }
  }

  const int32_t diff = (int32_t)clk.lrck_hz - (int32_t)SAMPLE_RATE;
  if (diff > (int32_t)(SAMPLE_RATE / 50) || diff < -(int32_t)(SAMPLE_RATE / 50)) {
    Serial.printf("[i2s] ВНИМАНИЕ: fs != SAMPLE_RATE (%d) — поправьте SAMPLE_RATE\n",
                  SAMPLE_RATE);
    Serial.println("      и запустите сервер с --sample-rate под фактическую fs");
  }
}
#endif

// ================================ setup ===============================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== radio-record streamer ===");
  Serial.printf("[sys] устройство %s, %lu Гц, %d кан., кадр %d мс\n",
                DEVICE_ID, (unsigned long)SAMPLE_RATE, CHANNELS, FRAME_MS);

  tunerInit();
  wifiConnect();

  if (!audio_init()) {
    Serial.println("[i2s] останов: I2S не инициализирован");
    for (;;) delay(1000);
  }

#if !AUDIO_SOURCE_TEST_TONE
  // Один холостой read запускает генератор клоков в master-режиме.
  static int16_t bootBuf[FRAME_SAMPLES * CHANNELS];
  audio_read(bootBuf, FRAME_SAMPLES, 50);
#if AUDIO_CLOCK_DIAG
  audioPrintClocks();
#endif
#endif

  // Аудио — на core 1 (там же, где loop, но с более высоким приоритетом),
  // Wi-Fi-стек живёт на core 0 и не мешает чтению I2S.
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 5, nullptr, 1);
  Serial.println("[sys] аудио-задача запущена");
}

void loop() {
  wifiKeepAlive();
  printStats();
  delay(5000);
}
