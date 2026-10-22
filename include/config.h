#pragma once

// ============================== Wi-Fi ==============================
#define WIFI_SSID       "OpenWrt9"
#define WIFI_PASSWORD   "21051986"

// ============================== Сервер =============================
// Адрес машины, где запущен radio_record. Схема, путь и порт в строке
// отбрасываются автоматически, но лучше писать только имя/IP.
#define SERVER_HOST     "radio-record.mldl.ru"
#define SERVER_PORT     19100

// Имя устройства: попадает в заголовок кадра и в имя каталога архива.
// Максимум 15 символов (поле device_id в протоколе — 16 байт с \0).
#define DEVICE_ID       "moscow-autoradio"

// =============================== Аудио =============================
// PCM1808 в режиме мастера тактируется от собственного кварца 12.288 МГц,
// поэтому доступны 48/32/24 кГц (12.288M / 256, 384, 512 fs), но не 16 кГц.
// Сервер запускайте с теми же параметрами:
//   python -m radio_record --port 9100 --sample-rate 48000 --channels 2
#define SAMPLE_RATE     48000
#define CHANNELS        2
#define FRAME_MS        20
#define FRAME_SAMPLES   (SAMPLE_RATE * FRAME_MS / 1000)   // 960 кадров на канал

// Кто тактирует шину I2S:
//   0 — PCM1808 (на модуле свой кварц, BCK/LRCK идут в ESP32, ESP32 — slave);
//   1 — ESP32 (генерирует BCK/LRCK и MCLK 256*fs = 12.288 МГц для PCM1808).
// Для модуля со входом SCKI без своего кварца нужен режим 1:
// MCLK -> SCKI, а MD0/MD1/FMT модуля посадить на GND (slave, I2S, 24 бита).
#define I2S_ESP32_MASTER 1

// =============================== Пины ==============================
#define PIN_I2S_BCK     26      // -> BCK  (PCM1808)
#define PIN_I2S_LRCK    25      // -> LRCK (PCM1808)
#define PIN_I2S_DATA    33      // <- DOUT (PCM1808)
#define PIN_I2S_MCLK    1       // -> SCKI (только при I2S_ESP32_MASTER 1)
                                
#define PIN_TUNER_RESET 12      // GPIO12 — strapping-пин, см. README
#define PIN_TUNER_SDA   21
#define PIN_TUNER_SCL   22

// =============================== Тюнер =============================
#define TUNER_FREQUENCY 9030    // 90.3 МГц, шаг 10 кГц
#define TUNER_VOLUME    10
#define TUNER_MONO      true

// ============================== Сеть ===============================
#define CONNECT_TIMEOUT_MS  3000    // таймаут установки TCP-соединения
#define SOCKET_TIMEOUT_MS   2000    // таймаут записи (backpressure сервера)
#define RECONNECT_DELAY_MS  3000    // пауза между попытками переподключения

// ============================ Отладка ==============================
// 1 — вместо I2S генерируется синус TEST_TONE_HZ (проверка пути до сервера).
#define AUDIO_SOURCE_TEST_TONE 0
#define TEST_TONE_HZ            1000
