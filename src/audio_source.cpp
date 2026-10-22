#include "audio_source.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"

#if AUDIO_SOURCE_TEST_TONE

// ==================== Генератор синуса (отладка без ADC) ====================
static float s_phase = 0.0f;

bool audio_init() { return true; }

size_t audio_read(int16_t *dst, size_t frames, uint32_t timeout_ms) {
  (void)timeout_ms;
  const float twoPi = 2.0f * (float)M_PI;
  const float step = twoPi * (float)TEST_TONE_HZ / (float)SAMPLE_RATE;
  for (size_t f = 0; f < frames; ++f) {
    const int16_t v = (int16_t)(sinf(s_phase) * 6000.0f);
    s_phase += step;
    if (s_phase > twoPi) s_phase -= twoPi;
    for (int c = 0; c < CHANNELS; ++c) dst[f * CHANNELS + c] = v;
  }
  vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
  return frames;
}

AudioClocks audio_measure_clocks(uint32_t window_ms) {
  (void)window_ms;
  return {0, 0, 0};
}

#else

#include <driver/i2s.h>
#include <driver/pcnt.h>

static const i2s_port_t kPort = I2S_NUM_0;

// Сырые 32-битные слоты I2S. PCM1808 отдаёт 24 значащих бита, выровненных влево.
static int32_t s_raw[FRAME_SAMPLES * I2S_CHANNELS];

static esp_err_t i2sInstall(bool useApll) {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)((I2S_ESP32_MASTER ? I2S_MODE_MASTER : I2S_MODE_SLAVE) | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S);
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = useApll;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = useApll ? (SAMPLE_RATE * 256) : 0;

  esp_err_t err = i2s_driver_install(kPort, &cfg, 0, nullptr);
  if (err != ESP_OK) return err;

  i2s_pin_config_t pins = {};
  pins.mck_io_num   = I2S_ESP32_MASTER ? PIN_I2S_MCLK : I2S_PIN_NO_CHANGE;
  pins.bck_io_num   = PIN_I2S_BCK;
  pins.ws_io_num    = PIN_I2S_LRCK;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = PIN_I2S_DATA;

  err = i2s_set_pin(kPort, &pins);
  if (err == ESP_OK && I2S_ESP32_MASTER) {
    err = i2s_set_clk(kPort, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
  }
  if (err != ESP_OK) i2s_driver_uninstall(kPort);
  return err;
}

bool audio_init() {
  esp_err_t err;
#if I2S_ESP32_MASTER
  err = i2sInstall(true);          // MCLK ровно 256*fs через APLL
  if (err != ESP_OK) {
    Serial.printf("[i2s] APLL %lu Гц не взят (%s), пробую PLL_D2\n",
                  (unsigned long)(SAMPLE_RATE * 256), esp_err_to_name(err));
    err = i2sInstall(false);
  }
#else
  err = i2sInstall(false);
#endif
  if (err != ESP_OK) {
    Serial.printf("[i2s] инициализация: %s\n", esp_err_to_name(err));
    return false;
  }

  i2s_zero_dma_buffer(kPort);
  return true;
}

size_t audio_read(int16_t *dst, size_t frames, uint32_t timeout_ms) {
  if (frames > FRAME_SAMPLES) frames = FRAME_SAMPLES;

  const size_t want = frames * I2S_CHANNELS * sizeof(int32_t);
  size_t got = 0;
  const esp_err_t err = i2s_read(kPort, s_raw, want, &got, pdMS_TO_TICKS(timeout_ms));
  if (err != ESP_OK || got < I2S_CHANNELS * sizeof(int32_t)) return 0;

  const size_t n = got / (I2S_CHANNELS * sizeof(int32_t));
  for (size_t f = 0; f < n; ++f) {
    const int32_t l = s_raw[f * I2S_CHANNELS + 0] >> 16;   // 24 бита -> 16 бит
#if CHANNELS == 1
    const int32_t r = s_raw[f * I2S_CHANNELS + 1] >> 16;
    dst[f] = (int16_t)((l + r) / 2);                       // даунмикс L+R
#else
    dst[f * 2 + 0] = (int16_t)l;
    dst[f * 2 + 1] = (int16_t)(s_raw[f * I2S_CHANNELS + 1] >> 16);
#endif
  }
  return n;
}

// Счётчик импульсов: даёт понять, приходит ли вообще тактирование.
// Окно короткое, чтобы 16-битный счётчик не переполнился на 12 МГц.
static esp_err_t pcntStart(pcnt_unit_t unit, int pin) {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = pin;
  cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  cfg.lctrl_mode     = PCNT_MODE_KEEP;
  cfg.hctrl_mode     = PCNT_MODE_KEEP;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DIS;
  cfg.counter_h_lim  = 32000;
  cfg.counter_l_lim  = 0;
  cfg.unit           = unit;
  cfg.channel        = PCNT_CHANNEL_0;

  esp_err_t err = pcnt_unit_config(&cfg);
  if (err != ESP_OK) return err;

  pcnt_filter_disable(unit);          // фильтр срезал бы мегагерцы
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
  return ESP_OK;
}

static uint32_t pcntStop(pcnt_unit_t unit, uint32_t window_ms) {
  int16_t count = 0;
  pcnt_counter_pause(unit);
  pcnt_get_counter_value(unit, &count);
  return (uint32_t)count * 1000u / window_ms;
}

AudioClocks audio_measure_clocks(uint32_t window_ms) {
  AudioClocks out{0, 0, 0};
  if (window_ms == 0) window_ms = 2;
  const uint32_t slowWindow = window_ms * 10;   // LRCK медленный — окно длиннее

  if (pcntStart(PCNT_UNIT_0, PIN_I2S_BCK) != ESP_OK ||
      pcntStart(PCNT_UNIT_1, PIN_I2S_LRCK) != ESP_OK ||
      pcntStart(PCNT_UNIT_2, PIN_I2S_MCLK) != ESP_OK) {
    return out;
  }

  delay(window_ms);
  out.bck_hz  = pcntStop(PCNT_UNIT_0, window_ms);
  out.mclk_hz = pcntStop(PCNT_UNIT_2, window_ms);
  delay(slowWindow - window_ms);
  out.lrck_hz = pcntStop(PCNT_UNIT_1, slowWindow);
  return out;
}

#endif  // AUDIO_SOURCE_TEST_TONE
