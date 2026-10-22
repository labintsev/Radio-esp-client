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
  return {0, 0};
}

#else

#include <driver/i2s.h>
#include <driver/pcnt.h>

static const i2s_port_t kPort = I2S_NUM_0;

// Сырые 32-битные слоты. PCM1808 отдаёт 24 значащих бита, выровненных влево.
static int32_t s_raw[FRAME_SAMPLES * CHANNELS];

bool audio_init() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)((I2S_ESP32_MASTER ? I2S_MODE_MASTER : I2S_MODE_SLAVE) | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S);
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = (I2S_ESP32_MASTER != 0);
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = I2S_ESP32_MASTER ? (SAMPLE_RATE * 256) : 0;

  esp_err_t err = i2s_driver_install(kPort, &cfg, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[i2s] driver_install: %s\n", esp_err_to_name(err));
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.mck_io_num   = I2S_ESP32_MASTER ? PIN_I2S_MCLK : I2S_PIN_NO_CHANGE;
  pins.bck_io_num   = PIN_I2S_BCK;
  pins.ws_io_num    = PIN_I2S_LRCK;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = PIN_I2S_DATA;

  err = i2s_set_pin(kPort, &pins);
  if (err != ESP_OK) {
    Serial.printf("[i2s] set_pin: %s\n", esp_err_to_name(err));
    i2s_driver_uninstall(kPort);
    return false;
  }

  if (I2S_ESP32_MASTER) {
    err = i2s_set_clk(kPort, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
    if (err != ESP_OK) {
      Serial.printf("[i2s] set_clk: %s\n", esp_err_to_name(err));
      i2s_driver_uninstall(kPort);
      return false;
    }
  }

  i2s_zero_dma_buffer(kPort);
  return true;
}

size_t audio_read(int16_t *dst, size_t frames, uint32_t timeout_ms) {
  if (frames > FRAME_SAMPLES) frames = FRAME_SAMPLES;

  const size_t want = frames * CHANNELS * sizeof(int32_t);
  size_t got = 0;
  const esp_err_t err = i2s_read(kPort, s_raw, want, &got, pdMS_TO_TICKS(timeout_ms));
  if (err != ESP_OK || got < CHANNELS * sizeof(int32_t)) return 0;

  // Целые кадры (L+R): got всегда кратен 8, т.к. запрашиваем ровно want.
  const size_t n = got / (CHANNELS * sizeof(int32_t));
  const size_t words = n * CHANNELS;
  for (size_t i = 0; i < words; ++i) {
    dst[i] = (int16_t)(s_raw[i] >> 16);   // 24 бита -> 16 бит
  }
  return n;
}

// Счётчик импульсов: даёт понять, приходит ли вообще тактирование.
// Окно короткое, чтобы 16-битный счётчик не переполнился на 3 МГц.
AudioClocks audio_measure_clocks(uint32_t window_ms) {
  if (window_ms == 0) window_ms = 4;

  pcnt_config_t bck = {};
  bck.pulse_gpio_num = PIN_I2S_BCK;
  bck.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  bck.lctrl_mode     = PCNT_MODE_KEEP;
  bck.hctrl_mode     = PCNT_MODE_KEEP;
  bck.pos_mode       = PCNT_COUNT_INC;
  bck.neg_mode       = PCNT_COUNT_DIS;
  bck.counter_h_lim  = 32000;
  bck.counter_l_lim  = 0;
  bck.unit           = PCNT_UNIT_0;
  bck.channel        = PCNT_CHANNEL_0;

  pcnt_config_t lrck = bck;
  lrck.pulse_gpio_num = PIN_I2S_LRCK;
  lrck.unit           = PCNT_UNIT_1;

  if (pcnt_unit_config(&bck) != ESP_OK || pcnt_unit_config(&lrck) != ESP_OK) {
    return {0, 0};
  }
  // Фильтр не включаем: 3 МГц он бы срезал.
  pcnt_filter_disable(PCNT_UNIT_0);
  pcnt_filter_disable(PCNT_UNIT_1);

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_1);

  delay(window_ms);

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_1);

  int16_t bckCount = 0;
  int16_t lrckCount = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &bckCount);
  pcnt_get_counter_value(PCNT_UNIT_1, &lrckCount);

  AudioClocks out;
  out.bck_hz  = (uint32_t)bckCount * 1000u / window_ms;
  out.lrck_hz = (uint32_t)lrckCount * 1000u / window_ms;
  return out;
}

#endif  // AUDIO_SOURCE_TEST_TONE
