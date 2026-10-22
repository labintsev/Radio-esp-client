#pragma once

#include <Arduino.h>
#include <WiFiClient.h>
#include <stddef.h>
#include <stdint.h>

// Заголовок кадра протокола radio-record-server: 43 байта, little endian.
#pragma pack(push, 1)
struct FrameHeader {
  char     magic[4];        // "RDIO"
  uint8_t  version;         // 1
  uint8_t  codec;           // 1 = PCM s16le
  char     device_id[16];   // ASCII, дополнен \0
  uint32_t seq;             // счётчик кадров соединения
  uint64_t first_sample;    // абсолютный номер первого сэмпла
  uint32_t payload_len;     // размер payload, 1..65536
  uint32_t sample_rate;     // сэмплов/с на канал
  uint8_t  channels;        // 1 или 2
};
#pragma pack(pop)
static_assert(sizeof(FrameHeader) == 43, "формат заголовка должен совпадать с сервером");

class AudioStreamer {
 public:
  // WiFiClient::connected() не const, поэтому и метод не const.
  bool connected() { return client_.connected(); }

  // Одна попытка подключения. При успехе сбрасывает seq (он на соединение).
  bool connect();

  void disconnect();

  // Отправляет один кадр (заголовок + PCM). false — соединение потеряно.
  bool sendFrame(const int16_t *pcm, size_t frames, uint64_t firstSample);

  uint32_t framesSent() const { return framesSent_; }
  uint32_t connectCount() const { return connectCount_; }

 private:
  WiFiClient client_;
  uint32_t seq_ = 0;
  uint32_t framesSent_ = 0;
  uint32_t connectCount_ = 0;
};
