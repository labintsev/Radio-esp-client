#include "streamer.h"

#include <string.h>

#include "config.h"

static void copyDeviceId(char (&dst)[16]) {
  size_t len = strlen(DEVICE_ID);
  if (len > sizeof dst - 1) len = sizeof dst - 1;
  memcpy(dst, DEVICE_ID, len);
  memset(dst + len, 0, sizeof dst - len);
}

bool AudioStreamer::connect() {
  disconnect();

  Serial.printf("[net] подключение к %s:%u ... ", SERVER_HOST, (unsigned)SERVER_PORT);
  const uint32_t t0 = millis();
  if (!client_.connect(SERVER_HOST, SERVER_PORT, CONNECT_TIMEOUT_MS)) {
    Serial.println("не удалось");
    return false;
  }

  client_.setNoDelay(true);              // иначе Nagle склеивает кадры
  client_.setTimeout(SOCKET_TIMEOUT_MS); // ограничивает блокировку на write
  seq_ = 0;                              // seq считается в пределах соединения

  Serial.printf("ok (%lu мс)\n", (unsigned long)(millis() - t0));
  return true;
}

void AudioStreamer::disconnect() {
  if (client_.connected()) client_.stop();
}

bool AudioStreamer::sendFrame(const int16_t *pcm, size_t frames, uint64_t firstSample) {
  const size_t payload = frames * CHANNELS * sizeof(int16_t);
  if (payload == 0 || payload > 65536) return false;

  FrameHeader h{};
  memcpy(h.magic, "RDIO", 4);
  h.version      = 1;
  h.codec        = 1;                    // PCM signed 16-bit LE
  copyDeviceId(h.device_id);
  h.seq          = seq_++;
  h.first_sample = firstSample;
  h.payload_len  = (uint32_t)payload;
  h.sample_rate  = SAMPLE_RATE;
  h.channels     = CHANNELS;

  if (client_.write(reinterpret_cast<const uint8_t *>(&h), sizeof h) != sizeof h) {
    return false;
  }
  if (client_.write(reinterpret_cast<const uint8_t *>(pcm), payload) != payload) {
    return false;
  }

  ++framesSent_;
  return true;
}
