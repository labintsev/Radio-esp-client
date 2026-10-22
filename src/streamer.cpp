#include "streamer.h"

#include <stdlib.h>
#include <string.h>

#include "config.h"

static void copyDeviceId(char (&dst)[16]) {
  size_t len = strlen(DEVICE_ID);
  if (len > sizeof dst - 1) len = sizeof dst - 1;
  memcpy(dst, DEVICE_ID, len);
  memset(dst + len, 0, sizeof dst - len);
}

// Принимает "host", "host/", "http://host/path", "host:9100" —
// возвращает чистое имя и порт (порт из строки важнее SERVER_PORT).
static bool resolveTarget(char *host, size_t hostSize, uint16_t *port) {
  const char *s = SERVER_HOST;
  if (strncmp(s, "http://", 7) == 0) s += 7;
  else if (strncmp(s, "https://", 8) == 0) s += 8;

  char buf[96];
  size_t n = 0;
  while (*s && *s != '/' && *s != ' ') {
    if (n + 1 >= sizeof buf) return false;
    buf[n++] = *s++;
  }
  buf[n] = '\0';
  if (n == 0) return false;

  *port = SERVER_PORT;
  char *colon = strrchr(buf, ':');
  if (colon != nullptr && colon[1] != '\0') {
    bool digits = true;
    for (const char *p = colon + 1; *p; ++p) {
      if (*p < '0' || *p > '9') { digits = false; break; }
    }
    if (digits) {
      *port = (uint16_t)atoi(colon + 1);
      *colon = '\0';
    }
  }

  if (strlen(buf) >= hostSize) return false;
  strcpy(host, buf);
  return true;
}

bool AudioStreamer::connect() {
  disconnect();

  char host[64];
  uint16_t port = 0;
  if (!resolveTarget(host, sizeof host, &port)) {
    Serial.println("[net] некорректный SERVER_HOST");
    return false;
  }

  Serial.printf("[net] подключение к %s:%u ... ", host, (unsigned)port);
  const uint32_t t0 = millis();
  if (!client_.connect(host, port, CONNECT_TIMEOUT_MS)) {
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
