#pragma once

#include <stddef.h>
#include <stdint.h>

// Поднимает I2S на приём: 48 кГц, стерео, слот 32 бита (PCM1808 отдаёт 24).
// Возвращает false, если драйвер не установился.
bool audio_init();

// Читает не более `frames` кадров (сэмплов на канал) в dst.
// dst — interleaved L,R,..., int16. Возвращает число прочитанных кадров,
// 0 при таймауте (нет клока от PCM1808 / тишина в DMA).
size_t audio_read(int16_t *dst, size_t frames, uint32_t timeout_ms);
