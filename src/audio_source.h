#pragma once

#include <stddef.h>
#include <stdint.h>

// Частоты на пинах BCK и LRCK (нули — тактирования нет).
struct AudioClocks {
  uint32_t bck_hz;
  uint32_t lrck_hz;
};

// Поднимает I2S на приём: 48 кГц, стерео, слот 32 бита (PCM1808 отдаёт 24).
// Возвращает false, если драйвер не установился.
bool audio_init();

// Читает не более `frames` кадров (сэмплов на канал) в dst.
// dst — interleaved L,R,..., int16. Возвращает число прочитанных кадров,
// 0 при таймауте (нет клока от PCM1808 / тишина в DMA).
size_t audio_read(int16_t *dst, size_t frames, uint32_t timeout_ms);

// Измеряет частоты на линиях BCK/LRCK счётчиком импульсов (PCNT).
// Вызывать после audio_init(): в master-режиме ESP32 генерирует клоки только
// после первого запуска чтения.
AudioClocks audio_measure_clocks(uint32_t window_ms);
