// dmd_reader.h
#ifndef DMD_READER_H
#define DMD_READER_H

static bool pin_is_stably_high(uint pin, uint32_t stable_ms, uint32_t sample_ms, uint32_t timeout_ms);
int read_dmd();

#endif
