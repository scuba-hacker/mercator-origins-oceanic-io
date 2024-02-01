#pragma once

#include <array>
#include <cstddef>

const int lilyGoT4FullScreenBufferSize = 270000;        

// MBJ REFACTOR - how to get rid of lilyGoT4FullScreenBufferSize here? c.f. C array []

extern const std::array<uint16_t, lilyGoT4FullScreenBufferSize> lily_wraysbury_N;
extern const std::array<uint16_t, lilyGoT4FullScreenBufferSize> lily_wraysbury_W;
extern const std::array<uint16_t, lilyGoT4FullScreenBufferSize> lily_wraysbury_SW;
extern const std::array<uint16_t, lilyGoT4FullScreenBufferSize> lily_wraysbury_S;
extern const std::array<uint16_t, lilyGoT4FullScreenBufferSize> lily_wraysbury_SE;
extern const std::array<uint16_t, lilyGoT4FullScreenBufferSize> lily_wraysbury_All;

