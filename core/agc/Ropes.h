// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#ifndef COLUMBIA_ROPES_H
#define COLUMBIA_ROPES_H

#include <cstddef>
#include <cstdint>

namespace agc {

// Comanche 055 rope image in the standard .bin layout (36 banks).
extern const uint8_t kComanche055Rope[73728];
constexpr size_t kComanche055RopeSize = 73728;

}  // namespace agc

#endif
