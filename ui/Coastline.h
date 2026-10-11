// Columbia — world coastlines (Natural Earth 1:110m, public domain).
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#ifndef COLUMBIA_UI_COASTLINE_H
#define COLUMBIA_UI_COASTLINE_H

#include <cstdint>

namespace ui {

// Points (latitude, longitude) in 1/100 degree; line i runs from
// kCoastLineStarts[i] to kCoastLineStarts[i + 1] - 1.
extern const int16_t kCoastPoints[][2];
extern const uint16_t kCoastLineStarts[];
extern const int kCoastLineCount;

}  // namespace ui

#endif
