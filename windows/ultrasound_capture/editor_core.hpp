#pragma once

#include "capture_app.hpp"

namespace uscapture {

enum class RoiTarget { NW, N, NE, E, SE, S, SW, W, Move };

struct PixelPoint {
    int x = 0, y = 0;
};

// Exact pixel changes. Reject invalid input, unsupported axes, or an out-of-bounds result.
std::optional<Rect> nudge_roi(const Rect& roi, int frame_width, int frame_height,
                              RoiTarget target, int dx, int dy);

// Drag deltas are relative to the rectangle at pointer-down. Clamp at the frame
// and minimum size; ignore motion tangential to a selected edge.
std::optional<Rect> drag_roi(const Rect& roi, int frame_width, int frame_height,
                             RoiTarget target, int dx, int dy);

// Edges/corners use half-open rectangle boundaries, including right and bottom.
// Move uses an actual pixel inside the rectangle. Anchors must be finite in [0,1].
PixelPoint inspection_point(const Rect& roi, RoiTarget target,
                            double anchor_x = 0.5, double anchor_y = 0.5);

} // namespace uscapture
