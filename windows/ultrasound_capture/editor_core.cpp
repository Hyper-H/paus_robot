#include "editor_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uscapture {
namespace {

struct Edges {
    bool west = false, north = false, east = false, south = false, move = false;
};

std::optional<Edges> selected_edges(RoiTarget target) {
    switch (target) {
    case RoiTarget::NW: return Edges{true, true, false, false, false};
    case RoiTarget::N: return Edges{false, true, false, false, false};
    case RoiTarget::NE: return Edges{false, true, true, false, false};
    case RoiTarget::E: return Edges{false, false, true, false, false};
    case RoiTarget::SE: return Edges{false, false, true, true, false};
    case RoiTarget::S: return Edges{false, false, false, true, false};
    case RoiTarget::SW: return Edges{true, false, false, true, false};
    case RoiTarget::W: return Edges{true, false, false, false, false};
    case RoiTarget::Move: return Edges{false, false, false, false, true};
    }
    return std::nullopt;
}

std::optional<Rect> from_boundaries(std::int64_t left, std::int64_t top,
                                    std::int64_t right, std::int64_t bottom,
                                    int frame_width, int frame_height) {
    if (left < 0 || top < 0 || right > frame_width || bottom > frame_height ||
        left >= right || top >= bottom)
        return std::nullopt;
    return Rect{static_cast<int>(left), static_cast<int>(top),
                static_cast<int>(right - left), static_cast<int>(bottom - top)};
}

int anchored(int origin, int extent, double anchor) {
    return static_cast<int>(static_cast<std::int64_t>(origin) +
                            std::llround(static_cast<double>(extent) * anchor));
}

} // namespace

std::optional<Rect> nudge_roi(const Rect& roi, int frame_width, int frame_height,
                              RoiTarget target, int dx, int dy) {
    const auto edges = selected_edges(target);
    if (!edges || !valid_rect(roi, frame_width, frame_height)) return std::nullopt;
    if ((dx != 0 && !(edges->west || edges->east || edges->move)) ||
        (dy != 0 && !(edges->north || edges->south || edges->move)))
        return std::nullopt;

    std::int64_t left = roi.x, top = roi.y;
    std::int64_t right = left + roi.width, bottom = top + roi.height;
    if (edges->west || edges->move) left += dx;
    if (edges->east || edges->move) right += dx;
    if (edges->north || edges->move) top += dy;
    if (edges->south || edges->move) bottom += dy;
    return from_boundaries(left, top, right, bottom, frame_width, frame_height);
}

std::optional<Rect> drag_roi(const Rect& roi, int frame_width, int frame_height,
                             RoiTarget target, int dx, int dy) {
    const auto edges = selected_edges(target);
    if (!edges || !valid_rect(roi, frame_width, frame_height)) return std::nullopt;
    std::int64_t left = roi.x, top = roi.y;
    std::int64_t right = left + roi.width, bottom = top + roi.height;
    if (edges->move) {
        left = std::clamp<std::int64_t>(left + dx, 0, frame_width - roi.width);
        top = std::clamp<std::int64_t>(top + dy, 0, frame_height - roi.height);
        right = left + roi.width;
        bottom = top + roi.height;
    } else {
        if (edges->west) left = std::clamp<std::int64_t>(left + dx, 0, right - 1);
        if (edges->east) right = std::clamp<std::int64_t>(right + dx, left + 1, frame_width);
        if (edges->north) top = std::clamp<std::int64_t>(top + dy, 0, bottom - 1);
        if (edges->south) bottom = std::clamp<std::int64_t>(bottom + dy, top + 1, frame_height);
    }
    return from_boundaries(left, top, right, bottom, frame_width, frame_height);
}

PixelPoint inspection_point(const Rect& roi, RoiTarget target, double anchor_x, double anchor_y) {
    if (!std::isfinite(anchor_x) || !std::isfinite(anchor_y) ||
        anchor_x < 0 || anchor_x > 1 || anchor_y < 0 || anchor_y > 1)
        throw std::invalid_argument("Inspection anchors must be finite values between zero and one");
    if (!valid_rect(roi, std::numeric_limits<int>::max(), std::numeric_limits<int>::max()))
        throw std::invalid_argument("Inspection needs a nonempty rectangle with representable boundaries");
    const auto edges = selected_edges(target);
    if (!edges) throw std::invalid_argument("Unknown ROI inspection target");
    if (edges->move)
        return {anchored(roi.x, roi.width - 1, anchor_x), anchored(roi.y, roi.height - 1, anchor_y)};
    const int x = edges->west ? roi.x : edges->east ? roi.x + roi.width : anchored(roi.x, roi.width, anchor_x);
    const int y = edges->north ? roi.y : edges->south ? roi.y + roi.height : anchored(roi.y, roi.height, anchor_y);
    return {x, y};
}

} // namespace uscapture
