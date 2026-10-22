#include "editor_core.hpp"

#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace uscapture;
constexpr std::array<RoiTarget, 9> kTargets{
    RoiTarget::NW, RoiTarget::N, RoiTarget::NE, RoiTarget::E, RoiTarget::SE,
    RoiTarget::S, RoiTarget::SW, RoiTarget::W, RoiTarget::Move};

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool same(const Rect& a, const Rect& b) {
    return a.x == b.x && a.y == b.y && a.width == b.width && a.height == b.height;
}

void expect(const std::optional<Rect>& actual, Rect expected, const char* message) {
    check(actual && same(*actual, expected), message);
}

void point(PixelPoint actual, int x, int y, const char* message) {
    check(actual.x == x && actual.y == y, message);
}

template <typename Function> void rejects(Function function, const char* message) {
    try { function(); }
    catch (const std::invalid_argument&) { return; }
    throw std::runtime_error(message);
}

void exact_changes() {
    const Rect initial{2, 3, 5, 4};
    for (const auto target : kTargets) {
        expect(nudge_roi(initial, 10, 10, target, 0, 0), initial, "Selecting a handle changed the ROI");
        expect(drag_roi(initial, 10, 10, target, 0, 0), initial, "Zero drag changed the ROI");
    }
    expect(nudge_roi(initial, 10, 10, RoiTarget::W, 1, 0), {3, 3, 4, 4}, "West one-pixel nudge wrong");
    expect(nudge_roi(initial, 10, 10, RoiTarget::E, 1, 0), {2, 3, 6, 4}, "East one-pixel nudge wrong");
    expect(nudge_roi(initial, 10, 10, RoiTarget::N, 0, 1), {2, 4, 5, 3}, "North one-pixel nudge wrong");
    expect(nudge_roi(initial, 10, 10, RoiTarget::S, 0, 1), {2, 3, 5, 5}, "South one-pixel nudge wrong");
    expect(nudge_roi(initial, 10, 10, RoiTarget::NW, 1, 1), {3, 4, 4, 3}, "NW one-pixel nudge wrong");
    expect(nudge_roi(initial, 10, 10, RoiTarget::NE, 1, 1), {2, 4, 6, 3}, "NE one-pixel nudge wrong");
    expect(nudge_roi(initial, 10, 10, RoiTarget::SE, 1, 1), {2, 3, 6, 5}, "SE one-pixel nudge wrong");
    expect(nudge_roi(initial, 10, 10, RoiTarget::SW, 1, 1), {3, 3, 4, 5}, "SW one-pixel nudge wrong");
    expect(nudge_roi(initial, 10, 10, RoiTarget::Move, -1, 2), {1, 5, 5, 4}, "Move resized the ROI");
    for (const auto target : {RoiTarget::N, RoiTarget::S})
        check(!nudge_roi(initial, 10, 10, target, 1, 0), "Horizontal control enabled for horizontal edge");
    for (const auto target : {RoiTarget::W, RoiTarget::E})
        check(!nudge_roi(initial, 10, 10, target, 0, 1), "Vertical control enabled for vertical edge");
    check(!nudge_roi(initial, 10, 10, RoiTarget::N, 1, 1), "Unsupported diagonal nudge partially applied");
    expect(drag_roi(initial, 10, 10, RoiTarget::N, 100, 1), {2, 4, 5, 3}, "Drag did not ignore irrelevant axis");
    expect(drag_roi(initial, 10, 10, RoiTarget::W, 1, 100), {3, 3, 4, 4}, "Drag moved an unrelated edge");
}

void limits_and_inspection() {
    const Rect single{4, 3, 1, 1};
    check(!nudge_roi(single, 5, 4, RoiTarget::E, 1, 0), "Right boundary exceeded");
    check(!nudge_roi(single, 5, 4, RoiTarget::S, 0, 1), "Bottom boundary exceeded");
    check(!nudge_roi(single, 5, 4, RoiTarget::W, 1, 0), "Zero-width ROI accepted");
    check(!nudge_roi(single, 5, 4, RoiTarget::N, 0, 1), "Zero-height ROI accepted");
    expect(nudge_roi(single, 5, 4, RoiTarget::W, -1, 0), {3, 3, 2, 1}, "One-pixel ROI did not expand correctly");
    expect(drag_roi(single, 5, 4, RoiTarget::NW, 100, 100), single, "Drag violated minimum one-pixel size");
    expect(drag_roi(single, 5, 4, RoiTarget::Move, 100, 100), single, "Drag moved beyond final pixel");
    expect(drag_roi(single, 5, 4, RoiTarget::Move, -100, -100), {0, 0, 1, 1}, "Move not clamped at origin");
    point(inspection_point(single, RoiTarget::SE), 5, 4, "Right/bottom boundary coordinates incorrectly clipped");
    point(inspection_point(single, RoiTarget::Move, 1, 1), 4, 3, "Move inspection exceeded final image pixel");
    point(inspection_point({2, 3, 5, 4}, RoiTarget::NW), 2, 3, "NW inspection incorrect");
    point(inspection_point({2, 3, 5, 4}, RoiTarget::NE), 7, 3, "NE inspection incorrect");
    point(inspection_point({2, 3, 5, 4}, RoiTarget::SE), 7, 7, "SE inspection incorrect");
    point(inspection_point({2, 3, 5, 4}, RoiTarget::SW), 2, 7, "SW inspection incorrect");
    point(inspection_point({2, 3, 5, 4}, RoiTarget::N, 0, 1), 2, 3, "North anchor incorrectly used vertical position");
    point(inspection_point({2, 3, 5, 4}, RoiTarget::S, 1, 0), 7, 7, "South anchor missed boundary endpoint");
    point(inspection_point({2, 3, 5, 4}, RoiTarget::W, 1, 0.5), 2, 5, "West anchor incorrect");
    point(inspection_point({2, 3, 5, 4}, RoiTarget::E, 0, 1), 7, 7, "East anchor incorrect");
    point(inspection_point({2, 3, 5, 4}, RoiTarget::Move, 1, 1), 6, 6, "Move anchor used boundary instead of pixel");
    const auto invalid_target = static_cast<RoiTarget>(99);
    check(!drag_roi(single, 5, 4, invalid_target, 0, 0), "Unknown drag target accepted");
    check(!nudge_roi(single, 5, 4, invalid_target, 0, 0), "Unknown nudge target accepted");
    for (const auto bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -0.1, 1.1}) {
        rejects([&] { inspection_point(single, RoiTarget::Move, bad, 0.5); }, "Invalid horizontal anchor accepted");
        rejects([&] { inspection_point(single, RoiTarget::N, 0.5, bad); }, "Invalid vertical anchor accepted");
    }
    rejects([&] { inspection_point(single, invalid_target); }, "Unknown inspection target accepted");
    rejects([] { inspection_point({0, 0, 0, 1}, RoiTarget::Move); }, "Empty inspection rectangle accepted");
    const int maximum = std::numeric_limits<int>::max();
    const Rect last{maximum - 1, maximum - 1, 1, 1};
    point(inspection_point(last, RoiTarget::SE), maximum, maximum, "Large boundary coordinate overflow");
    expect(drag_roi(last, maximum, maximum, RoiTarget::Move, maximum, maximum), last, "Large drag overflow");
    check(!nudge_roi(last, maximum, maximum, RoiTarget::Move, maximum, maximum), "Large nudge overflow accepted");
    rejects([&] { inspection_point({maximum - 1, 0, 2, 1}, RoiTarget::E); }, "Unrepresentable boundary accepted");
    check(!nudge_roi({0, 0, 0, 0}, 5, 4, RoiTarget::Move, 0, 0), "Invalid initial nudge rectangle accepted");
    check(!drag_roi({0, 0, 1, 1}, 0, 4, RoiTarget::Move, 0, 0), "Invalid frame dimensions accepted");
}

void exhaustive_small_frame() {
    const std::array<int, 7> deltas{-7, -1, 0, 1, 7, std::numeric_limits<int>::min(), std::numeric_limits<int>::max()};
    for (int left = 0; left < 5; ++left) for (int top = 0; top < 4; ++top)
    for (int right = left + 1; right <= 5; ++right) for (int bottom = top + 1; bottom <= 4; ++bottom) {
        const Rect roi{left, top, right - left, bottom - top};
        for (const auto target : kTargets) for (const int dx : deltas) for (const int dy : deltas) {
            const auto drag = drag_roi(roi, 5, 4, target, dx, dy);
            check(drag && valid_rect(*drag, 5, 4), "Drag escaped the frame or produced zero area");
            const auto nudge = nudge_roi(roi, 5, 4, target, dx, dy);
            check(!nudge || valid_rect(*nudge, 5, 4), "Nudge escaped the frame or produced zero area");
            if (nudge) check(same(*drag, *nudge), "Legal exact nudge differs from equivalent drag");
            if (target == RoiTarget::Move)
                check(drag->width == roi.width && drag->height == roi.height, "Move unexpectedly changed ROI size");
            if (dx == 0 && dy == 0) check(same(*drag, roi) && nudge && same(*nudge, roi), "Click-only interaction changed geometry");
        }
        for (const auto target : kTargets) for (const double x : {0.0, 0.5, 1.0}) for (const double y : {0.0, 0.5, 1.0}) {
            const auto p = inspection_point(roi, target, x, y);
            check(p.x >= left && p.y >= top && p.x <= right && p.y <= bottom, "Inspection point escaped ROI boundaries");
            if (target == RoiTarget::Move) check(p.x < right && p.y < bottom, "Move inspection is not a valid pixel");
        }
    }
}
} // namespace

int main() {
    try {
        exact_changes();
        limits_and_inspection();
        exhaustive_small_frame();
        std::cout << "editor_core_test: exact nudges, clamped drags, pixel edges and inspection anchors passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "editor_core_test: " << error.what() << '\n';
        return 1;
    }
}
