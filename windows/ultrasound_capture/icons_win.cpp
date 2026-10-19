#include "icons_win.hpp"
#include "icon_masks.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace uscapture {

HBITMAP create_ui_icon(UiIcon icon, int size, COLORREF ink) {
    const auto index = static_cast<std::size_t>(icon);
    if (size < 1 || size > 256 || index >= icon_masks.size()) return nullptr;
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info.bmiHeader.biWidth = size;
    info.bmiHeader.biHeight = -size;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    info.bmiHeader.biCompression = BI_RGB;
    void* memory = nullptr;
    HBITMAP bitmap = CreateDIBSection(nullptr, &info, DIB_RGB_COLORS, &memory, nullptr, 0);
    if (!bitmap) return nullptr;
    auto* pixels = static_cast<std::uint8_t*>(memory);
    const auto& alpha = icon_masks[index];
    for (int y = 0; y < size; ++y) {
        const double sy = std::clamp((y + .5) * icon_mask_size / size - .5, 0.0, double(icon_mask_size - 1));
        const int y0 = static_cast<int>(sy), y1 = std::min(y0 + 1, icon_mask_size - 1);
        for (int x = 0; x < size; ++x) {
            const double sx = std::clamp((x + .5) * icon_mask_size / size - .5, 0.0, double(icon_mask_size - 1));
            const int x0 = static_cast<int>(sx), x1 = std::min(x0 + 1, icon_mask_size - 1);
            const double ax = sx - x0, ay = sy - y0;
            const double top = alpha[y0 * icon_mask_size + x0] * (1 - ax) + alpha[y0 * icon_mask_size + x1] * ax;
            const double bottom = alpha[y1 * icon_mask_size + x0] * (1 - ax) + alpha[y1 * icon_mask_size + x1] * ax;
            const auto a = static_cast<unsigned>(std::lround(top * (1 - ay) + bottom * ay));
            const auto offset = (static_cast<std::size_t>(y) * size + x) * 4;
            pixels[offset] = static_cast<std::uint8_t>((GetBValue(ink) * a + 127) / 255);
            pixels[offset + 1] = static_cast<std::uint8_t>((GetGValue(ink) * a + 127) / 255);
            pixels[offset + 2] = static_cast<std::uint8_t>((GetRValue(ink) * a + 127) / 255);
            pixels[offset + 3] = static_cast<std::uint8_t>(a);
        }
    }
    return bitmap;
}

void paint_ui_icon(HDC target, HBITMAP bitmap, int x, int y, int size) {
    if (!target || !bitmap || size < 1) return;
    BITMAP info{};
    if (!GetObjectW(bitmap, sizeof(info), &info)) return;
    HDC source = CreateCompatibleDC(target);
    if (!source) return;
    const HGDIOBJ previous = SelectObject(source, bitmap);
    BLENDFUNCTION blend{AC_SRC_OVER, 0, 255, AC_SRC_ALPHA};
    AlphaBlend(target, x, y, size, size, source, 0, 0, info.bmWidth, info.bmHeight, blend);
    SelectObject(source, previous);
    DeleteDC(source);
}

} // namespace uscapture
