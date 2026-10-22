#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

namespace uscapture {

enum class UiIcon {
    Crop, Settings, Back, Play, Pause, Draw, Move, Undo, Restore, FullWindow,
    Save, FolderOpen, Up, Down, Left, Right, Crosshair, Ruler, Trash, Refresh, Check
};

// The caller owns the returned premultiplied-alpha bitmap.
HBITMAP create_ui_icon(UiIcon icon, int size, COLORREF ink);
void paint_ui_icon(HDC target, HBITMAP bitmap, int x, int y, int size);

} // namespace uscapture
