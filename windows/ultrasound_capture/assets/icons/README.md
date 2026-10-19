# Embedded UI Icons

These are standard Lucide 0.468.0 icons, not custom drawings. Their order matches
`UiIcon` in `icons_win.hpp`; the icon-name mapping is in `generate_icons.cjs`.
The source arrays were exported from the project's existing Lucide bundle:

`/mnt/data/projects/paus_robot/designs/us-roi-calibration/assets/lucide.min.js`

Upstream: https://github.com/lucide-icons/lucide/tree/0.468.0

License copied from: https://raw.githubusercontent.com/lucide-icons/lucide/0.468.0/LICENSE

Each SVG retains Lucide's 24-unit viewBox and 2-unit round stroke. It is rasterized
to a transparent 48x48 PNG with a black stroke. `icon_masks.hpp` contains only the
PNG alpha samples; native code applies its own color when creating the bitmap.
`contact-sheet.png` is a visual check and is not loaded by the application.

To regenerate from the checked-in SVGs, install/use Node and Playwright with an
available Chromium, then run this script. These tools are build-time only:

```sh
PLAYWRIGHT_MODULE=/path/to/node_modules/playwright node generate_icons.cjs
```

To re-export SVGs from the same Lucide version, also supply
`LUCIDE_MODULE=/path/to/lucide.min.js`. Existing SVG files are sufficient for an
offline regeneration. `CHROME_EXECUTABLE=/path/to/chrome` selects an existing
system browser instead of Playwright's downloaded Chromium. The final Windows executable has no Node, browser, network,
SVG renderer, or external image-file dependency for its icons.

Include this license when distributing the icons or application.
