// Generates build-time assets only. The Windows application does not use Node or a browser.
const fs = require("node:fs");
const path = require("node:path");
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || "playwright");

const icons = [
  ["Crop", "Crop"], ["Settings", "Settings2"], ["Back", "ArrowLeft"],
  ["Play", "Play"], ["Pause", "Pause"], ["Draw", "SquareDashedMousePointer"],
  ["Move", "Move"], ["Undo", "Undo2"], ["Restore", "RotateCcw"],
  ["FullWindow", "Scan"], ["Save", "Save"], ["FolderOpen", "FolderOpen"],
  ["Up", "ArrowUp"], ["Down", "ArrowDown"], ["Left", "ArrowLeft"],
  ["Right", "ArrowRight"], ["Crosshair", "Crosshair"], ["Ruler", "Ruler"],
  ["Trash", "Trash2"], ["Refresh", "RefreshCw"], ["Check", "Check"],
];
const size = 48;
const escapeAttribute = (value) => String(value).replaceAll("&", "&amp;")
  .replaceAll('"', "&quot;").replaceAll("<", "&lt;").replaceAll(">", "&gt;");
function serialize([tag, attributes, children = []]) {
  const attrs = Object.entries(attributes).map(([key, value]) => ` ${key}="${escapeAttribute(value)}"`).join("");
  return `<${tag}${attrs}>${children.map(serialize).join("")}</${tag}>`;
}

async function main() {
  if (process.env.LUCIDE_MODULE) {
    const lucide = require(process.env.LUCIDE_MODULE);
    for (const [name, source] of icons) {
      const node = lucide.icons[source];
      if (!node) throw new Error(`Missing Lucide icon: ${source}`);
      const svg = serialize([node[0], { ...node[1], width: size, height: size, stroke: "#000000" }, node[2]]);
      fs.writeFileSync(path.join(__dirname, `${name}.svg`), svg + "\n");
    }
  }
  const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME_EXECUTABLE, args: ["--no-sandbox"] });
  try {
    const page = await browser.newPage({ viewport: { width: 560, height: 240 }, deviceScaleFactor: 1 });
    const masks = [];
    const pngs = [];
    for (const [name] of icons) {
      const svg = fs.readFileSync(path.join(__dirname, `${name}.svg`), "utf8");
      const rendered = await page.evaluate(async ({ svg, size }) => {
        const url = URL.createObjectURL(new Blob([svg], { type: "image/svg+xml" }));
        try {
          const image = new Image();
          image.src = url;
          await image.decode();
          const canvas = document.createElement("canvas");
          canvas.width = canvas.height = size;
          const context = canvas.getContext("2d");
          context.drawImage(image, 0, 0, size, size);
          const rgba = context.getImageData(0, 0, size, size).data;
          return { png: canvas.toDataURL("image/png"), alpha: Array.from({ length: size * size }, (_, i) => rgba[i * 4 + 3]) };
        } finally {
          URL.revokeObjectURL(url);
        }
      }, { svg, size });
      if (rendered.alpha.length !== 2304 || !rendered.alpha.some((v) => v > 0) || !rendered.alpha.some((v) => v === 0))
        throw new Error(`Blank or nontransparent icon: ${name}`);
      fs.writeFileSync(path.join(__dirname, `${name}.png`), Buffer.from(rendered.png.split(",")[1], "base64"));
      masks.push(rendered.alpha);
      pngs.push(rendered.png);
    }
    let header = "#pragma once\n\n#include <array>\n#include <cstdint>\n\n";
    header += "// Generated from Lucide 0.468.0. See assets/icons/LICENSE and generate_icons.cjs.\n";
    header += "namespace uscapture {\ninline constexpr int icon_mask_size = 48;\n";
    header += "inline constexpr std::array<std::array<std::uint8_t, 2304>, 21> icon_masks = {{\n";
    for (let i = 0; i < masks.length; i++) {
      header += `    {{ // ${icons[i][0]} (${icons[i][1]})\n`;
      for (let row = 0; row < size; row++) header += "        " + masks[i].slice(row * size, (row + 1) * size).join(",") + ",\n";
      header += "    }},\n";
    }
    header += "}};\n} // namespace uscapture\n";
    fs.writeFileSync(path.resolve(__dirname, "../../icon_masks.hpp"), header);
    const sheet = await page.evaluate(async ({ icons, pngs }) => {
      const canvas = document.createElement("canvas");
      canvas.width = 560;
      canvas.height = 240;
      const context = canvas.getContext("2d");
      context.fillStyle = "#ffffff";
      context.fillRect(0, 0, canvas.width, canvas.height);
      context.fillStyle = "#202020";
      context.font = "10px sans-serif";
      context.textAlign = "center";
      for (let i = 0; i < pngs.length; i++) {
        const image = new Image(); image.src = pngs[i]; await image.decode();
        const x = (i % 7) * 80, y = Math.floor(i / 7) * 80;
        context.drawImage(image, x + 16, y + 4);
        context.fillText(icons[i][0], x + 40, y + 68);
      }
      return canvas.toDataURL("image/png");
    }, { icons, pngs });
    fs.writeFileSync(path.join(__dirname, "contact-sheet.png"), Buffer.from(sheet.split(",")[1], "base64"));
    console.log(`Generated ${masks.length} Lucide icons and 48x48 alpha masks`);
  } finally {
    await browser.close();
  }
}

main().catch((error) => { console.error(error); process.exitCode = 1; });
