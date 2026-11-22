# What Vector's simulated camera sees on the light cube

The SDK OBJ's texture coordinates use the top of the source image as their origin. Three.js flips loaded images by default. With the old loader, Chromium placed pieces of glyphs on the corner caps and left the centers of two faces blank, while the raw OBJ UVs themselves swapped and horizontally mirrored several faces (`-Y` rendered as an identical copy of `+X`) and grazed an adjacent atlas cell on `-X`. The web viewer (`web/src/game/cube.js`) now both samples the atlas in the light-on-dark polarity (`flipY = false`) and repairs marker UVs onto centered 23.0 mm panels ordered per `Block::AddFace` (`-X` front, `+X` back, `-Y` right, `+Y` left, `+Z` top, `-Z` bottom). The two gray values chosen for shell and caps approximate the photograph. Absolute reflectance and exposure remain unmeasured.

![Same six orthographic cameras with the original and corrected web materials](faces.png)

With a 44 mm cube, the rendered outer border measures 23.0–23.1 mm horizontally and vertically across all six faces at a threshold of 140/255, with center offsets at or below 0.05 mm and no stray texture lines at any threshold. These reproduce the web asset as actually drawn. The engine's nominal square is 23 mm. Render sampling is 0.1 mm per pixel under uniform lighting. The [threshold sweep](measurements.csv) records all six faces across thresholds 110, 140, and 170.

This corrects the optional SDK optical surface seen by both viewer cameras. The recorded-pose analysis projects a mathematical 23 mm segment directly, so the rendering fix cannot account for its raised-stripe residual.

To reproduce, install the pinned SDK assets with the web project's fetch tool, start Vite from the web directory, and install the righting experiment's Playwright dependency. From this evidence directory run:

```sh
CHROMIUM=/path/to/chromium node render.cjs http://127.0.0.1:5173/ /tmp/cube-projections
python measure.py /tmp/cube-projections
```
