"""
Patch PNGdec to insert PNG_MAX_BUFFERED_PIXELS 4802 above #include "zutil.h"
so that it is defined before the value is consumed, without editing the
library source by hand (which is regenerated on pio clean).

Applied automatically by pre-build-script.py before each build.
"""

import os

TARGET = os.path.join(
    ".", ".pio", "libdeps", "mercator-origins-oceanic",
    "PNGdec", "src", "PNGdec.h"
)

ANCHOR  = '#include "zutil.h"'
INSERT  = '\n// Increase PNG buffer to support 600px wide RGB images\n // Formula: ((width * bytes_per_pixel + 1) * 2) for 2 scanlines with filter byte\n// For 600px RGB8: (600*4+1)*2 = 4802\n#define PNG_MAX_BUFFERED_PIXELS 4802\n'
MARKER  = '// [patched] PNG_MAX_BUFFERED_PIXELS'


def apply(env):
    if not os.path.isfile(TARGET):
        print(f"[patch] WARNING: {TARGET} not found, skipping.")
        return

    with open(TARGET, "r") as f:
        content = f.read()

    if MARKER in content:
        print("[patch] PNGdec PNG_MAX_BUFFERED_PIXELS already patched.")
        return

    if ANCHOR not in content:
        print(f"[patch] WARNING: '{ANCHOR}' not found in PNGdec.h, skipping.")
        return

    patched = content.replace(ANCHOR, INSERT + MARKER + "\n" + ANCHOR, 1)

    with open(TARGET, "w") as f:
        f.write(patched)
    print("[patch] PNGdec patched: inserted PNG_MAX_BUFFERED_PIXELS 4802 before zutil.h include.")
