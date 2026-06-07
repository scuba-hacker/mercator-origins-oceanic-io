"""
Patch LilyGo's LVGL config so LVGL vector graphics has its required
dependencies enabled.

Applied automatically by pre-build-script.py before each build.
"""

import os
import re

TARGET = os.path.join(
    ".", ".pio", "libdeps", "mercator-origins-oceanic",
    "LilyGo-AMOLED-Series", "src", "lv_conf.h"
)

REPLACEMENTS = {
    r"^#define\s+LV_USE_FLOAT\s+0\b": "#define LV_USE_FLOAT            1",
    r"^#define\s+LV_USE_MATRIX\s+0\b": "#define LV_USE_MATRIX           1",
}


def apply(env):
    if not os.path.isfile(TARGET):
        print(f"[patch] WARNING: {TARGET} not found, skipping.")
        return

    with open(TARGET, "r", encoding="utf-8-sig") as f:
        content = f.read()

    if not re.search(r"^#define\s+LV_USE_VECTOR_GRAPHIC\s+1\b", content, flags=re.MULTILINE):
        print("[patch] LVGL vector graphics is disabled, skipping matrix/float patch.")
        return

    patched = content
    changed = []
    for pattern, replacement in REPLACEMENTS.items():
        patched_next, count = re.subn(pattern, replacement, patched, flags=re.MULTILINE)
        if count:
            patched = patched_next
            changed.append(replacement.split()[1])

    if not changed:
        print("[patch] LVGL vector graphics dependencies already enabled.")
        return

    with open(TARGET, "w", encoding="utf-8") as f:
        f.write(patched)

    print(f"[patch] LVGL vector graphics enabled dependencies: {', '.join(changed)}.")
