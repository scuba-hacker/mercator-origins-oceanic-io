"""
Patch SimpleFTPServer to use LittleFS instead of FFat on ESP32.
Applied automatically by pre-build-script.py before each build.
"""

import os
import re

TARGET = os.path.join(
    ".", ".pio", "libdeps", "mercator-origins-oceanic",
    "SimpleFTPServer", "FtpServerKey.h"
)

PATTERN = r"(#define\s+DEFAULT_STORAGE_TYPE_ESP32\s+)STORAGE_FFAT"
REPLACEMENT = r"\1STORAGE_LITTLEFS"
ALREADY_PATCHED = "DEFAULT_STORAGE_TYPE_ESP32"
ALREADY_PATCHED_VALUE = "STORAGE_LITTLEFS"


def apply(env):
    if not os.path.isfile(TARGET):
        print(f"[patch] WARNING: {TARGET} not found, skipping.")
        return

    with open(TARGET, "r") as f:
        content = f.read()

    if re.search(r"#define\s+DEFAULT_STORAGE_TYPE_ESP32\s+STORAGE_LITTLEFS", content):
        print("[patch] SimpleFTPServer already patched (STORAGE_LITTLEFS).")
        return

    patched, count = re.subn(PATTERN, REPLACEMENT, content)
    if count == 0:
        print("[patch] WARNING: DEFAULT_STORAGE_TYPE_ESP32 STORAGE_FFAT not found, skipping.")
        return

    with open(TARGET, "w") as f:
        f.write(patched)
    print("[patch] SimpleFTPServer patched: DEFAULT_STORAGE_TYPE_ESP32 -> STORAGE_LITTLEFS")
