Import ("env")

env.Replace(PROGNAME="mercator-origins-oceanic-io")

import os

lvgl_demos_dir = os.path.join(
    env.subst("$PROJECT_LIBDEPS_DIR"),
    env.subst("$PIOENV"),
    "lvgl",
    "demos",
)

if os.path.isdir(lvgl_demos_dir):
    lvgl_dir = os.path.dirname(lvgl_demos_dir)
    lilygo_src_dir = os.path.join(
        env.subst("$PROJECT_LIBDEPS_DIR"),
        env.subst("$PIOENV"),
        "LilyGo-AMOLED-Series",
        "src",
    )
    demo_include_paths = [lvgl_dir, os.path.dirname(lvgl_dir)]
    if os.path.isdir(lilygo_src_dir):
        demo_include_paths.append(lilygo_src_dir)

    env.AppendUnique(CPPPATH=demo_include_paths)
    env.BuildSources(
        os.path.join(env.subst("$BUILD_DIR"), "lvgl_demos"),
        lvgl_demos_dir,
    )

# Apply library patches
import sys
sys.path.insert(0, os.path.join(os.getcwd(), "patches"))
import SimpleFTPServer_littlefs
SimpleFTPServer_littlefs.apply(env)

import PNGdec_buffered_pixels
PNGdec_buffered_pixels.apply(env)
