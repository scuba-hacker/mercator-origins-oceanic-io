Import ("env")

env.Replace(PROGNAME="mercator-origins-oceanic-io")

# Apply library patches
import sys, os
sys.path.insert(0, os.path.join(os.getcwd(), "patches"))
import SimpleFTPServer_littlefs
SimpleFTPServer_littlefs.apply(env)

import PNGdec_buffered_pixels
PNGdec_buffered_pixels.apply(env)
