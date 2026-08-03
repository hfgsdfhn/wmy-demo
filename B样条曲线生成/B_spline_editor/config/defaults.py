"""Default field data for the Upre 2026 internal Robocon competition."""
from pathlib import Path


# Calibrated from the supplied drawing: its 100 px blue start areas are 700 mm,
# and its 202 px storage area is 1400 mm x 400 mm.
FIELD_WIDTH_MM = 11200.0
FIELD_HEIGHT_MM = 6100.0

# The supplied field image is kept beside the application directory.
FIELD_IMAGE_PATH = Path(__file__).resolve().parents[2] / "屏幕截图 2026-08-02 104037.png"

# Pixel rectangle containing only the field in the supplied screenshot.
FIELD_IMAGE_CROP = (30, 38, 1604, 874)

# The lower-left corner of the blue start block in the cropped image.
# Coordinates are image pixels, measured from the cropped image's left/top.
BLUE_START_ORIGIN_PIXEL = (8.0, 871.0)
