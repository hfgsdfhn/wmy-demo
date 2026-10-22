"""Default field data for the Upre 2026 internal Robocon competition."""
from pathlib import Path


# 规则书规定棕色围栏内部的比赛场地为 11000 mm x 11000 mm。
FIELD_WIDTH_MM = 11000.0
FIELD_HEIGHT_MM = 11000.0

# The supplied field image is kept beside the application directory.
FIELD_IMAGE_PATH = Path(__file__).resolve().parents[1] / "field_default.png"

# 默认地图已经裁剪为围栏内部的完整比赛场地。
FIELD_IMAGE_CROP = None

# 地图左下角同时是红色启动区左下角，整张图片按 11000 mm 场地标定。
RED_START_ORIGIN_PIXEL = (0.0, 0.0)
