"""场地地图：目标方位表，供"预置地图导航"使用。

场地按 120cm × 60cm 布置，机器人固定在场地正中心，云台原地旋转。
角度约定：以 1/2/3 号目标所在方向为 0°，俯视顺时针为正（单位：度）。

TARGETS 每个目标：
    angle        场上方位角（度，0～360），与 config.TAG_ANGLE 一致
    half_width   搜索范围半宽（度）：任务找目标时只在 angle ± half_width
                 内往复搜索，不转出这个范围；不写则用 DEFAULT_HALF_WIDTH
    distance_cm  距云台中心的水平距离（cm），按场地示意图估算，仅作参考
                 （调俯仰角、判断远近时可用，不参与旋转计算）

用法：任务开始时查 TARGETS[tag]["angle"]，算出还差多少度转过去，
停稳后静止取帧识别，避免边转边认导致画面拖影；没看到就在
TAG_WINDOW 给出的角度范围内往复找。
"""

FIELD_WIDTH_CM = 120     # 场地长（cm）
FIELD_HEIGHT_CM = 60     # 场地宽（cm）

HOME_ANGLE = 0           # 任务起点方向：1/2/3 号目标所在方位

DEFAULT_HALF_WIDTH = 20  # 目标没写 half_width 时，搜索范围默认 ±20°

TARGETS = {
    1:  {"angle": 0,   "half_width": 30, "distance_cm": 50},
    2:  {"angle": 0,   "half_width": 30, "distance_cm": 49},
    3:  {"angle": 0,   "half_width": 30, "distance_cm": 49},
    4:  {"angle": 30,  "half_width": 20, "distance_cm": 49},
    5:  {"angle": 55,  "half_width": 20, "distance_cm": 28},
    6:  {"angle": 140, "half_width": 20, "distance_cm": 32},
    7:  {"angle": 180, "half_width": 20, "distance_cm": 38},
    8:  {"angle": 180, "half_width": 20, "distance_cm": 40},
    9:  {"angle": 235, "half_width": 20, "distance_cm": 32},
    10: {"angle": 330, "half_width": 25, "distance_cm": 45},
}

# 编号 → 方位角，方便直接查表
TAG_ANGLE = {tag_id: info["angle"] for tag_id, info in TARGETS.items()}

# 编号 → 搜索角度范围 (下限, 上限)，上下限允许出现在 0～360 之外，
# 比较时用 angle_diff 处理跨 0° 的情况
TAG_WINDOW = {
    tag_id: (info["angle"] - info.get("half_width", DEFAULT_HALF_WIDTH),
             info["angle"] + info.get("half_width", DEFAULT_HALF_WIDTH))
    for tag_id, info in TARGETS.items()
}


def angle_diff(angle, reference):
    """算 angle 相对 reference 的有符号角差。

    输入：angle 角度；reference 参考角度（单位：度）
    输出：差值，范围 (-180, 180]，正数表示角度增大方向
    """

    diff = (angle - reference) % 360
    if diff > 180:
        diff -= 360
    return diff


def window_bounds(tag_id):
    """取某个目标的搜索角度范围 (下限, 上限)；地图里没有该目标返回 None。"""

    return TAG_WINDOW.get(tag_id)


def in_window(tag_id, angle):
    """角度是否落在该目标的搜索范围内（自动处理绕 0°）。"""

    window = TAG_WINDOW.get(tag_id)
    if window is None:
        return False
    center = TAG_ANGLE[tag_id]
    half = (window[1] - window[0]) / 2.0
    return abs(angle_diff(angle, center)) <= half


def shortest_delta(current_angle, target_angle):
    """算从当前朝向转到目标方位的最短转角。

    输入：current_angle 当前朝向（度）；target_angle 目标方位（度）
    输出：转角（度），正数表示角度增大方向转，负数表示角度减小方向转，
         范围 (-180, 180]。
    """

    delta = (target_angle - current_angle) % 360
    if delta > 180:
        delta -= 360
    return delta
