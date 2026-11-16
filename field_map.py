"""场地地图：目标方位表，供"预置地图导航"使用。

场地按 120cm × 60cm 布置，机器人固定在场地正中心，云台原地旋转。
角度约定：以 1/2/3 号目标所在方向为 0°，俯视顺时针为正（单位：度）。

TARGETS 每个目标：
    angle        场上方位角（度，0～360），与 config.TAG_ANGLE 一致
    distance_cm  距云台中心的水平距离（cm），按场地示意图估算，仅作参考
                 （调俯仰角、判断远近时可用，不参与旋转计算）

用法：任务开始时查 TARGETS[tag]["angle"]，算出还差多少度转过去，
停稳后静止取帧识别，避免边转边认导致画面拖影。
"""

FIELD_WIDTH_CM = 120     # 场地长（cm）
FIELD_HEIGHT_CM = 60     # 场地宽（cm）

HOME_ANGLE = 0           # 任务起点方向：1/2/3 号目标所在方位

TARGETS = {
     4: {"angle":   0, "distance_cm": 49},
     5: {"angle":  45, "distance_cm": 28},
     6: {"angle": 201, "distance_cm": 32},
     7: {"angle": 261, "distance_cm": 38},
     8: {"angle": 287, "distance_cm": 40},
     9: {"angle": 349, "distance_cm": 32},
    10: {"angle": 224, "distance_cm": 45},
}

# 编号 → 方位角，方便直接查表
TAG_ANGLE = {tag_id: info["angle"] for tag_id, info in TARGETS.items()}

# 参与"朝向校正"的标签：1/2/3 号贴在同一个方向，方位完全重合，单靠它们
# 既区分不出彼此、也定不准朝向，所以只当导航目标，不拿来推算当前位置。
CALIBRATION_TAGS = frozenset(TAG_ANGLE) - {1, 2, 3}


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
