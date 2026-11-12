"""圆目标相关功能：找圆、选圆、对准圆心（供 track_fire.py / 标定工具使用）。

    找圆任务（任务里写 "circle": True）：
        不找标签，改为寻找画面中的圆，圆心到位后执行动作；多个圆时
        自动选择半径最大的那个。

    标签附近找圆（任务里写 "aim_circle": True，总开关 AIM_CIRCLE_NEAR_TAG）：
        标签锁定后，在标签附近找圆（杯口），把圆心对准到 offset 位置后
        才执行动作；标签附近暂时没有圆时，云台原地等待圆出现。

    俯仰角微调（PITCH_PER_PX）：
        摄像头固定、发射器在其头顶，标签贴在杯内底部，而要打进的是
        杯口（圆心），两者在画面里的垂直像素差 × 该系数 = 俯仰修正角。
        标定方法：先设 0 观察打印的垂直差，实测打高就把系数往负调、
        打低往正调。
"""

import math

from board import board
from vision import vision
from config import VOLTAGE_POLARITY

# 标签附近找圆：总开关默认关闭，需要该功能的任务在任务里写 "aim_circle": True
AIM_CIRCLE_NEAR_TAG = False   # 总开关；单个任务可用 "aim_circle": True 打开
CIRCLE_NEAR_FACTOR = 2.5      # "附近"范围 = 标签对角线长度 × 该倍数
CIRCLE_NEAR_MIN = 100         # "附近"范围的下限（像素），标签很小时保底

PITCH_PER_PX = 0.06   # 度/像素：圆心与标签中心的垂直差换算成俯仰修正角


def pick_circle_near(circles, target, last_aim):
    """从检测到的圆里挑出"离标签最近"的圆。

    以标签中心为参考：圆心与标签中心相距"标签对角线 × CIRCLE_NEAR_FACTOR"
    以内（不小于 CIRCLE_NEAR_MIN）才算附近，有多个时选圆心离标签最近的
    （杯口圆心就贴着杯底的标签，最近的才是本杯开口；最大半径可能选中
    背景里更大的圆）。标签暂时丢失时，改为围绕上一次对准的圆心找，
    避免标签闪断导致目标来回跳。
    输入：circles 检测到的圆列表；target 目标标签（可能为 None）；
         last_aim 上一次对准的圆 (x, y, r)（可能为 None）
    输出：选中的圆 (x, y, r)；附近没有圆时返回 None。
    """

    if target is not None:
        diag = math.dist(target.corners[0], target.corners[2])  # 标签对角线长度
        near_dist = max(CIRCLE_NEAR_FACTOR * diag, CIRCLE_NEAR_MIN)
        ref_x, ref_y = target.center[0], target.center[1]
    elif last_aim is not None:
        near_dist = CIRCLE_NEAR_MIN
        ref_x, ref_y = last_aim[0], last_aim[1]
    else:
        return None

    near = [c for c in circles
            if math.hypot(c[0] - ref_x, c[1] - ref_y) <= near_dist]
    return (min(near, key=lambda c: math.hypot(c[0] - ref_x, c[1] - ref_y))
            if near else None)


def largest_circle(circles):
    """挑出半径最大的圆。

    多个圆时选半径最大的：同心圆即最外圈（圆心相同），多个目标时
    最大的一般也是最近、最明显的那个。
    输入：circles 检测到的圆列表
    输出：半径最大的圆 (x, y, r)；没有圆时返回 None。
    """

    return max(circles, key=lambda c: c[2]) if circles else None


def aim_target(tags, circles, tag_id, use_circle):
    """自动扫描俯仰角时，从当前帧挑出用来判断垂直是否居中的目标。

    输入：tags 标签列表；circles 圆列表（不找圆时为 None）；
         tag_id 目标标签编号（None 表示找圆任务）；use_circle 是否瞄准圆心
    输出：目标中心 (x, y) 或圆 (x, y, r)；没找到时返回 None。
    """

    target = (next((t for t in tags if t.tag_id == tag_id), None)
              if tag_id is not None else None)
    if not use_circle:
        return target.center if target is not None else None
    if tag_id is None:   # 找圆任务：任意圆都算目标
        return largest_circle(circles)
    aim = pick_circle_near(circles, target, None)   # 标签任务：优先用标签附近的圆
    if aim is not None:
        return aim
    return target.center if target is not None else None


def circle_pitch(base_pitch, aim, target, last_dy, pitch_min, pitch_max):
    """按圆心相对标签中心的垂直像素差，微调执行动作时的俯仰角。

    标签贴在杯内底部、要打进的是杯口（圆心），两者在画面里的高度不一致：
    "圆心 − 标签中心"的垂直差 × PITCH_PER_PX 就是俯仰修正量。标签闪断时
    沿用上一次的垂直差。
    输入：base_pitch 基准俯仰角；aim 圆心 (x, y, r)；target 标签（可能为 None）；
         last_dy 上一次的垂直差；pitch_min/pitch_max 俯仰角安全范围
    输出：(pitch, dy)：修正后的俯仰角、本次使用的垂直差（正数表示圆心偏下）。
    """

    if target is not None:
        last_dy = int(aim[1] - target.center[1])
    pitch = int(base_pitch + last_dy * PITCH_PER_PX)
    return max(pitch_min, min(pitch_max, pitch)), last_dy


def track_circle(camera, offset, speed_limit, motor_speed, center_half_width,
                 poll_key):
    """跟踪画面中的圆，直到圆心到达期望位置。

    输入：camera 摄像头对象；offset 期望的圆心水平位置（相对画面中心的
         像素偏移，画面中心为 0）；speed_limit 精细速度幅值；
         motor_speed 由偏差算速度的函数（主程序传入）；center_half_width
         居中阈值（像素）；poll_key 每帧读一次按键的函数，返回 None 表示
         继续、True 表示用户退出、整数 n 表示请求跳到任务 n
    输出：True 已到位；False 用户按 Q/Esc 退出；
         整数 n 表示按了数字键，请求跳到任务 n。
    """

    last_speed = 0   # 上一次发送的速度（0 表示停止）
    while True:
        frame = vision.read_frame(camera)
        if frame is None:
            continue

        center_x = frame.shape[1] // 2                          # 画面中心 X
        center_y = frame.shape[0] // 2                          # 画面中心 Y
        circles = vision.detect_circles(frame)                  # 检测画面中的圆

        if not circles:
            speed = speed_limit * VOLTAGE_POLARITY   # 没发现圆，云台转动搜索
            status = "未发现圆，搜索中"
        else:
            x, y, r = largest_circle(circles)
            error = int(x - center_x - offset)
            speed = motor_speed(error, speed_limit)
            status = (f"圆心：偏差 {error}（X:{x - center_x:.0f} "
                      f"Y:{y - center_y:.0f}，半径 {r:.0f}）")

            if abs(error) <= center_half_width:   # 圆心已到达瞄准位置
                print(f"{status} |=> 锁定圆心")
                board.set_motor_b(0)  # 云台停止
                return True

        # 只在速度变化时发送命令
        if speed != last_speed:
            board.set_motor_b(speed)
            last_speed = speed
            status = f"{status} |=> 发送指令"
            print(status)

        vision.show_frame(frame, [], status, circles)
        action = poll_key()
        if action is True:
            print("按 Q 或 Esc 退出。")
            return False
        if isinstance(action, int):
            board.set_motor_b(0)
            return action
