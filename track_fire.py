"""按任务顺序执行：调整俯仰 → 地图导航 / 跟踪标签或圆 → 执行动作。

TASKS 每个任务的字段（除 tag、action 外均可省略）：
    tag       ：目标 AprilTag 编号
    offset    ：期望水平位置，相对画面中心的像素偏移（右正左负，0=中心）
    angle     ：舵机俯仰角 0～180（AUTO_PITCH 开启时作为扫描起点）
    action    ：到位后动作，"laser" 激光 3 秒 / "fire" 发射一次
    hold      ：动作后停留秒数（云台不动）
    search    ：False 表示未发现目标时原地等待（缺省为转动搜索）
    aim_circle：True 表示锁定标签后继续对准附近的圆心
    circle    ：True 表示不找标签，改为直接找圆

按键：数字键 1~9（0=任务 10）跳到对应任务，之后仍按原顺序继续；
      A 键解锁后重做当前任务；Q / Esc 退出。
"""

import math
import time

from board import board
from vision import vision
from config import (SERIAL_PORT, VOLTAGE_POLARITY, GIMBAL_SPEED_REF,
                    GIMBAL_DEG_PER_SEC, GIMBAL_CLOCKWISE_SIGN)
from field_map import TAG_ANGLE, CALIBRATION_TAGS, shortest_delta
from circle_aim import AIM_CIRCLE_NEAR_TAG, PITCH_PER_PX, pick_circle_near

# ============================== 云台与对准 ==============================
MOTOR_SPEED = 180        # 任务 1/2/3 的云台转速度
LATER_MOTOR_SPEED = 130  # 任务 4 起的云台转速度
CENTER_HALF_WIDTH = 13   # 离瞄准点多近算"居中"（像素）
CENTER_DIRECTION = -1    # 回中方向：+1 默认，-1 反向（不影响搜索方向）
DETECT_SETTLE = 0.15     # 识别到目标后等云台停稳的秒数
AIM_OFFSET_PX = 0        # 任务未写 offset 时的默认瞄准位置（0=画面中心）

# ============================== 标签附近找圆 ==============================
# AIM_CIRCLE_NEAR_TAG、PITCH_PER_PX、pick_circle_near 统一在 circle_aim.py 定义

# ============================== 自动识别俯仰角 ==============================
AUTO_PITCH = False        # 总开关；False 时沿用任务里写死的 angle
PITCH_START = 90          # 扫描起点（任务写了 angle 则用它当起点）
PITCH_MIN, PITCH_MAX = 60, 140
PITCH_SCAN_STEP = 3       # 扫描步长（度）
PITCH_SCAN_DWELL = 0.5    # 每个角度停留观察的秒数
PITCH_TOLERANCE = 20      # 垂直偏差多少像素以内算已对准

# ============================== 地图导航 ==============================
HEADING_SPEED = 255       # 导航速度：用标定过的满速，角度换算最准
HEADING_MIN_DELTA = 3     # 转角小于该值就不转，避免来回微抖
COARSE_SWITCH_PX = 120    # 偏差超过该值先用导航速度靠过去，再降到精细速度
MAP_CONFIRM_FRAMES = 3    # 到目标方位后静止连拍确认的帧数
MAP_NAV_ATTEMPTS = 2      # 导航最多修正几次（用实时校正后的朝向再转一次）
CAMERA_HFOV_DEG = 62.0      # 摄像头水平视场角（度）：把像素偏移换算成角度

_heading = None           # 当前朝向（0～360）；None 表示还没标定过
_drive_speed = 0          # 最近一次水平电机指令（含符号）
_drive_time = 0.0         # 该指令生效的时刻（用于朝向开环记账）
_current_task_no = 1      # 当前任务序号（1 起）：A 键解锁后据此重做

# 任务列表：按顺序逐个执行
TASKS = [
    {"tag": 1, "offset": -45, "angle": 110, "action": "fire", "search": False},
    {"tag": 2, "offset": -45, "angle": 110, "action": "fire", "search": False},
    {"tag": 3, "offset": -45, "angle": 110, "action": "fire", "search": False},
    {"tag": 4, "offset": -70, "angle": 101, "action": "laser"},
    {"tag": 5, "offset": 10, "angle": 107, "action": "fire"},
    {"tag": 6, "offset": -47, "angle": 110, "action": "laser"},
    {"tag": 7, "offset": -200, "angle": 106, "action": "fire"},
    {"tag": 8, "offset": 120, "angle": 92, "action": "fire"},
    {"tag": 9, "offset": -49, "angle": 111, "action": "laser"},
    {"tag": 10, "offset": 9, "angle": 105, "action": "fire"},
    # 追加"找圆 → 对准圆心 → 发射"的示例任务：
    # {"circle": True, "offset": -20, "angle": 102, "action": "fire"},
]


# ============================== 基础工具 ==============================
def motor_speed(error, speed_limit=MOTOR_SPEED):
    """偏差 → 云台速度；居中的返回 0。"""

    if error > CENTER_HALF_WIDTH:
        return -speed_limit * VOLTAGE_POLARITY * CENTER_DIRECTION
    if error < -CENTER_HALF_WIDTH:
        return speed_limit * VOLTAGE_POLARITY * CENTER_DIRECTION
    return 0


def key_to_task_index(key):
    """数字键 → 任务序号（1～9、0 表示 10）；其余返回 None。"""

    if ord("1") <= key <= ord("9"):
        return key - ord("0")
    if key == ord("0"):
        return 10
    return None


def task_key(number):
    """任务序号 → 对应的数字键编码（10 用 0）。"""

    return ord("0") if number >= 10 else ord(str(number))


def set_current_task(number):
    """记录当前任务序号，供 A 键解锁时重做。"""

    global _current_task_no
    _current_task_no = number


def key_result(key):
    """跟踪循环里的按键处理：False=退出，整数=跳到该任务，None=继续。"""

    if vision.is_quit_key(key):
        print("按 Q 或 Esc 退出。")
        return False
    jump = key_to_task_index(key)
    if jump is not None and jump <= len(TASKS):
        drive(0)
        return jump
    return None


# ============================== 朝向记账 ==============================
def _integrate_heading(now=None):
    """把"上次指令到此刻"的转动量累加进朝向（开环记账）。

    搜索时云台会连续转很久，不实时记账的话朝向会停在最后一次看到标签
    的角度，下次导航算出的转角就不对。
    """

    global _heading, _drive_time
    now = time.monotonic() if now is None else now
    if _heading is not None and _drive_speed:
        rate = _drive_speed / GIMBAL_SPEED_REF * GIMBAL_DEG_PER_SEC
        _heading = (_heading + rate * (now - _drive_time)) % 360
    _drive_time = now


def drive(speed):
    """设置水平电机速度，同时把转动量实时记进朝向。

    所有驱动云台的地方都走这里（代替直接调 board.set_motor_b）。
    """

    global _drive_speed
    _integrate_heading()
    _drive_speed = int(speed)
    if _drive_speed:
        board.set_motor_b(_drive_speed)
    else:
        board.stop_motor_b()


def learn_heading(tag_id):
    """看到已知标签时，用地图把当前朝向校正为准确值。

    只用 CALIBRATION_TAGS 里的标签：1/2/3 号方位重合，不参与定位。
    """

    global _heading
    if tag_id in CALIBRATION_TAGS:
        _integrate_heading()
        _heading = TAG_ANGLE[tag_id] % 360


def px_to_deg(dx, width):
    """画面水平像素偏移 → 角度偏移（小孔成像），右正左负。"""

    focal = (width / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG) / 2)
    return math.degrees(math.atan(dx / focal))


def learn_from_tags(frame, tags):
    """用画面里任意已知标签实时推算朝向（不要求标签居中）。

    目标标签还没扫到时，其它二维码就是当前位置的参照：按小孔成像
    把标签的水平像素偏移换算成角度，朝向 = 标签方位 − 角度偏移。
    多个同时可见时取最靠近画面中心的一个（误差最小）。
    1/2/3 号方位重合、定不准位置，不参与校正（见 field_map.CALIBRATION_TAGS）。
    输入：frame 当前帧；tags 识别到的标签
    输出：True 校正成功；False 画面里没有可用于定位的标签
    """

    global _heading

    if frame is None or not tags:
        return False
    center_x = frame.shape[1] / 2.0
    known = [t for t in tags if t.tag_id in CALIBRATION_TAGS]
    if not known:
        return False
    tag = min(known, key=lambda t: abs(t.center[0] - center_x))
    _integrate_heading()             # 先把转动记账结算到此刻，再钉准
    _heading = (TAG_ANGLE[tag.tag_id]
                - px_to_deg(tag.center[0] - center_x, frame.shape[1])) % 360
    return True


def read_tags(camera, detector):
    """读一帧并识别标签；每帧都用画面里的已知标签校正朝向。

    目标标签还没扫到也没关系：其它二维码同样能定位当前朝向，转动中
    也持续更新，避免只靠开环记账累积误差。
    """

    _integrate_heading()
    frame, tags = vision.read_tags(camera, detector)
    learn_from_tags(frame, tags)
    return frame, tags


def wait_key():
    """读按键；A 键刚解锁时返回当前任务的数字键（等于重做当前任务）。"""

    if board.take_unlock_event():
        # 锁定期间云台可能被人手拨动：朝向作废、舵机复位，由当前任务重新建立
        global _heading
        _heading = None
        drive(0)
        ok = board.reset_servo()
        print(f"\n>>> A 键解锁：舵机复位 {'成功' if ok else '未确认'}，"
              f"重做任务 {_current_task_no}")
        return task_key(_current_task_no)
    return vision.wait_key()


def status_with_lock(status):
    """画面状态栏附加 A 键锁停提示。"""

    return f"{status}  【A键锁停】" if board.motor_b_locked() else status


def pump_frames(camera, detector, seconds, status):
    """等若干秒，期间持续刷新画面并响应按键；返回按下的键（无键为 255）。"""

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        frame, tags = read_tags(camera, detector)
        if frame is not None:
            vision.show_frame(frame, tags, status_with_lock(status))
        key = wait_key()
        if key != 255:      # waitKey 无按键返回 -1，取低 8 位后是 255
            return key
    return 255


def execute_action(action, camera, detector):
    """执行动作，期间画面实时刷新：True 继续 / False 退出 / 整数=跳转。"""

    if action == "laser":
        board.laser_on()
        try:     # finally 保证中途退出也一定关激光
            key = pump_frames(camera, detector, 3, "激光中（3 秒）……")
        finally:
            board.laser_off()
        print("    动作：激光发射 3 秒")
    elif action == "fire":
        board.fire_once()
        key = pump_frames(camera, detector, 3, "发射中，等待机构复位……")
        print("    动作：发射一次")
    else:
        raise ValueError(f"未知动作: {action}（只支持 laser / fire）")

    result = key_result(key)
    return True if result is None else result


# ============================== 地图导航 ==============================
def rotate_by(camera, detector, delta, speed=HEADING_SPEED):
    """按角度定时转动云台（开环），转动期间画面保持刷新。"""

    global _heading

    if abs(delta) < HEADING_MIN_DELTA:
        return True            # 差得太小，不值得动

    deg_per_sec = GIMBAL_DEG_PER_SEC * speed / GIMBAL_SPEED_REF
    seconds = abs(delta) / max(deg_per_sec, 1e-3)
    print(f"    地图导航：转 {delta:+.0f}°（{seconds:.1f} 秒）")
    drive(speed * GIMBAL_CLOCKWISE_SIGN * (1 if delta > 0 else -1))
    try:
        key = pump_frames(camera, detector, seconds,
                          f"地图导航：转 {delta:+.0f}°")
    finally:
        drive(0)
    if vision.is_quit_key(key):
        _heading = None        # 半路被打断，朝向不确定，等看到标签再定
        return False
    return True


def rotate_to(camera, detector, target_angle, speed=HEADING_SPEED):
    """转到指定场上方位角；朝向未知时返回 None。"""

    if _heading is None:
        return None
    return rotate_by(camera, detector,
                     shortest_delta(_heading, target_angle), speed)


def goto_map_position(camera, detector, tag_id):
    """任务开始：按预置地图转到目标方位，停稳后静止确认。

    全程实时定位：没扫到目标标签时，画面里任何已知标签都当作当前位置
    的参照（像素偏移 → 角度），所以转到一半、或只看到旁边标签也能把
    朝向拉回来。转完若发现朝向又被校正过，就用新朝向再修正一次。
    返回 True 继续执行 / False 用户退出 / None 不走导航（交回搜索流程）。
    """

    target = TAG_ANGLE.get(tag_id)
    for _ in range(MAP_NAV_ATTEMPTS):
        frame, tags = read_tags(camera, detector)
        if any(seen.tag_id == tag_id for seen in tags):
            return True                  # 目标本来就在画面里，不用转
        if _heading is None or target is None:
            return None                  # 朝向未知或地图里没有该目标

        print(f"    地图导航：当前朝向 {_heading:.0f}° → 目标 {target}°")
        if rotate_to(camera, detector, target) is False:
            return False

        drive(0)             # 到点先停稳
        time.sleep(DETECT_SETTLE)
        for _ in range(MAP_CONFIRM_FRAMES):
            frame, tags = read_tags(camera, detector)
            if frame is not None:
                vision.show_frame(frame, tags, f"地图导航：确认标签 {tag_id}")
            if any(seen.tag_id == tag_id for seen in tags):
                learn_heading(tag_id)    # 看到目标，顺便把朝向校得更准
                print(f"    地图导航：已到位，看到标签 {tag_id}")
                return True
            if vision.is_quit_key(wait_key()):
                return False
    print(f"    地图导航：已到位但没看到标签 {tag_id}，改用搜索")
    return True


# ============================== 自动识别俯仰角 ==============================
def scan_pitch(camera, detector, tag_id, use_circle, start_angle,
               speed_limit=MOTOR_SPEED):
    """自动扫描俯仰角，找目标垂直居中的角度；返回 (angle, ok)。"""

    def clamp(angle):
        return max(PITCH_MIN, min(PITCH_MAX, angle))

    start = clamp(start_angle)
    angles = [start]   # 先在起点试，再向两侧交替扩展
    span = (PITCH_MAX - PITCH_MIN) // PITCH_SCAN_STEP
    for k in range(1, span + 1):
        for candidate in (start + k * PITCH_SCAN_STEP,
                          start - k * PITCH_SCAN_STEP):
            if PITCH_MIN <= candidate <= PITCH_MAX:
                angles.append(candidate)

    best_angle, best_dy = start, None
    print(f"    自动识别俯仰角度（扫描 {PITCH_MIN}～{PITCH_MAX}°，"
          f"步长 {PITCH_SCAN_STEP}°）……")
    drive(speed_limit * VOLTAGE_POLARITY)   # 扫描同时让云台水平搜索
    try:
        for angle in angles:
            board.set_servo_angle(angle)
            deadline = time.monotonic() + PITCH_SCAN_DWELL
            while time.monotonic() < deadline:
                # 扫描时云台在水平转动，每帧都用画面里的标签实时校正朝向
                frame, tags = read_tags(camera, detector)
                if frame is None:
                    continue

                center_y = frame.shape[0] // 2
                circles = vision.detect_circles(frame) if use_circle else None
                if tag_id is None:      # 找圆任务：任意圆都算目标
                    ref = max(circles, key=lambda c: c[2]) if circles else None
                else:                    # 标签任务：优先用标签附近的圆
                    target = next((t for t in tags if t.tag_id == tag_id), None)
                    aim = (pick_circle_near(circles, target, None)
                           if use_circle else None)
                    ref = (aim if aim is not None else
                           (target.center if target is not None else None))

                status = f"自动识别俯仰角：{angle}°"
                if ref is not None:
                    dy = int(ref[1] - center_y)   # 正数表示目标偏下
                    if best_dy is None or abs(dy) < abs(best_dy):
                        best_angle, best_dy = angle, dy
                    status = f"{status}，垂直偏差 {dy}"
                    if abs(dy) <= PITCH_TOLERANCE:
                        print(f"    自动识别俯仰角度：{angle}°"
                              f"（垂直偏差 {dy} 像素）")
                        return angle, True

                vision.show_frame(frame, tags, status_with_lock(status), circles)
                if vision.is_quit_key(wait_key()):
                    print("按 Q 或 Esc 退出。")
                    return best_angle, False
    finally:
        drive(0)

    if best_dy is None:
        print(f"    未发现目标，俯仰角度沿用 {best_angle}°")
    else:
        print(f"    自动识别俯仰角度：{best_angle}°"
              f"（垂直偏差 {best_dy} 像素）")
    return best_angle, True


# ============================== 跟踪 ==============================
def track_tag(camera, detector, tag_id, offset, aim_circle=False,
              base_pitch=PITCH_START, search=True, speed_limit=MOTOR_SPEED,
              coarse_speed=None):
    """跟踪标签直到瞄准位置（可再对准附近圆心）。

    coarse_speed 非空时：找目标先用粗调速度，偏差大于 COARSE_SWITCH_PX
    也用粗调速度靠过去，接近后降到 speed_limit。
    返回 True 到位 / False 退出 / 整数 n 跳到任务 n。
    """

    def align_speed(error):
        if coarse_speed and abs(error) > COARSE_SWITCH_PX:
            return motor_speed(error, coarse_speed)
        return motor_speed(error, speed_limit)

    search_speed = coarse_speed or speed_limit
    last_speed = 0
    last_aim = None     # 上次对准的圆 (x, y, r)：标签闪断时按它找回
    last_dy = 0         # 圆心相对标签中心的垂直像素差
    searching = True    # 是否处于左右转动搜索的状态

    while True:
        frame, tags = read_tags(camera, detector)
        if frame is None:
            continue

        center_x = frame.shape[1] // 2
        target = next((t for t in tags if t.tag_id == tag_id), None)
        circles = vision.detect_circles(frame) if aim_circle else None
        aim = pick_circle_near(circles, target, last_aim) if aim_circle else None

        if target is None and aim is None:
            searching = True
            if search:
                speed = search_speed * VOLTAGE_POLARITY   # 转动搜索
                status = f"标签 {tag_id}：未发现，搜索中"
            else:
                speed = 0                                 # 原地等待
                status = f"标签 {tag_id}：未发现，原地等待"
        else:
            if searching:      # 刚识别到目标：先停转，等停稳再测偏差
                searching = False
                if search:
                    drive(0)
                    last_speed = 0
                    print(f"    识别到标签 {tag_id}，停止左右转动，开始对准。")
                    time.sleep(DETECT_SETTLE)

            if aim is not None:
                # 标签附近有圆：以圆心为瞄准点
                last_aim = aim
                error = int(aim[0] - center_x - offset)
                speed = align_speed(error)
                status = f"圆心：偏差 {error}（半径 {aim[2]:.0f}）"

                if abs(error) <= CENTER_HALF_WIDTH:   # 圆心已到瞄准位置
                    # 标签贴在杯底、要打的是杯口，按两者垂直差微调俯仰角
                    if target is not None:
                        last_dy = int(aim[1] - target.center[1])
                    pitch = max(PITCH_MIN, min(
                        PITCH_MAX, int(base_pitch + last_dy * PITCH_PER_PX)))
                    drive(0)
                    board.set_servo_angle(pitch)
                    time.sleep(0.3)      # 等舵机到位再执行动作
                    print(f"{status} |=> 锁定圆心，俯仰角 {pitch}°"
                          f"（基准 {base_pitch}°，垂直差 {last_dy:+d}px）")
                    return True
            else:
                # 偏差 = 标签位置 −（画面中心 + 任务指定的 offset）
                error = int(target.center[0] - center_x - offset)
                status = f"标签 {tag_id}：偏差 {error}"

                if abs(error) <= CENTER_HALF_WIDTH:
                    if not aim_circle:
                        print(f"{status} |=> 锁定目标")
                        drive(0)
                        return True
                    speed = 0            # 标签已居中，原地等附近出现圆
                    status = f"{status}，等待出现圆"
                else:
                    speed = align_speed(error)

        if speed != last_speed:      # 只在速度变化时发指令
            drive(speed)
            last_speed = speed
            status = f"{status} |=> 发送指令"
            print(status)

        vision.show_frame(frame, tags, status_with_lock(status), circles)
        result = key_result(wait_key())
        if result is not None:
            return result


def track_circle(camera, offset, speed_limit=MOTOR_SPEED):
    """跟踪画面里的圆（多个时取半径最大）直到圆心到达瞄准位置。"""

    last_speed = 0
    while True:
        frame = vision.read_frame(camera)
        if frame is None:
            continue

        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        circles = vision.detect_circles(frame)

        if not circles:
            speed = speed_limit * VOLTAGE_POLARITY   # 没发现圆，转动搜索
            status = "未发现圆，搜索中"
        else:
            x, y, r = max(circles, key=lambda c: c[2])
            error = int(x - center_x - offset)
            speed = motor_speed(error, speed_limit)
            status = (f"圆心：偏差 {error}（X:{x - center_x:.0f} "
                      f"Y:{y - center_y:.0f}，半径 {r:.0f}）")

            if abs(error) <= CENTER_HALF_WIDTH:
                print(f"{status} |=> 锁定圆心")
                drive(0)
                return True

        if speed != last_speed:      # 只在速度变化时发指令
            drive(speed)
            last_speed = speed
            status = f"{status} |=> 发送指令"
            print(status)

        vision.show_frame(frame, [], status, circles)
        result = key_result(wait_key())
        if result is not None:
            return result


# ============================== 主流程 ==============================
def main():
    detector = vision.create_detector()   # 创建标签检测器
    camera = vision.open_camera()         # 打开摄像头

    try:
        board.connect(SERIAL_PORT)
        board.set_servo_angle(90)   # 云台初始俯仰
        drive(0)                    # 一开始云台不动
        print("云台已就绪，开始按顺序执行任务。"
              "按数字键 1~9（0=任务 10）可随时切换任务。")

        i = 0   # 当前任务下标；数字键跳转后从这里继续按顺序执行
        while i < len(TASKS):
            set_current_task(i + 1)
            task = TASKS[i]
            # 任务 1/2/3 用原速度，任务 4 起降到 150（跳回前三个会自动恢复）
            speed_limit = MOTOR_SPEED if i < 3 else LATER_MOTOR_SPEED
            is_circle = bool(task.get("circle"))
            aim_circle = (not is_circle
                          and bool(task.get("aim_circle", AIM_CIRCLE_NEAR_TAG)))
            offset = task.get("offset", AIM_OFFSET_PX)
            start_angle = task.get("angle", PITCH_START)
            if is_circle:
                target_name = "找圆"
            elif aim_circle:
                target_name = f"标签 {task['tag']} + 圆心"
            else:
                target_name = f"标签 {task['tag']}"
            print(f"\n=== 任务 {i+1}/{len(TASKS)}：{target_name}，"
                  f"动作 {task['action']}，瞄准位置 {offset:+d} ===")

            angle = start_angle
            if AUTO_PITCH:      # 自动识别俯仰角度（结果只在本次运行使用）
                angle, ok = scan_pitch(camera, detector,
                                       None if is_circle else task["tag"],
                                       is_circle or aim_circle, start_angle,
                                       speed_limit)
                if not ok:
                    break
            board.set_servo_angle(angle)

            if is_circle:
                result = track_circle(camera, offset, speed_limit)
            else:
                # 任务 1/2/3 不走导航（开局目标就在眼前，直接原地等）；
                # 朝向未知时返回 None，走原来的搜索流程
                nav = (None if i < 3
                       else goto_map_position(camera, detector, task["tag"]))
                if nav is False:
                    break
                result = track_tag(camera, detector, task["tag"], offset,
                                   aim_circle, start_angle,
                                   task.get("search", True), speed_limit,
                                   HEADING_SPEED if nav else None)
            if result is False:     # 用户退出
                break
            if type(result) is int:  # 数字键跳转
                i = result - 1
                print(f"\n>>> 切换到任务 {i+1}/{len(TASKS)}")
                continue

            result = execute_action(task["action"], camera, detector)
            if result is False:
                break
            if type(result) is int:
                i = result - 1
                print(f"\n>>> 切换到任务 {i+1}/{len(TASKS)}")
                continue

            hold = task.get("hold", 0)
            if hold:
                print(f"    停留 {hold} 秒……")
                if vision.is_quit_key(pump_frames(camera, detector, hold,
                                                  "停留中……")):
                    break
            i += 1
        else:
            print("\n所有任务完成。")
    finally:
        # 收尾：不管正常退出还是出错，都会执行
        if board.is_connected():
            drive(0)
            board.disconnect()
        camera.release()           # 关掉摄像头
        vision.close_windows()     # 关掉窗口


if __name__ == "__main__":
    main()
