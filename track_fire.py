"""按任务顺序执行：调整舵机俯仰 → 跟踪标签 / 找圆 → 执行动作（激光 / 发射）

任务定义（TASKS）：
    每个任务对应一个特定编号的 AprilTag，包含：
        tag     ：要跟踪的标签编号
        offset  ：标签期望位置，相对画面中心的像素偏移（右正左负，0 即画面中心）
        angle   ：跟踪该标签时舵机（俯仰）角度，0～180
        action  ：标签到位后执行的动作，"laser" 发射激光 或 "fire" 发射一次
         hold    ：可选，动作执行完成后停留的秒数（云台保持不动），缺省为不停留

    找圆任务（把 tag 换成 circle=True 即可）：
        circle  ：True 表示不找标签，改为寻找画面中的圆，圆心到位后执行动作
        offset  ：圆心期望位置（通常与标签任务的瞄准点相同，即发射线位置）
        其余字段与标签任务一致。

    标签附近找圆（由 circle_aim.py 提供，开关 AIM_CIRCLE_NEAR_TAG 控制）：
        标签任务在锁定标签后，会在标签附近找圆并把圆心对准到 offset
        位置，对准后才执行动作；标签附近暂时没有圆时，云台原地等待圆出现。
        某个任务不需要找圆时，可在该任务里加 "aim_circle": False。

    自动识别（默认开启）：
        俯仰角度：AUTO_PITCH 打开时，每个任务开始前先自动扫描舵机俯仰角，
                  找出目标垂直居中的角度，不必在任务里手填 angle
                  （填了则作为扫描起点）。识别结果只在本次运行使用。
        期望位置：任务不写 offset 时，水平瞄准位置自动取 AIM_OFFSET_PX
                  （默认画面中心 0）；写了 offset 则沿用任务里的值。
        搜索方式：任务写 "search": False 时，未发现该标签云台不转动、
                  原地等待标签出现在画面里（任务 1 默认如此）。

操作：在摄像头窗口按数字键 1~9 立即跳到对应任务（0 表示任务 10），
      跳过后从该任务继续按原顺序执行；按 Q 或 Esc 退出。
"""

import time

from board import board
from vision import vision
from config import (SERIAL_PORT, VOLTAGE_POLARITY, GIMBAL_SPEED_REF,
                    GIMBAL_DEG_PER_SEC, GIMBAL_CLOCKWISE_SIGN)
from field_map import (TAG_ANGLE, TAG_WINDOW, angle_diff, in_window,
                       shortest_delta)
from circle_aim import (AIM_CIRCLE_NEAR_TAG, aim_target, circle_pitch,
                        pick_circle_near, track_circle)

MOTOR_SPEED = 180        # 任务 1/2/3 的云台转速度
LATER_MOTOR_SPEED = 150  # 完成标签 1/2/3 后，任务 4 起的云台转速度
CENTER_HALF_WIDTH = 10   # 标签离画面中心多少像素以内算"居中"
CENTER_DIRECTION = -1    # 标签/圆心回中的旋转方向：+1 默认，-1 反向（不影响搜索方向）
DETECT_SETTLE = 0.15     # 识别到标签后先停止左右转动、等云台停稳的秒数

# 自动识别舵机俯仰角度：每个任务开始前扫描角度，找目标垂直居中的位置
# 扫描会在 60～140° 之间来回摆动，对固定角度的老任务列表默认关闭；
# 需要自动找角度时改为 True（任务里的 angle 则作为扫描起点）
AUTO_PITCH = False        # 总开关；False 时沿用任务里写死的 angle
PITCH_START = 90          # 扫描起点（任务写了 angle 则用它当起点）
PITCH_MIN = 60            # 扫描下限（度）
PITCH_MAX = 140           # 扫描上限（度）
PITCH_SCAN_STEP = 3       # 扫描步长（度）
PITCH_SCAN_DWELL = 0.5    # 每个角度停留观察的秒数
PITCH_TOLERANCE = 20      # 目标垂直偏差多少像素以内算已对准

# 自动识别标签期望位置：任务未写 offset 时使用的水平瞄准位置
AIM_OFFSET_PX = 0         # 0 表示画面中心

# ============ 预置地图导航（field_map.py）============
# 每个任务的方位在场地地图里是固定的：任务开始先按"朝向 → 目标方位"的
# 最短路径定时转到目标附近，停稳后再静止识别，避免边转边认画面拖影。
HEADING_SPEED = 255       # 导航转动速度：用标定过的满速，换算最准
HEADING_MIN_DELTA = 3     # 转角小于这个度数就不转，避免来回微抖
COARSE_SWITCH_PX = 120    # 导航结束后：偏差超过该像素先用导航速度靠过去，靠近再降速
MAP_CONFIRM_FRAMES = 3    # 转到目标方位后，静止连拍多少帧确认目标
HEADING_LEARN_TOL_PX = 100  # 标签离画面中心多少像素以内，才用它校正朝向

# 范围搜索：在地图给的"目标方位 ± half_width"内往复找目标，
# 不转出范围（兜底方式：范围内一直找，不退回整圈搜索）。
WINDOW_SEARCH_SPEED = 90  # 范围搜索的转动速度：比正常搜索慢，减少拖影
SEARCH_STEP_DEG = 8       # 范围搜索每一小步的角度（走一步停一下再看）

_heading = None           # 当前朝向（场上角度 0～360）；None 表示还没识别到标签
_scan_dir = 1             # 范围搜索的当前方向：+1 角度增大、-1 角度减小
_current_task_no = 1      # 当前任务序号（1 起）：A 键解锁后据此重做当前任务

# 任务列表：按顺序逐个执行
TASKS = [
    # tag：标签编号；angle：舵机俯仰角度；
    # action：动作（laser 激光 / fire 发射）；offset：标签期望位置（右正左负，0 即画面中心）
    # search：可选，False 表示未发现该标签时云台不转动、原地等待（缺省为搜索转动）
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
    # "aim_circle": True  使用时，标签任务会自动找圆并对准圆心，对准后才执行动作
    # 找圆任务示例：去掉下面一行的注释，就会在标签任务后追加"找圆 → 对准圆心 → 发射"
    # {"circle": True, "offset": -20, "angle": 102, "action": "fire"},
]



def motor_speed(error, speed_limit=MOTOR_SPEED):
    """根据偏差算出云台速度。

    输入：error 偏差；speed_limit 电机速度幅值
    输出：速度；偏差在居中范围内时返回 0。
    """

    if error > CENTER_HALF_WIDTH:
        return -speed_limit * VOLTAGE_POLARITY * CENTER_DIRECTION   # 标签偏右，转向回中
    if error < -CENTER_HALF_WIDTH:
        return speed_limit * VOLTAGE_POLARITY * CENTER_DIRECTION    # 标签偏左，转向回中
    return 0                                     # 标签居中


def key_to_task_index(key):
    """把按键转换成任务序号。

    数字键 1～9 对应任务 1～9，数字键 0 对应任务 10；其余按键返回 None。
    输入：key 按键值；输出：任务序号（1 起）或 None。
    """

    if ord("1") <= key <= ord("9"):
        return key - ord("0")
    if key == ord("0"):
        return 10
    return None


def set_current_task(number):
    """记录当前正在执行的任务序号（1 起），供 A 键解锁时重做。"""

    global _current_task_no
    _current_task_no = number


def task_key(number):
    """任务序号 → 对应的数字键编码（1～9 原样，10 用 0）。"""

    return ord("0") if number >= 10 else ord(str(number))


def wait_key():
    """读按键；A 键刚解锁时，返回"当前任务的数字键"，等于重做当前任务。

    解锁后不原地补发速度（那样会自己乱转），而是走和手按数字键完全
    相同的跳转逻辑：停住 → 重新导航 → 重新对准 → 重新执行。
    输入：无
    输出：按键编码；没按键返回 255
    """

    if board.take_unlock_event():
        # 锁定期间云台可能被人手拨动：水平朝向作废，舵机复位到安全角，
        # 再由当前任务重新摆俯仰、重新导航对准。
        global _heading
        _heading = None
        board.stop_motor_b()
        ok = board.reset_servo()
        print(f"\n>>> A 键解锁：舵机复位 {'成功' if ok else '未确认'}，"
              f"重做任务 {_current_task_no}")
        return task_key(_current_task_no)
    return vision.wait_key()


def heading():
    """当前朝向（场上角度）；None 表示还没通过标签标定过。"""

    return _heading


def learn_heading(tag_id):
    """看到已知标签时，用地图把当前朝向校正为准确值。

    输入：tag_id 看到的标签编号；输出：无（不认识该编号时不做改动）
    """

    global _heading
    if tag_id in TAG_ANGLE:
        _heading = TAG_ANGLE[tag_id] % 360


def learn_position(frame, tags):
    """云台停稳时，用画面里居中的已知标签校正朝向（位置自校正）。

    停下来的那一帧里，居中的标签方位就是云台朝向，可以重新确认
    自己在地图上的位置，抵消开环定时转动的累积误差。
    输入：frame 当前帧；tags 识别到的标签列表；输出：无
    """

    if frame is None or not tags:
        return
    center_x = frame.shape[1] // 2
    for seen in tags:
        if (seen.tag_id in TAG_ANGLE
                and abs(seen.center[0] - center_x) <= HEADING_LEARN_TOL_PX):
            old = _heading
            learn_heading(seen.tag_id)   # 居中标签：朝向就是它的方位
            if old is None or abs(angle_diff(_heading, old)) > 1:
                print(f"    位置校正：看到标签 {seen.tag_id} → 朝向 "
                      f"{_heading:.0f}°")
            return


def read_tags(camera, detector, learn=True):
    """读一帧并识别标签，顺便用画面里居中的已知标签校正云台朝向。

    只有标签接近画面中心时，它的方位才约等于云台朝向；偏差太大
    （超过 HEADING_LEARN_TOL_PX）只用来识别，不参与朝向校正。
    导航期间传 learn=False：否则转到一半扫过别的标签会把朝向改写，
    导航的转角基准跟着变，看起来像"一看到别的标签就停住不转"。
    输入：camera/detector 视觉对象；learn 是否用画面标签校正朝向
    输出：frame 当前帧；tags 识别到的标签列表
    """

    frame, tags = vision.read_tags(camera, detector)
    if learn:
        learn_position(frame, tags)
    return frame, tags


def rotate_by(camera, detector, delta, speed=HEADING_SPEED, label="地图导航"):
    """按角度定时转动云台（开环），转动期间画面保持刷新。

    输入：delta 转角（度），正数按"角度增大"方向转；speed 电机速度；
         label 画面/日志里的动作名称
    输出：True 正常转完；False 用户按 Q/Esc 中途退出
    """

    global _heading

    if _heading is not None:   # 先记账：按时长转动，转了多少就加多少
        _heading = (_heading + delta) % 360
    if abs(delta) < HEADING_MIN_DELTA:
        return True            # 差得太小，不值得动

    deg_per_sec = GIMBAL_DEG_PER_SEC * speed / GIMBAL_SPEED_REF   # 按速度比例换算
    seconds = abs(delta) / max(deg_per_sec, 1e-3)
    direction = 1 if delta > 0 else -1
    if label == "地图导航":
        print(f"    地图导航：转 {delta:+.0f}°（{seconds:.1f} 秒）")
    key = 255
    board.set_motor_b(speed * GIMBAL_CLOCKWISE_SIGN * direction)
    try:
        key = pump_frames(camera, detector, seconds,
                          f"{label}：转 {delta:+.0f}°", learn=False)
    finally:
        board.stop_motor_b()
    if vision.is_quit_key(key):
        _heading = None        # 半路被打断，实际朝向不确定，等下次看到标签再定
        return False
    return True


def rotate_to(camera, detector, target_angle, speed=HEADING_SPEED):
    """转到指定场上方位角（地图上的绝对角度）。

    输入：target_angle 目标方位（度）；speed 电机速度
    输出：True 正常；False 用户退出；None 朝向未知（调用方要先搜索标签）
    """

    if _heading is None:
        return None
    return rotate_by(camera, detector,
                     shortest_delta(_heading, target_angle), speed)


def window_step(camera, detector, tag_id, speed=WINDOW_SEARCH_SPEED):
    """在目标的地图角度范围内走一小步（走一步停一下）。

    从当前朝向沿 _scan_dir 走 SEARCH_STEP_DEG；碰到范围边界就停在
    边界上并反向，保证永远不转出地图给的范围。
    输入：tag_id 目标标签；speed 转动速度（范围搜索用较低速度）
    输出：True 正常走完；False 用户按 Q/Esc 退出
    """

    global _scan_dir

    window = TAG_WINDOW.get(tag_id)
    if window is None or _heading is None:
        return True

    lo, hi = window
    center = TAG_ANGLE[tag_id]
    half = (hi - lo) / 2.0
    step = min(SEARCH_STEP_DEG, half) if half > 0 else 0

    offset = angle_diff(_heading, center)          # 当前相对范围中心的偏移
    next_offset = offset + _scan_dir * step
    if abs(next_offset) >= half - 1e-6:            # 到达/越过边界：停在边界并反向
        offset = half if next_offset > 0 else -half
        _scan_dir = -_scan_dir
    else:
        offset = next_offset

    delta = angle_diff(center + offset, _heading)
    if rotate_by(camera, detector, delta, speed, label="范围搜索") is False:
        return False
    board.stop_motor_b()
    time.sleep(DETECT_SETTLE)   # 等云台停稳，下一帧才用来识别
    return True


def goto_map_position(camera, detector, tag_id):
    """任务开始：按预置地图转到目标方位，停稳后再静止取帧确认。

    流程：先用画面里已有的已知标签校正朝向 → 按最短路径转到该任务的
    场上方位 → 停稳后连拍几帧 → 看到目标就交给精细对准，看不到就
    退回原来的连续搜索（兜底）。
    输入：camera/detector 视觉对象；tag_id 本任务要打的标签
    输出：True 继续执行；False 用户按 Q/Esc；None 朝向未知（不做导航）
    """

    frame, tags = read_tags(camera, detector, learn=False)
    if any(seen.tag_id == tag_id for seen in tags):
        return True              # 目标本来就在画面里，不用转
    if _heading is None:
        return None              # 一个已知标签都没看到，交给原搜索逻辑
    target = TAG_ANGLE.get(tag_id)
    if target is None:
        return None              # 地图里没有这个目标，按原流程走

    print(f"    地图导航：当前朝向 {_heading:.0f}° → 目标 {target}°")
    if rotate_to(camera, detector, target) is False:
        return False             # 用户中途退出

    board.stop_motor_b()         # 到点先停稳
    time.sleep(DETECT_SETTLE)
    for _ in range(MAP_CONFIRM_FRAMES):
        frame, tags = read_tags(camera, detector, learn=False)
        if frame is not None:
            vision.show_frame(frame, tags, f"地图导航：确认标签 {tag_id}")
        if any(seen.tag_id == tag_id for seen in tags):
            learn_heading(tag_id)   # 看到目标，顺便把朝向校得更准
            print(f"    地图导航：已到位，看到标签 {tag_id}")
            return True
        if vision.is_quit_key(wait_key()):
            return False
    print(f"    地图导航：已到位但没看到标签 {tag_id}，改用搜索")
    return True



def status_with_lock(status):
    """画面状态栏附加 A 键锁定提示。

    A 键锁定时固件会丢弃 @MB 指令并停转，解锁后由上层把当前任务重做，
    这里只负责让操作者看得见。
    """

    return f"{status}  【A键锁停】" if board.motor_b_locked() else status


def pump_frames(camera, detector, seconds, status, learn=True):
    """等待指定秒数，期间持续刷新画面并响应按键。

    动作（激光/发射）和停留都要等好几秒，如果直接 time.sleep，
    画面窗口不会刷新，看起来就是"卡住不动"，按键也切不了任务。
    输入：camera/detector 视觉对象；seconds 等待秒数；status 画面提示文字
    输出：期间按下的按键编码；没按键返回 255（wait_key 的无按键值）
    """

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        frame, tags = read_tags(camera, detector, learn)
        if frame is not None:
            vision.show_frame(frame, tags, status_with_lock(status))
        key = wait_key()
        if key != 255:      # waitKey 没按键时返回 -1，取低 8 位后是 255
            return key
    return 255


def execute_action(action, camera, detector):
    """执行任务指定的动作，执行期间画面保持实时刷新。

    输入：action 动作名，"laser" 发射激光 或 "fire" 发射一次；
         camera/detector 视觉对象（用来在等待期间刷新窗口）
    输出：True 继续执行；False 用户按 Q/Esc 退出；
         整数 n 表示动作期间按了数字键，请求跳到任务 n。
    """

    if action == "laser":
        board.laser_on()
        try:   # finally 保证中途退出也一定关激光
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

    if vision.is_quit_key(key):
        return False
    jump = key_to_task_index(key)
    if jump is not None and jump <= len(TASKS):
        return jump
    return True


def poll_circle_key():
    """找圆任务每帧读一次按键（圆相关的按键处理留在主程序里）。

    输入：无
    输出：None 继续；True 用户按 Q/Esc 退出；
         整数 n 表示按了数字键，请求跳到任务 n。
    """

    key = wait_key()
    if vision.is_quit_key(key):
        print("按 Q 或 Esc 退出。")
        return True
    jump = key_to_task_index(key)
    if jump is not None and jump <= len(TASKS):
        board.set_motor_b(0)
        return jump
    return None


def scan_pitch(camera, detector, tag_id, use_circle, start_angle,
               speed_limit=MOTOR_SPEED):
    """自动扫描舵机俯仰角度，找出目标垂直居中的角度。

    从 start_angle 出发，向上下两侧交替扩展扫描；每到一个角度停留
    PITCH_SCAN_DWELL 秒，同时让云台水平搜索，使目标进入画面。目标垂直
    偏差（目标 Y − 画面中心 Y）进入 PITCH_TOLERANCE 即提前结束。
    输入：camera/detector 视觉对象；tag_id 目标标签 id（None 表示只找圆）；
         use_circle True 表示以圆为瞄准目标；start_angle 扫描起点角度
    输出：(angle, ok)：angle 识别到的角度；ok 为 False 表示用户退出。
    """

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
    board.set_motor_b(speed_limit * VOLTAGE_POLARITY)   # 扫描同时让云台水平搜索
    try:
        for angle in angles:
            board.set_servo_angle(angle)
            deadline = time.monotonic() + PITCH_SCAN_DWELL
            while time.monotonic() < deadline:
                # 扫描俯仰时云台也在水平转动，朝向时刻在变，不在这里校正
                frame, tags = read_tags(camera, detector, learn=False)
                if frame is None:
                    continue

                center_y = frame.shape[0] // 2                       # 画面中心 Y
                circles = vision.detect_circles(frame) if use_circle else None
                ref = aim_target(tags, circles, tag_id, use_circle)

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
        board.stop_motor_b()

    if best_dy is None:
        print(f"    未发现目标，俯仰角度沿用 {best_angle}°")
    else:
        print(f"    自动识别俯仰角度：{best_angle}°"
              f"（垂直偏差 {best_dy} 像素）")
    return best_angle, True


def track_tag(camera, detector, tag_id, offset, aim_circle=False,
              base_pitch=PITCH_START, search=True, speed_limit=MOTOR_SPEED,
              coarse_speed=None):
    """跟踪指定 id 的标签直到期望位置（可改为对准标签附近的圆心）。

    输入：camera/detector 视觉对象；tag_id 目标标签 id；
         offset 期望的水平位置（相对画面中心的像素偏移，画面中心为 0）；
         aim_circle True 时，标签锁定后继续对准离标签最近的圆心；
         base_pitch 基准俯仰角（任务里的 angle），锁定圆心后按
         "圆心 − 标签中心"的垂直像素差微调（换算见 circle_aim.py）；
         search True 时未发现目标会左右转动搜索，False 则原地等待；
         speed_limit 精细速度；coarse_speed 粗调速度（缺省 None 表示不用）：
             找到目标前一直用粗调速度搜索，偏差大于 COARSE_SWITCH_PX
             时也用粗调速度靠过去，进入小偏差后才降到精细速度
    输出：True 已到位；False 用户按 Q/Esc 退出；
         整数 n 表示按了数字键，请求跳到任务 n。
    """

    def align_speed(error):
        """对准速度：远距离用粗调速度，接近后降到精细速度。"""

        if coarse_speed and abs(error) > COARSE_SWITCH_PX:
            return motor_speed(error, coarse_speed)
        return motor_speed(error, speed_limit)

    search_speed = coarse_speed or speed_limit   # 找到目标前用的转动速度
    last_speed = 0   # 上一次发送的速度（0 表示停止）
    last_aim = None  # 上一次对准的圆 (x, y, r)，标签丢失时按它找回圆
    last_dy = 0      # 上一次"圆心 − 标签中心"的垂直像素差（标签闪断时沿用）
    searching = True # 是否处于左右转动搜索目标的状态
    while True:
        # 搜索中云台一直在转，此刻看到的标签只能说明"上一瞬间"的朝向，
        # 拿来校正会把朝向记成旧值；只有还没定位过自己时才靠它引导一次。
        frame, tags = read_tags(camera, detector,
                                learn=not searching or _heading is None)
        if frame is None:
            continue

        center_x = frame.shape[1] // 2                          # 画面中心
        target = next((t for t in tags if t.tag_id == tag_id), None)  # 找目标标签

        aim = None
        circles = None
        if aim_circle:
            circles = vision.detect_circles(frame)
            aim = pick_circle_near(circles, target, last_aim)

        if target is None and aim is None:
            if search:
                # 没发现目标：只在地图给的角度范围内找，不转出范围
                searching = True
                window = TAG_WINDOW.get(tag_id)
                if window is not None and _heading is not None:
                    # 停稳的这帧先确认自身位置（看到邻近标签就校正朝向）
                    learn_position(frame, tags)
                    if not in_window(tag_id, _heading):
                        rotate_to(camera, detector, TAG_ANGLE[tag_id],
                                  WINDOW_SEARCH_SPEED)   # 先回到范围内
                    if window_step(camera, detector, tag_id) is False:
                        return False
                    speed = 0      # 已由 window_step 走完一步并停下
                    status = (f"标签 {tag_id}：范围搜索 "
                              f"{window[0]:.0f}°～{window[1]:.0f}°")
                else:
                    # 朝向还未知：先连续转圈，靠画面里的标签定位自己
                    speed = search_speed * VOLTAGE_POLARITY
                    if _heading is not None:
                        speed = 0   # 刚认出朝向，停下改走范围搜索
                        status = (f"标签 {tag_id}：已定位朝向 "
                                  f"{_heading:.0f}°，转入范围搜索")
                    else:
                        status = f"标签 {tag_id}：未发现，搜索中"
            else:
                # 不搜索：云台保持不动，原地等待标签进入画面
                speed = 0
                status = f"标签 {tag_id}：未发现，原地等待"
        else:
            if searching:   # 刚识别到目标：先停止左右转动，再做精细对准
                searching = False
                if search:
                    board.set_motor_b(0)
                    last_speed = 0
                    print(f"    识别到标签 {tag_id}，停止左右转动，开始对准。")
                    time.sleep(DETECT_SETTLE)   # 等云台停稳再测偏差

            if aim is not None:
                # 标签附近有圆：以圆心为瞄准点
                last_aim = aim
                error = int(aim[0] - center_x - offset)
                speed = align_speed(error)
                status = f"圆心：偏差 {error}（半径 {aim[2]:.0f}）"

                if abs(error) <= CENTER_HALF_WIDTH:   # 圆心已到达瞄准位置
                    # 按圆心相对标签中心的垂直像素差微调俯仰角：
                    # 标签贴在杯内底部，要打进的是杯口，两者高度差不一致
                    pitch, last_dy = circle_pitch(base_pitch, aim, target,
                                                  last_dy, PITCH_MIN, PITCH_MAX)
                    board.set_motor_b(0)  # 云台停止
                    board.set_servo_angle(pitch)  # 按圆心位置微调俯仰角
                    time.sleep(0.3)               # 等舵机走到位再执行动作
                    print(f"{status} |=> 锁定圆心，俯仰角 {pitch}°"
                          f"（基准 {base_pitch}°，垂直差 {last_dy:+d}px）")
                    return True
            else:
                # 偏差 = 测量值（标签位置）− 期望值（画面中心 + 任务指定的 offset）
                error = int(target.center[0] - center_x - offset)
                status = f"标签 {tag_id}：偏差 {error}"

                if abs(error) <= CENTER_HALF_WIDTH:   # 偏差在阈值内，认为已居中
                    if aim_circle:
                        speed = 0                     # 标签已居中，原地等待附近出现圆
                        status = f"{status}，等待出现圆"
                    else:
                        print(f"{status} |=> 锁定目标")
                        board.set_motor_b(0) # 云台停止
                        return True
                else:
                    speed = align_speed(error)

        # 只在速度变化时发送命令
        if speed != last_speed:
            board.set_motor_b(speed)
            last_speed = speed
            status = f"{status} |=> 发送指令"
            print(status)

        vision.show_frame(frame, tags, status_with_lock(status), circles)
        key = wait_key()
        if vision.is_quit_key(key):
            print("按 Q 或 Esc 退出。")
            return False
        jump = key_to_task_index(key)   # 数字键：停住云台，返回任务序号
        if jump is not None and jump <= len(TASKS):
            board.set_motor_b(0)
            return jump


def main():
    detector = vision.create_detector() # 创建标签检测器
    camera = vision.open_camera() # 打开摄像头

    try:
        board.connect(SERIAL_PORT) # 连接控制板
        board.set_servo_angle(90)  # 云台初始角度
        board.stop_motor_b()   # 一开始云台不动
        print("云台已就绪，开始按顺序执行任务。"
              "按数字键 1~9（0=任务 10）可随时切换任务。")

        i = 0   # 当前任务下标（0 起）；数字键跳转后从这里继续按顺序执行
        while i < len(TASKS):
            set_current_task(i + 1)   # A 键解锁时按这个序号重做当前任务
            task = TASKS[i]
            # 标签 1/2/3 沿用原速度；从任务 4 起降为 150，数字键跳回前 3 发也会恢复
            speed_limit = MOTOR_SPEED if i < 3 else LATER_MOTOR_SPEED
            is_circle = bool(task.get("circle"))   # True：找圆任务；False：标签任务
            # 标签任务默认锁定后继续对准附近的圆心（总开关 AIM_CIRCLE_NEAR_TAG，
            # 单个任务可用 "aim_circle": False 关闭）
            aim_circle = (not is_circle
                          and bool(task.get("aim_circle", AIM_CIRCLE_NEAR_TAG)))
            offset = task.get("offset", AIM_OFFSET_PX)   # 没写 offset 就用自动瞄准位置
            start_angle = task.get("angle", PITCH_START)  # 没写 angle 就从默认角度开始扫描
            if is_circle:
                target_name = "找圆"
            elif aim_circle:
                target_name = f"标签 {task['tag']} + 圆心"
            else:
                target_name = f"标签 {task['tag']}"
            print(f"\n=== 任务 {i+1}/{len(TASKS)}：{target_name}，"
                  f"动作 {task['action']}，瞄准位置 {offset:+d} ===")

            angle = start_angle
            if AUTO_PITCH:   # 自动识别俯仰角度（结果只在本次运行使用）
                angle, ok = scan_pitch(camera, detector,
                                       None if is_circle else task["tag"],
                                       is_circle or aim_circle, start_angle,
                                       speed_limit)
                if not ok:   # 用户退出，跳过剩余任务
                    break
            board.set_servo_angle(angle)  # 调整装置俯仰角度

            if is_circle:  # 找圆任务：对准圆心
                result = track_circle(camera, offset, speed_limit,
                                      motor_speed, CENTER_HALF_WIDTH,
                                      poll_circle_key)
            else:          # 标签任务：跟踪标签（可再对准附近圆心）
                # 先用预置地图转到目标方位，停稳后再静止识别；
                # 任务 1/2/3 不走地图导航（开局目标就在眼前，直接原地等），
                # 朝向未知时返回 None，直接走原来的搜索流程
                nav = (None if i < 3
                       else goto_map_position(camera, detector, task["tag"]))
                if nav is False:   # 用户退出，跳过剩余任务
                    break
                result = track_tag(camera, detector, task["tag"], offset,
                                   aim_circle, start_angle,
                                   task.get("search", True), speed_limit,
                                   HEADING_SPEED if nav else None)
            if result is False:   # 用户退出，跳过剩余任务
                break
            if type(result) is int:   # 按了数字键：跳到对应任务重新执行
                i = result - 1
                print(f"\n>>> 切换到任务 {i+1}/{len(TASKS)}")
                continue

            result = execute_action(task["action"], camera, detector)
            if result is False:   # 动作执行期间按了 Q/Esc，跳过剩余任务
                break
            if type(result) is int:   # 动作执行期间按了数字键：跳到对应任务
                i = result - 1
                print(f"\n>>> 切换到任务 {i+1}/{len(TASKS)}")
                continue
            hold = task.get("hold", 0)      # 可选：动作后停留，云台保持不动
            if hold:
                print(f"    停留 {hold} 秒……")
                if vision.is_quit_key(pump_frames(camera, detector, hold,
                                                  "停留中……")):
                    break
            i += 1   # 正常完成，进入下一个任务
        else:
            print("\n所有任务完成。")
    finally:
        # 收尾：不管正常退出还是出错，都会执行
        if board.is_connected():   # 已连接才需要停机和断开
            board.stop_motor_b()
            board.disconnect()
        camera.release()           # 关掉摄像头
        vision.close_windows()     # 关掉窗口


if __name__ == "__main__":
    main()
