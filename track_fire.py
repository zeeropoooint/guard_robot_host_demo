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
        其余字段与标签任务一致；多个圆时自动选择半径最大的那个。

    标签附近找圆（默认开启，由 AIM_CIRCLE_NEAR_TAG 控制）：
        标签任务在锁定标签后，会自动在标签附近找圆并把圆心对准到 offset
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

import math
import time

from board import board
from vision import vision
from config import SERIAL_PORT, VOLTAGE_POLARITY

MOTOR_SPEED = 160        # 云台转速度
SEARCH_SPEED = MOTOR_SPEED * VOLTAGE_POLARITY   # 未发现目标标签时，向一个方向旋转搜索
CENTER_HALF_WIDTH = 10   # 标签离画面中心多少像素以内算"居中"
CENTER_DIRECTION = -1    # 标签/圆心回中的旋转方向：+1 默认，-1 反向（不影响搜索方向）
DETECT_SETTLE = 0.15     # 识别到标签后先停止左右转动、等云台停稳的秒数

# 标签附近找圆：标签锁定后，找离标签最近的圆（杯口），对准圆心后才执行动作。
# 总开关默认关闭，需要该功能的任务在任务里写 "aim_circle": True
AIM_CIRCLE_NEAR_TAG = False   # 总开关；单个任务可用 "aim_circle": True 打开
CIRCLE_NEAR_FACTOR = 2.5      # "附近"范围 = 标签对角线长度 × 该倍数
CIRCLE_NEAR_MIN = 100         # "附近"范围的下限（像素），标签很小时保底

# 俯仰角微调：摄像头固定、发射器在其头顶，标签贴在杯内底部，
# 而要打进的是杯口（圆心），两者在画面里的垂直像素差 × 该系数 = 俯仰修正角。
# 标定方法：先设 0 观察打印的垂直差，实测打高就把系数往负调、打低往正调
PITCH_PER_PX = 0.06   # 度/像素

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

# 任务列表：按顺序逐个执行
TASKS = [
    # tag：标签编号；angle：舵机俯仰角度；
    # action：动作（laser 激光 / fire 发射）；offset：标签期望位置（右正左负，0 即画面中心）
    # search：可选，False 表示未发现该标签时云台不转动、原地等待（缺省为搜索转动）
    {"tag": 1, "offset": -45, "angle": 110, "action": "fire", "hold": 6.5, "search": False},
    {"tag": 2, "offset": -45, "angle": 110, "action": "fire", "hold": 6.5},
    {"tag": 3, "offset": -45, "angle": 110, "action": "fire"},
    {"tag": 4, "offset": -24, "angle": 110, "action": "laser"},
    {"tag": 5, "offset": 10, "angle": 105, "action": "fire"},
    {"tag": 6, "offset": -29, "angle": 109, "action": "laser"},
    {"tag": 7, "offset": -99, "angle": 104, "action": "fire"},
    {"tag": 8, "offset": 233, "angle": 90, "action": "fire"},
    {"tag": 9, "offset": -24, "angle": 116, "action": "laser"},
    {"tag": 10, "offset": 9, "angle": 105, "action": "fire"},
    # "aim_circle": True  使用时，标签任务会自动找圆并对准圆心，对准后才执行动作
    # 找圆任务示例：去掉下面一行的注释，就会在标签任务后追加"找圆 → 对准圆心 → 发射"
    # {"circle": True, "offset": -20, "angle": 102, "action": "fire"},
]



def motor_speed(error):
    """根据偏差算出云台速度。

    输入：error 偏差
    输出：速度；偏差在居中范围内时返回 0。
    """

    if error > CENTER_HALF_WIDTH:
        return -MOTOR_SPEED * VOLTAGE_POLARITY * CENTER_DIRECTION   # 标签偏右，转向回中
    if error < -CENTER_HALF_WIDTH:
        return MOTOR_SPEED * VOLTAGE_POLARITY * CENTER_DIRECTION    # 标签偏左，转向回中
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


def execute_action(action):
    """执行任务指定的动作。

    输入：action 动作名，"laser" 发射激光 或 "fire" 发射一次
    输出：无；动作名未知时抛出 ValueError。
    """

    if action == "laser":
        board.laser_on_for(3)
        print("    动作：激光发射 3 秒")
    elif action == "fire":
        board.fire_once()
        time.sleep(3)   # 留时间给发射机构完成动作
        print("    动作：发射一次")
    else:
        raise ValueError(f"未知动作: {action}（只支持 laser / fire）")


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


def scan_pitch(camera, detector, tag_id, use_circle, start_angle):
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
    board.set_motor_b(SEARCH_SPEED)   # 扫描同时让云台水平搜索，目标才会进入画面
    try:
        for angle in angles:
            board.set_servo_angle(angle)
            deadline = time.monotonic() + PITCH_SCAN_DWELL
            while time.monotonic() < deadline:
                frame, tags = vision.read_tags(camera, detector)
                if frame is None:
                    continue

                center_y = frame.shape[0] // 2                       # 画面中心 Y
                circles = vision.detect_circles(frame) if use_circle else None
                if tag_id is None:   # 找圆任务：任意圆都算目标
                    ref = max(circles, key=lambda c: c[2]) if circles else None
                else:                # 标签任务：优先用标签附近的圆
                    target = next((t for t in tags if t.tag_id == tag_id), None)
                    aim = (pick_circle_near(circles, target, None)
                           if use_circle else None)
                    ref = (aim if aim is not None
                           else (target.center if target is not None else None))

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

                vision.show_frame(frame, tags, status, circles)
                if vision.is_quit_key(vision.wait_key()):
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
              base_pitch=PITCH_START, search=True):
    """跟踪指定 id 的标签直到期望位置（可改为对准标签附近的圆心）。

    输入：camera/detector 视觉对象；tag_id 目标标签 id；
         offset 期望的水平位置（相对画面中心的像素偏移，画面中心为 0）；
         aim_circle True 时，标签锁定后继续对准离标签最近的圆心；
         base_pitch 基准俯仰角（任务里的 angle），锁定圆心后按
         "圆心 − 标签中心"的垂直像素差 × PITCH_PER_PX 微调；
         search True 时未发现目标会左右转动搜索，False 则原地等待
    输出：True 已到位；False 用户按 Q/Esc 退出；
         整数 n 表示按了数字键，请求跳到任务 n。
    """

    last_speed = 0   # 上一次发送的速度（0 表示停止）
    last_aim = None  # 上一次对准的圆 (x, y, r)，标签丢失时按它找回圆
    last_dy = 0      # 上一次"圆心 − 标签中心"的垂直像素差（标签闪断时沿用）
    searching = True # 是否处于左右转动搜索目标的状态
    while True:
        frame, tags = vision.read_tags(camera, detector)
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
                # 没发现目标：云台左右转动搜索，直到标签进入画面
                searching = True
                speed = SEARCH_SPEED
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
                speed = motor_speed(error)
                status = f"圆心：偏差 {error}（半径 {aim[2]:.0f}）"

                if abs(error) <= CENTER_HALF_WIDTH:   # 圆心已到达瞄准位置
                    # 按圆心相对标签中心的垂直像素差微调俯仰角：
                    # 标签贴在杯内底部，要打进的是杯口，两者高度差不一致
                    if target is not None:
                        last_dy = int(aim[1] - target.center[1])
                    pitch = int(base_pitch + last_dy * PITCH_PER_PX)
                    pitch = max(PITCH_MIN, min(PITCH_MAX, pitch))  # 限制在安全范围
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
                    speed = motor_speed(error)

        # 只在速度变化时发送命令
        if speed != last_speed:
            board.set_motor_b(speed)
            last_speed = speed
            status = f"{status} |=> 发送指令"
            print(status)

        vision.show_frame(frame, tags, status, circles)
        key = vision.wait_key()
        if vision.is_quit_key(key):
            print("按 Q 或 Esc 退出。")
            return False
        jump = key_to_task_index(key)   # 数字键：停住云台，返回任务序号
        if jump is not None and jump <= len(TASKS):
            board.set_motor_b(0)
            return jump


def track_circle(camera, offset):
    """跟踪画面中的圆，直到圆心到达期望位置。

    输入：camera 摄像头对象；offset 期望的圆心水平位置
         （相对画面中心的像素偏移，画面中心为 0）
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
            speed = SEARCH_SPEED   # 没发现圆，云台转动搜索
            status = "未发现圆，搜索中"
        else:
            # 多个圆时选半径最大的：同心圆即最外圈（圆心相同），
            # 多个目标时最大的一般也是最近、最明显的那个
            x, y, r = max(circles, key=lambda c: c[2])
            error = int(x - center_x - offset)
            speed = motor_speed(error)
            status = (f"圆心：偏差 {error}（X:{x - center_x:.0f} "
                      f"Y:{y - center_y:.0f}，半径 {r:.0f}）")

            if abs(error) <= CENTER_HALF_WIDTH:   # 圆心已到达瞄准位置
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
        key = vision.wait_key()
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
            task = TASKS[i]
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
                                       is_circle or aim_circle, start_angle)
                if not ok:   # 用户退出，跳过剩余任务
                    break
            board.set_servo_angle(angle)  # 调整装置俯仰角度

            if is_circle:  # 找圆任务：对准圆心
                result = track_circle(camera, offset)
            else:          # 标签任务：跟踪标签（可再对准附近圆心）
                result = track_tag(camera, detector, task["tag"], offset,
                                   aim_circle, start_angle,
                                   task.get("search", True))
            if result is False:   # 用户退出，跳过剩余任务
                break
            if type(result) is int:   # 按了数字键：跳到对应任务重新执行
                i = result - 1
                print(f"\n>>> 切换到任务 {i+1}/{len(TASKS)}")
                continue

            execute_action(task["action"])  # 目标到位后执行动作
            hold = task.get("hold", 0)      # 可选：动作后停留，云台保持不动
            if hold:
                print(f"    停留 {hold} 秒……")
                time.sleep(hold)
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
