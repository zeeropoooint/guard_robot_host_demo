"""水平闭环跟踪 + 地图导航 + 按任务顺序执行（覆盖 1～10 号位全部目标）

整体流程：
    对零 —— 先搜索基准标签（①②③ 号位，map.REFERENCE_TAGS）并锁定，
            锁定方向即地图 0°
    导航 —— 每个任务先按 map.py 登记的相对角度定时转向目标大概方位
    跟踪 —— 云台电机 PD 水平跟踪标签；搜索途中一旦看到任何已登记标签
            （"见标修正"），先把该标签对准画面中心（此时当前朝向即其
            地图角度），再按相对角度差快速转向目标方位，不再慢扫
    动作 —— 标签到位后执行动作（激光 / 发射）
    回写 —— 锁定时朝向估计与地图记录差得够多（MIN_RECORD_SPIN）就把
            实测方向记回地图（record_angle），地图越用越准

硬件要点（为什么只做水平闭环）：
    摄像头只随云台水平转动，俯仰固定——画面里标签的垂直位置
    不可控，仅作参考显示；俯仰舵机带动的是激光 / 发射机构，
    负责武器的高低瞄准，每个任务的 angle 就是预先调好的粗瞄准
    角度，跟踪期间保持不变。因此锁定只看水平偏差。

任务定义（TASKS）：
    每个任务对应一个特定编号的 AprilTag，包含：
        tag     ：要跟踪的标签编号（须已登记在 map.py 的 TARGET_ANGLES）
        offset  ：标签期望位置，相对画面中心的像素偏移（右正左负，0 即画面中心）
        angle   ：该标签的武器俯仰粗瞄准角度（0～180，跟踪中保持不变）
        action  ：标签到位后执行的动作，"laser" 发射激光 或 "fire" 发射一次

操作：在摄像头窗口按 Q 或 Esc 退出。
"""

import time

import map
from board import board
from vision import vision
from control import control
from config import SERIAL_PORT, VOLTAGE_POLARITY

MOTOR_SPEED = 180        # 云台转速（搜索 / 定时转向共用，标定见 DEGREES_PER_SECOND）
SEARCH_SPEED = MOTOR_SPEED * VOLTAGE_POLARITY   # 未发现目标标签时，向一个方向旋转搜索
CENTER_HALF_WIDTH = 10   # X 方向：标签离画面中心多少像素以内算"居中"

# —— 地图导航参数（配合 map.py 的水平相对角度）——
ALIGN_PITCH = 102        # 对零时的武器俯仰角度（①②③ 号位高度，取自标签 2 调试值）
DEGREES_PER_SECOND = 5   # 实测标定：MOTOR_SPEED 转速下 5 秒从 5 号位转到 4 号位（约 25°）
TURN_SIGN = 1            # 地图正方向（朝 ④⑤⑥ 一侧）对应的电机方向，反了改成 -1
TURN_TOLERANCE = 3       # 地图算出的转向角小于该度数就不转，直接交给视觉闭环
MIN_RECORD_SPIN = 20     # 朝向估计偏离地图记录达到该度数才回写（过滤检测抖动）
MAX_HEADING_FIXES = 3    # 单次搜索中最多"见标修正"次数（防止无限修正循环）

# 搜索旋转在地图方向上的正负号（把搜索耗时折算成扫过角度时使用）
SPIN_MAP_DIRECTION = (1 if SEARCH_SPEED > 0 else -1) * TURN_SIGN

# —— 跟踪稳态参数（低通滤波 + PD 控制，与 track_pid.py 一致）——
KP = 3.0                 # P 比例系数：偏差每 1 像素对应的转速
KD = 0.5                 # D 微分系数：偏差每秒收敛 1 像素对应的刹车力（0 退回纯 P）
ALPHA = 0.4              # 低通滤波系数 0～1（越小越平滑、响应越迟钝）
MAX_SPEED = 255          # 云台最大速度（转速上限）
MIN_SPEED = 150          # 云台最小速度（太低电机带不动）
D_LIMIT = 150            # 刹车力上限：接近中心时最多减速多少，防止刹车过猛或不刹车

# X 轴稳定控制器（低通滤波 + PD），实现在 control/control.py，与 track_pid.py 共用
x_stabilizer = control.Stabilizer(
    kp=KP,
    kd=KD,
    alpha=ALPHA,
    max_speed=MAX_SPEED,
    min_speed=MIN_SPEED,
    deadzone=CENTER_HALF_WIDTH,
    d_limit=D_LIMIT,
)

# 任务列表：按顺序逐个执行，覆盖 1～10 号位全部目标
TASKS = [
    # tag：标签编号；angle：武器俯仰粗瞄准角度（跟踪中保持不变）；
    # action：动作（laser 激光 / fire 发射）；offset：标签期望位置（右正左负，0 即画面中心）
    # angle / offset 依据装置照片估算：锚点 ②=102、⑥=86、④=100 为实测值；
    # 规律是角度越大枪口越朝下（高位目标 < 102，低位目标 > 102），±5° 内微调
    {"tag": 1, "offset": -20, "angle": 94, "action": "laser"},
    {"tag": 2, "offset": -20, "angle": 102, "action": "laser"},
    {"tag": 3, "offset": -20, "angle": 110, "action": "laser"},
    {"tag": 4, "offset": -20, "angle": 100, "action": "fire"},
    {"tag": 5, "offset": -20, "angle": 106, "action": "laser"},
    {"tag": 6, "offset": -24, "angle": 86, "action": "laser"},
    {"tag": 7, "offset": -20, "angle": 88, "action": "laser"},
    {"tag": 8, "offset": -20, "angle": 100, "action": "laser"},
    {"tag": 9, "offset": -20, "angle": 112, "action": "laser"},
    {"tag": 10, "offset": -20, "angle": 108, "action": "laser"},
]


def motor_speed(error):
    """根据偏差算出云台速度（低通滤波 + PD 控制）。

    输入：error 偏差
    输出：速度；偏差在居中范围内时返回 0。
    """

    return x_stabilizer.update(error) * VOLTAGE_POLARITY


def turn_by(degrees):
    """按地图方向定时转动指定角度（开环，精度取决于 DEGREES_PER_SECOND 标定）。

    输入：degrees 要转的角度（度），正负号表示地图方向
    输出：无；转完自动停机。
    """

    direction = TURN_SIGN * (1 if degrees >= 0 else -1)
    board.set_motor_b(direction * MOTOR_SPEED)
    time.sleep(abs(degrees) / DEGREES_PER_SECOND)
    board.set_motor_b(0)


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
        print(f"   未知动作: {action}")


def record_measured_angle(tag_id, heading):
    """把锁定时推算出的实测方向回写地图。

    输入：tag_id 锁定的标签编号；heading 锁定时刻的朝向估计（地图角度）
    输出：无；标签未登记地图、朝向未知或偏差太小时不回写。
    """

    base = map.angle_of(tag_id)
    if base is None or heading is None:
        return
    measured = heading % 360
    if abs(map.shortest_turn(base, measured)) < MIN_RECORD_SPIN:
        return
    map.record_angle(tag_id, measured)
    print(f"    地图回写：{tag_id} 号位实测约 {measured:.0f}°（原记录 {base}°）")


def center_tag(camera, detector, tag_id, timeout=3.0):
    """把指定标签转到画面水平中心（快速转向前的对准步骤）。

    只有标签位于画面中心时，"当前朝向 = 该标签的地图角度"才严格成立，
    之后的相对角度计算才准确。

    输入：camera/detector 视觉对象；tag_id 要对准的标签编号；
         timeout 最长对准时间（秒）
    输出：True 已对准；超时、跟丢或用户按键返回 False。
    """

    deadline = time.perf_counter() + timeout
    last_speed = 0
    while time.perf_counter() < deadline:
        frame, tags = vision.read_tags(camera, detector)
        if frame is None:
            continue
        target = next((t for t in tags if t.tag_id == tag_id), None)
        if target is None:              # 对准途中跟丢了，视为失败
            board.set_motor_b(0)
            return False
        error = int(target.center[0] - frame.shape[1] // 2)
        if abs(error) < CENTER_HALF_WIDTH:
            board.set_motor_b(0)
            return True
        speed = motor_speed(error)
        if speed != last_speed:
            board.set_motor_b(speed)
            last_speed = speed
        vision.show_frame(frame, tags, f"对准 {tag_id} 号位：X 偏差 {error}")
        if vision.is_quit_key(vision.wait_key()):
            return False
    board.set_motor_b(0)
    return False


def track_tag(camera, detector, tag_ids, offset, heading=None, nav_target=None):
    """跟踪指定标签直到水平到位（云台电机闭环，锁定只看水平偏差）。

    输入：camera/detector 视觉对象；tag_ids 目标标签编号（单个整数，
         或对零时的编号元组）；offset 期望的标签水平位置（相对画面中心，
         画面中心为 0）；heading 当前朝向估计（地图角度，对零时未知传
         None）；nav_target 目标标签的地图角度（对零传 None，不启用
         见标修正）
    输出：(锁定的标签编号, 结束时的朝向估计)；用户按 Q/Esc 退出时
         标签编号为 None。
    """

    wanted = {tag_ids} if isinstance(tag_ids, int) else set(tag_ids)
    label = "+".join(str(t) for t in sorted(wanted))
    last_speed = 0            # 上一次发送的速度（0 表示停止）
    fixes = 0                 # 已用掉的"见标修正"次数
    last_time = time.perf_counter()
    while True:
        now = time.perf_counter()
        frame, tags = vision.read_tags(camera, detector)
        if frame is None:
            continue
        dt = now - last_time   # 距上一帧的时间（秒）
        last_time = now

        center_x = frame.shape[1] // 2                          # 画面中心 X
        center_y = frame.shape[0] // 2                          # 画面中心 Y
        target = next((t for t in tags if t.tag_id in wanted), None)  # 找目标标签

        if target is None:
            # —— 见标修正：搜索中看到其他已登记标签——先把它对准画面中心
            # （此刻当前朝向 = 它的地图角度），再用相对角度差快速转向目标 ——
            if nav_target is not None and fixes < MAX_HEADING_FIXES:
                seen = next((t for t in tags
                             if map.angle_of(t.tag_id) is not None), None)
                if seen is not None and center_tag(camera, detector,
                                                   seen.tag_id):
                    # 标签已居中：当前朝向即该标签的地图角度，
                    # 据此算出到目标方位的差值并快速转向
                    if heading is not None:
                        heading = map.angle_of(seen.tag_id)
                    turn = map.shortest_turn(map.angle_of(seen.tag_id),
                                             nav_target)
                    if abs(turn) > TURN_TOLERANCE:
                        fixes += 1
                        last_speed = 0
                        print(f"已对准 {seen.tag_id} 号位，"
                              f"按地图快速转向 {turn:.0f}°")
                        turn_by(turn)
                        if heading is not None:
                            heading = nav_target   # 转向后朝向按地图值记
                        last_time = time.perf_counter()
                        continue
                    # 转向差在容差内（例如 ①②③ 同为 0°）：不转向，
                    # 落入下面的正常搜索流程，避免原地死循环
                else:
                    last_time = time.perf_counter()   # 对准失败也重置计时

            speed = SEARCH_SPEED   # 没发现目标标签，云台转动搜索
            if heading is not None:
                heading += DEGREES_PER_SECOND * dt * SPIN_MAP_DIRECTION
            status = f"标签 {label}：未发现，搜索中"
        else:
            # —— X 轴：云台电机闭环 ——
            # 偏差 = 测量值（标签位置）− 期望值（画面中心 + 任务指定的 offset）
            error = int(target.center[0] - center_x - offset)
            speed = motor_speed(error)

            # 垂直偏差仅作参考显示：摄像头俯仰固定，画面垂直位置不可控
            error_y = int(target.center[1] - center_y)

            status = f"标签 {target.tag_id}：X 偏差 {error}，Y 偏差 {error_y}"

            if abs(error) < CENTER_HALF_WIDTH:
                print(f"{status} |=> 锁定目标")
                board.set_motor_b(0) # 云台停止
                if len(wanted) == 1:   # 对零阶段朝向按定义即基准角度，不回写
                    record_measured_angle(target.tag_id, heading)
                return target.tag_id, heading

        # 只在速度变化时发送命令
        if speed != last_speed:
            board.set_motor_b(speed)
            last_speed = speed
            status = f"{status} |=> 发送指令"
            print(status)

        vision.show_frame(frame, tags, status)
        if vision.is_quit_key(vision.wait_key()):
            print("按 Q 或 Esc 退出。")
            return None, heading


def main():
    detector = vision.create_detector() # 创建标签检测器
    camera = vision.open_camera() # 打开摄像头

    try:
        board.connect(SERIAL_PORT) # 连接控制板
        board.set_servo_angle(90)  # 云台初始角度
        board.stop_motor_b()   # 一开始云台不动
        print("云台已就绪。")

        # —— 第一步：对零 —— 找到基准标签（①②③）并锁定，锁定方向即地图 0°
        print(f"\n=== 对零：搜索基准标签 {map.REFERENCE_TAGS} ===")
        board.set_servo_angle(ALIGN_PITCH)  # 舵机先转到基准标签所在高度
        time.sleep(0.5)   # 等舵机到位
        locked, heading = track_tag(camera, detector, map.REFERENCE_TAGS, 0)
        if locked is None:   # 用户按 Q/Esc 退出
            return
        heading = map.angle_of(locked)   # 对零后的朝向（0°）
        print(f"\n对零完成：锁定 {locked} 号位，当前朝向 {heading}°")

        # —— 第二步：按地图依次执行任务 ——
        for i, task in enumerate(TASKS, 1):
            print(f"\n=== 任务 {i}/{len(TASKS)}：标签 {task['tag']}，"
                  f"武器俯仰 {task['angle']}，动作 {task['action']} ===")
            board.set_servo_angle(task["angle"])  # 武器俯仰粗瞄准到指定角度
            time.sleep(0.5)   # 等舵机到位，避免画面还在晃就开始跟踪

            # 按地图相对角度转向目标大概方位（开环定时转动）
            target_angle = map.angle_of(task["tag"])
            if heading is not None and target_angle is not None:
                turn = map.shortest_turn(heading, target_angle)   # 最短转向
                if abs(turn) > TURN_TOLERANCE:
                    print(f"    按地图转向：{heading:.0f}° → {target_angle}°，转动 {turn}°")
                    turn_by(turn)
                heading = target_angle   # 转完后的朝向按地图值记
            else:
                print("    该标签未登记地图角度，跳过转向直接搜索")

            locked, heading = track_tag(camera, detector, task["tag"],
                                        task["offset"], heading,
                                        target_angle)  # 水平跟踪标签直到到位
            if locked is None:   # 用户按 Q/Esc 退出
                break
            # 锁定时朝向估计即该标签实测方向（回写在 track_tag 内完成）
            heading = map.angle_of(locked)   # 按地图值（含实测回写）校准朝向
            execute_action(task["action"])  # 标签居中后执行动作
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
