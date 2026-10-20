"""按任务顺序执行：调整舵机俯仰 → 跟踪指定标签 → 执行动作（激光 / 发射）

任务定义（TASKS）：
    每个任务对应一个特定编号的 AprilTag，包含：
        tag     ：要跟踪的标签编号
        offset  ：标签期望位置，相对画面中心的像素偏移（右正左负，0 即画面中心）
        angle   ：跟踪该标签时舵机（俯仰）角度，0～180
        action  ：标签到位后执行的动作，"laser" 发射激光 或 "fire" 发射一次
         hold    ：可选，动作执行完成后停留的秒数（云台保持不动），缺省为不停留

操作：在摄像头窗口按 Q 或 Esc 退出。
"""

import time

from board import board
from vision import vision
from config import SERIAL_PORT, VOLTAGE_POLARITY, resolve_serial_port

MOTOR_SPEED = 180        # 云台转速度
SEARCH_SPEED = MOTOR_SPEED * VOLTAGE_POLARITY   # 未发现目标标签时，向一个方向旋转搜索
CENTER_HALF_WIDTH = 10   # 标签离画面中心多少像素以内算"居中"

# 任务列表：按顺序逐个执行
TASKS = [
    # tag：标签编号；angle：舵机俯仰角度；
    # action：动作（laser 激光 / fire 发射）；offset：标签期望位置（右正左负，0 即画面中心）
    {"tag": 1, "offset": -20, "angle": 102, "action": "fire", "hold": 6 },
    {"tag": 2, "offset": -20, "angle": 102, "action": "fire", "hold": 6,},
    {"tag": 3, "offset": -20, "angle": 102, "action": "fire"},
    {"tag": 4, "offset": -54, "angle": 106, "action": "laser"},
    {"tag": 5, "offset": -19, "angle": 102, "action": "fire"},
    {"tag": 6, "offset": -38, "angle": 113, "action": "laser"},
    {"tag": 7, "offset": -20, "angle": 100, "action": "fire"},
    {"tag": 8, "offset": -20, "angle": 100, "action": "fire"},
    {"tag": 9, "offset": -75, "angle": 116, "action": "laser"},
    {"tag": 10, "offset": -20, "angle": 101, "action": "fire"},
]



def motor_speed(error):
    """根据偏差算出云台速度。

    输入：error 偏差
    输出：速度；偏差在居中范围内时返回 0。
    """

    if error > CENTER_HALF_WIDTH:
        return -MOTOR_SPEED * VOLTAGE_POLARITY   # 标签偏右，右转
    if error < -CENTER_HALF_WIDTH:
        return MOTOR_SPEED * VOLTAGE_POLARITY    # 标签偏左，左转
    return 0                                     # 标签居中


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


def track_tag(camera, detector, tag_id, offset):
    """跟踪指定 id 的标签直到期望位置。

    输入：camera/detector 视觉对象；tag_id 目标标签 id；
         offset 期望的标签水平位置（相对画面中心的像素偏移，画面中心为 0）
    输出：True 已到位；False 用户按 Q/Esc 退出。
    """

    last_speed = 0   # 上一次发送的速度（0 表示停止）
    while True:
        frame, tags = vision.read_tags(camera, detector)
        if frame is None:
            continue

        center_x = frame.shape[1] // 2                          # 画面中心
        target = next((t for t in tags if t.tag_id == tag_id), None)  # 找目标标签

        if target is None:
            speed = SEARCH_SPEED   # 没发现目标标签，云台转动搜索
            status = f"标签 {tag_id}：未发现，搜索中"
        else:
            # 偏差 = 测量值（标签位置）− 期望值（画面中心 + 任务指定的 offset）
            error = int(target.center[0] - center_x - offset)
            speed = motor_speed(error)
            status = f"标签 {tag_id}：偏差 {error}"

            if abs(error) <= CENTER_HALF_WIDTH:   # 偏差在阈值内，认为已居中
                print(f"{status} |=> 锁定目标")
                board.set_motor_b(0) # 云台停止
                return True

        # 只在速度变化时发送命令
        if speed != last_speed:
            board.set_motor_b(speed)
            last_speed = speed
            status = f"{status} |=> 发送指令"
            print(status)

        vision.show_frame(frame, tags, status)
        if vision.is_quit_key(vision.wait_key()):
            print("按 Q 或 Esc 退出。")
            return False


def main():
    detector = vision.create_detector() # 创建标签检测器
    camera = vision.open_camera() # 打开摄像头

    try:
        board.connect(resolve_serial_port(SERIAL_PORT)) # 连接控制板
        board.set_servo_angle(90)  # 云台初始角度
        board.stop_motor_b()   # 一开始云台不动
        print("云台已就绪，开始按顺序执行任务。")

        for i, task in enumerate(TASKS, 1):
            print(f"\n=== 任务 {i}/{len(TASKS)}：标签 {task['tag']}，"
                  f"舵机角度 {task['angle']}，动作 {task['action']} ===")
            board.set_servo_angle(task["angle"])  # 调整装置俯仰角度
            if not track_tag(camera, detector, task["tag"], task["offset"]):  # 跟踪标签直到到位
                break
            execute_action(task["action"])  # 标签居中后执行动作
            hold = task.get("hold", 0)      # 可选：动作后停留，云台保持不动
            if hold:
                print(f"    停留 {hold} 秒……")
                time.sleep(hold)
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
