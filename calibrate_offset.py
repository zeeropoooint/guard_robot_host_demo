"""标定工具：测定 track_fire.py 中 TASKS 所需的 offset（期望位置）和 angle（俯仰角度）。

使用流程：
    1. 启动后默认标定 2 号标签（按 - / = 切换），机器人自动搜索并把标签转到画面中心
    2. 居中后进入手动模式，用键盘微调云台左右、发射器俯仰，
       让激光点（或发射方向）正好对准目标
    3. 按空格记录：终端打印一行字典，直接复制进 track_fire.py 的 TASKS 即可

按键（在摄像头窗口上操作）：
    A / D   云台左转 / 右转（点动微调）
    W / S   俯仰角增大 / 减小（发射器上下）
    - / =   目标标签编号减一 / 加一
    R       重新自动居中
    L       测试激光 3 秒
    F       测试发射一次
    空格    记录当前 offset / angle 并打印任务行
    Q / Esc 退出

画面左上角实时显示：模式、标签编号、俯仰角度、标签实时偏移；
记录后画面中会出现一条黄色竖线，标记已保存的期望位置。
"""

import time

import cv2

from board import board
from vision import vision
from config import SERIAL_PORT, VOLTAGE_POLARITY

# ============================== 参数 ==============================
MOTOR_SPEED = 180        # 自动模式云台转速
SEARCH_SPEED = -MOTOR_SPEED * VOLTAGE_POLARITY  # 未发现标签时的搜索方向（取反=与原方向相反）
CENTER_HALF_WIDTH = 10   # 标签离画面中心多少像素以内算"居中"

PAN_NUDGE_SPEED = 200    # 手动模式云台点动速度
PAN_NUDGE_MS = 150       # 手动模式每次点动时长（毫秒）：太短会转不动，太长不够精细

SERVO_START = 90         # 启动时的舵机初始角度
SERVO_STEP = 1           # 每按一次 W/S 角度变化量
SERVO_MIN, SERVO_MAX = 0, 180


def motor_speed(error):
    """根据偏差算云台速度（与 track_fire.py 一致）。

    输入：error 偏差
    输出：速度；偏差在居中范围内时返回 0。
    """

    if error > CENTER_HALF_WIDTH:
        return MOTOR_SPEED * VOLTAGE_POLARITY    # 标签偏右
    if error < -CENTER_HALF_WIDTH:
        return -MOTOR_SPEED * VOLTAGE_POLARITY   # 标签偏左
    return 0                                     # 标签居中


def nudge_pan(direction):
    """云台点动一下，并打印反馈（确认按键已收到）。

    输入：direction +1 左转，-1 右转
    输出：无
    """

    board.run_motor_b_for(direction * PAN_NUDGE_SPEED * VOLTAGE_POLARITY, PAN_NUDGE_MS)
    print(f"云台向{'左' if direction > 0 else '右'}点动 {PAN_NUDGE_MS}ms")


def nudge_servo(angle, delta):
    """舵机角度微调并限制在 0～180。

    输入：angle 当前角度；delta 变化量
    输出：调整后的新角度
    """

    new_angle = max(SERVO_MIN, min(SERVO_MAX, angle + delta))
    if new_angle != angle:
        board.set_servo_angle(new_angle)
        print(f"舵机：{new_angle} 度")
    return new_angle


def main():
    detector = vision.create_detector()
    camera = vision.open_camera()

    try:
        board.connect(SERIAL_PORT)
    except board.BoardConnectionError as exc:
        camera.release()
        raise SystemExit(  # 打印清晰提示后退出，不再抛一长串 traceback
            f"无法连接控制板：{exc}\n"
            "排查：1) 确认 USB 线是数据线（能传数据，不是仅充电线）且已插好；\n"
            "      2) 运行 ls /dev/cu.usb* 查看实际串口号；\n"
            "      3) 把看到的串口号填入 config.py 的 SERIAL_PORT。"
        )

    try:
        board.stop_motor_b()
        angle = SERVO_START
        board.set_servo_angle(angle)

        tag_id = 2   # 默认标定 2 号标签，窗口中按 - / = 切换

        mode = "AUTO"          # AUTO 自动居中 / MANUAL 手动微调
        last_speed = 0         # 自动模式下上一次发送的速度
        saved = None           # 已记录的 (offset, angle)
        print("云台就绪：自动搜索 2 号标签中（按 - / = 切换标签）。"
              "居中后可按 A/D/W/S 微调，空格记录。")

        while True:
            frame, tags = vision.read_tags(camera, detector)
            if frame is None:
                continue

            center_x = frame.shape[1] // 2
            target = next((t for t in tags if t.tag_id == tag_id), None)
            live_offset = int(target.center[0] - center_x) if target else None

            # ---------- 自动模式：转动云台把标签送到画面中心 ----------
            if mode == "AUTO":
                if target is None:
                    speed = SEARCH_SPEED
                else:
                    speed = motor_speed(live_offset)
                    if abs(live_offset) < CENTER_HALF_WIDTH:
                        board.set_motor_b(0)
                        last_speed = 0
                        mode = "MANUAL"
                        print(f"标签 {tag_id} 已居中，进入手动微调模式。")
                if mode == "AUTO" and speed != last_speed:
                    board.set_motor_b(speed)
                    last_speed = speed

            # ---------- 画面显示：模式、角度、偏移、已记录位置 ----------
            if saved is not None:
                saved_x = center_x + saved[0]
                cv2.line(  # 黄色竖线：已记录的期望位置
                    frame, (saved_x, 0), (saved_x, frame.shape[0]), (0, 255, 255), 1
                )

            offset_text = str(live_offset) if live_offset is not None else "---"
            saved_text = f"saved off:{saved[0]} ang:{saved[1]}" if saved else ""
            status = (f"[{mode}] tag:{tag_id} angle:{angle} "
                      f"offset:{offset_text} {saved_text}")
            vision.show_frame(frame, tags, status)

            # ---------- 键盘控制 ----------
            key = vision.wait_key(10)
            if vision.is_quit_key(key):
                print("按 Q 或 Esc 退出。")
                break
            elif key in (ord("a"), ord("A")):   # 云台左转
                mode = "MANUAL"
                nudge_pan(-1)
                last_speed = None   # 点动是限时指令，会顶掉持续转动；强制自动模式下次重新发指令
            elif key in (ord("d"), ord("D")):   # 云台右转
                mode = "MANUAL"
                nudge_pan(+1)
                last_speed = None
            elif key in (ord("w"), ord("W")):   # 发射器上移
                mode = "MANUAL"
                angle = nudge_servo(angle, -SERVO_STEP)
            elif key in (ord("s"), ord("S")):   # 发射器下移
                mode = "MANUAL"
                angle = nudge_servo(angle, +SERVO_STEP)
            elif key == ord("-"):               # 标签编号减一
                tag_id = max(0, tag_id - 1)
                print(f"目标标签切换为 {tag_id}")
            elif key == ord("="):               # 标签编号加一
                tag_id += 1
                print(f"目标标签切换为 {tag_id}")
            elif key in (ord("r"), ord("R")):   # 重新自动居中
                mode = "AUTO"
                last_speed = None   # 强制下一帧重新发送速度指令
                print(f"重新自动居中：搜索标签 {tag_id}。")
            elif key in (ord("l"), ord("L")):   # 测试激光
                board.laser_on_for(3)
            elif key in (ord("f"), ord("F")):   # 测试发射
                board.fire_once()
                time.sleep(3)
            elif key == ord(" "):               # 记录 offset / angle
                if live_offset is None:
                    print("当前画面没有目标标签，无法记录。")
                else:
                    saved = (live_offset, angle)
                    print(f"\n标签 {tag_id} 记录成功！复制下面这行到 "
                          f"track_fire.py 的 TASKS 中：")
                    print(f'    {{"tag": {tag_id}, "offset": {live_offset}, '
                          f'"angle": {angle}, "action": "laser"}},\n')
    finally:
        if board.is_connected():
            board.stop_motor_b()
            board.disconnect()
        camera.release()
        vision.close_windows()


if __name__ == "__main__":
    main()
