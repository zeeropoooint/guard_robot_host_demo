"""标定工具：测定 track_fire.py 中 TASKS 所需的 offset（期望位置）和 angle（俯仰角度）。

使用流程：
    1. 启动后默认标定 TASKS 里的第一个标签（按 - / = 切换），
       工具自动按该任务现有的 angle 调整俯仰，再把目标转到期望位置
    2. 居中后进入手动模式，用键盘微调云台左右、发射器俯仰，
       让激光点（或发射方向）正好对准目标
    3. 按空格记录：终端打印一行字典（保留该任务原有的 action / search 等字段），
       直接替换进 track_fire.py 里 TASKS 对应的那一行即可

按键（在摄像头窗口上操作）：
    A / D   云台左转 / 右转（点动微调）
    W / S   俯仰角增大 / 减小（发射器上下）
    - / =   目标标签编号减一 / 加一（自动载入该任务的 angle 和瞄准方式）
    C       切换瞄准点：标签中心 / 标签附近的圆心（对应任务的 aim_circle）
    R       重新自动居中（未发现目标时强制转动搜索）
    L       测试激光 3 秒
    F       测试发射一次
    空格    记录当前 offset / angle 并打印任务行
    Q / Esc 退出

与主程序保持一致：
    - 回中方向、速度、居中阈值直接取自 track_fire.py；
    - 任务写了 "search": False 时，找到过一次后丢失标签不再转动，原地等待；
    - 任务写了 "aim_circle": True 时默认瞄准标签附近的圆心，offset 记录圆心位置；
    - 画面左上角实时显示 A 键锁停状态（固件上报 @LK1）。

画面左上角实时显示：模式、标签编号、俯仰角度、标签实时偏移；
记录后画面中会出现一条黄色竖线，标记该标签已保存的期望位置。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # 项目根目录加入模块搜索路径

import time

import cv2

from board import board
from vision import vision
from config import SERIAL_PORT, VOLTAGE_POLARITY
from track_fire import (TASKS, CENTER_DIRECTION, CENTER_HALF_WIDTH,
                        LATER_MOTOR_SPEED, MOTOR_SPEED, PITCH_START,
                        motor_speed, status_with_lock)
from circle_aim import AIM_CIRCLE_NEAR_TAG, pick_circle_near

# ============================== 参数 ==============================
PAN_NUDGE_SPEED = 200    # 手动模式云台点动速度
PAN_NUDGE_MS = 150       # 手动模式每次点动时长（毫秒）：太短会转不动，太长不够精细

SERVO_STEP = 1           # 每按一次 W/S 角度变化量
SERVO_MIN, SERVO_MAX = 0, 180

# 任务预设：切换标签时直接沿用 track_fire.py 里该任务的 angle / aim_circle
TASK_BY_TAG = {task["tag"]: task for task in TASKS if "tag" in task}
TAG_IDS = list(TASK_BY_TAG)
TAG_MIN = min(TAG_IDS) if TAG_IDS else 1
TAG_MAX = max(TAG_IDS) if TAG_IDS else 10


def task_angle(tag_id):
    """该标签在 TASKS 里的俯仰角；任务列表里没有时用 PITCH_START。

    输入：tag_id 标签编号
    输出：俯仰角度（度）
    """

    return TASK_BY_TAG.get(tag_id, {}).get("angle", PITCH_START)


def task_aims_circle(tag_id):
    """该任务是否瞄准标签附近的圆心。

    输入：tag_id 标签编号
    输出：bool；任务没写 aim_circle 时跟随主程序的总开关
    """

    return bool(TASK_BY_TAG.get(tag_id, {}).get("aim_circle", AIM_CIRCLE_NEAR_TAG))


def task_search_enabled(tag_id):
    """该任务是否允许未发现标签时转动搜索。

    输入：tag_id 标签编号
    输出：bool；缺省为 True
    """

    return bool(TASK_BY_TAG.get(tag_id, {}).get("search", True))


def task_speed(tag_id):
    """该任务在主程序里使用的云台速度（前三个任务更快）。

    输入：tag_id 标签编号
    输出：电机速度幅值
    """

    for index, task in enumerate(TASKS):
        if task.get("tag") == tag_id:
            return MOTOR_SPEED if index < 3 else LATER_MOTOR_SPEED
    return MOTOR_SPEED


def nudge_pan(direction):
    """云台点动一下，并打印反馈（确认按键已收到）。

    转动方向与主程序回中方向保持同一套符号：
    direction=+1 云台右转，-1 左转。
    输入：direction +1 右转，-1 左转
    输出：无
    """

    speed = -direction * PAN_NUDGE_SPEED * VOLTAGE_POLARITY * CENTER_DIRECTION
    board.run_motor_b_for(speed, PAN_NUDGE_MS)
    print(f"云台向{'右' if direction > 0 else '左'}点动 {PAN_NUDGE_MS}ms")


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


def format_value(value):
    """按 Python 字面量格式输出字典值（字符串用双引号，布尔用 True/False）。

    输入：value 字典值
    输出：可直接粘贴进 TASKS 的字符串
    """

    if isinstance(value, str):
        return f'"{value}"'
    if isinstance(value, bool):
        return "True" if value else "False"
    return repr(value)


def format_task(tag_id, offset, angle, aim_circle):
    """把标定结果格式化成一行 TASKS 字典，保留任务原有字段。

    输入：tag_id 标签编号；offset 期望位置；angle 俯仰角；
         aim_circle True 表示记录的是圆心位置，需要任务打开 aim_circle
    输出：一行字典字符串（不含缩进和结尾逗号）
    """

    task = dict(TASK_BY_TAG.get(tag_id, {"tag": tag_id, "action": "laser"}))
    task["tag"] = tag_id
    task["offset"] = offset
    task["angle"] = angle
    task.setdefault("action", "laser")
    if aim_circle:
        task["aim_circle"] = True
    else:
        task.pop("aim_circle", None)   # 标定的是标签中心，不再自动找圆

    order = ("tag", "offset", "angle", "action", "aim_circle",
             "search", "hold", "circle")
    keys = [key for key in order if key in task]
    keys += [key for key in task if key not in order]
    pairs = [f'"{key}": {format_value(task[key])}' for key in keys]
    return "{" + ", ".join(pairs) + "}"


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

        tag_id = TAG_MIN                 # 默认标定 TASKS 的第一个标签
        angle = task_angle(tag_id)       # 直接沿用任务里的俯仰角，保证标签在画面里
        aim_circle = task_aims_circle(tag_id)
        board.set_servo_angle(angle)

        mode = "AUTO"          # AUTO 自动居中 / MANUAL 手动微调
        last_speed = 0         # 自动模式下上一次发送的速度
        last_aim = None        # 上一次对准的圆 (x, y, r)，标签丢失时按它找回圆
        seen_once = False      # 当前标签是否已经找到过（search=False 的判断依据）
        force_search = False   # R 键强制搜索：忽略任务里的 search=False
        saved = {}             # tag_id -> (offset, angle)，每个标签各记一条
        print(f"云台就绪：自动搜索 {tag_id} 号标签中（angle={angle}，"
              f"{'找圆心' if aim_circle else '对准标签'}）。"
              "按 - / = 切换标签，R 强制搜索，空格记录。")

        def select_tag(new_tag_id):
            """切换目标标签：载入 TASKS 里的 angle / aim_circle 并重新自动居中。"""

            nonlocal tag_id, angle, aim_circle, mode, last_speed
            nonlocal last_aim, seen_once, force_search
            tag_id = new_tag_id
            angle = task_angle(tag_id)
            aim_circle = task_aims_circle(tag_id)
            board.set_servo_angle(angle)
            mode = "AUTO"
            last_speed = None      # 强制下一帧重新发送速度指令
            last_aim = None
            seen_once = False
            force_search = False
            print(f"目标标签切换为 {tag_id}（angle={angle}，"
                  f"{'找圆心' if aim_circle else '对准标签'}）")

        while True:
            frame, tags = vision.read_tags(camera, detector)
            if frame is None:
                continue

            center_x = frame.shape[1] // 2
            target = next((t for t in tags if t.tag_id == tag_id), None)

            # 瞄准点：默认标签中心；aim_circle 时优先用标签附近的圆心
            circles = None
            aim = None
            if aim_circle:
                circles = vision.detect_circles(frame)
                aim = pick_circle_near(circles, target, last_aim)
                if aim is not None:
                    last_aim = aim
            ref = aim if aim is not None else (
                target.center if target is not None else None)
            live_offset = int(ref[0] - center_x) if ref is not None else None
            if ref is not None:
                seen_once = True

            speed_limit = task_speed(tag_id)

            # ---------- 自动模式：转动云台把目标送到期望位置 ----------
            if mode == "AUTO":
                if ref is None:
                    # 与主程序一致：search=False 时不转；但第一次找标签、
                    # 或按 R 强制搜索时仍然转动，方便标定
                    can_search = force_search or not seen_once \
                        or task_search_enabled(tag_id)
                    speed = speed_limit * VOLTAGE_POLARITY if can_search else 0
                    searching = can_search
                else:
                    speed = motor_speed(live_offset, speed_limit)
                    searching = False
                    if abs(live_offset) < CENTER_HALF_WIDTH:
                        board.set_motor_b(0)
                        last_speed = 0
                        mode = "MANUAL"
                        print(f"标签 {tag_id} 已居中，进入手动微调模式。")
                if mode == "AUTO" and speed != last_speed:
                    board.set_motor_b(speed)
                    last_speed = speed

            # ---------- 画面显示：模式、角度、偏移、已记录位置 ----------
            if tag_id in saved:
                saved_x = center_x + saved[tag_id][0]
                cv2.line(  # 黄色竖线：该标签已记录的期望位置
                    frame, (saved_x, 0), (saved_x, frame.shape[0]), (0, 255, 255), 1
                )

            offset_text = str(live_offset) if live_offset is not None else "---"
            saved_text = (f"saved off:{saved[tag_id][0]} ang:{saved[tag_id][1]}"
                          if tag_id in saved else "")
            target_text = "圆心" if aim is not None else "标签"
            search_text = "" if mode != "AUTO" else (
                "搜索中" if searching else "已停止")
            status = (f"[{mode}] tag:{tag_id} angle:{angle} {target_text} "
                      f"offset:{offset_text} {search_text} {saved_text}")
            vision.show_frame(frame, tags, status_with_lock(status), circles)

            # ---------- 键盘控制 ----------
            key = vision.wait_key(10)
            if board.take_unlock_event():
                last_speed = None   # 锁定时固件停转过电机，解锁后重新发送速度
            if vision.is_quit_key(key):
                print("按 Q 或 Esc 退出。")
                break
            elif key in (ord("a"), ord("A")):   # 云台左转
                mode = "MANUAL"
                nudge_pan(-1)
                last_speed = None   # 点动是限时指令，会顶掉持续转动；强制重新发指令
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
                select_tag(max(TAG_MIN, tag_id - 1))
            elif key == ord("="):               # 标签编号加一
                select_tag(min(TAG_MAX, tag_id + 1))
            elif key in (ord("c"), ord("C")):   # 切换瞄准点：标签 / 附近圆心
                aim_circle = not aim_circle
                last_aim = None
                mode = "AUTO"
                last_speed = None
                print(f"瞄准点切换为：{'标签附近的圆心' if aim_circle else '标签中心'}")
            elif key in (ord("r"), ord("R")):   # 重新自动居中
                mode = "AUTO"
                last_speed = None
                force_search = True   # 用户主动要求，忽略任务里的 search=False
                seen_once = False
                print(f"重新自动居中：搜索标签 {tag_id}。")
            elif key in (ord("l"), ord("L")):   # 测试激光
                board.laser_on_for(3)
            elif key in (ord("f"), ord("F")):   # 测试发射
                board.fire_once()
                time.sleep(3)
            elif key == ord(" "):               # 记录 offset / angle
                if live_offset is None:
                    print("当前画面没有目标，无法记录。")
                elif aim_circle and aim is None:
                    print("当前标签附近没有检测到圆，无法记录圆心位置。")
                else:
                    saved[tag_id] = (live_offset, angle)
                    print(f"\n标签 {tag_id} 记录成功！复制下面这行替换 "
                          f"track_fire.py 的 TASKS 对应行：")
                    print(f"    {format_task(tag_id, live_offset, angle, aim_circle)},\n")
    finally:
        if board.is_connected():
            board.stop_motor_b()
            board.disconnect()
        camera.release()
        vision.close_windows()


if __name__ == "__main__":
    main()
