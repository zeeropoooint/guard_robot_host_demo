"""测试工具：转一圈测量二维码之间的相对方位和距离，生成以 4 号标签为 0° 的地图数据。

做三件事：
    1. 云台连续转整圈（默认两圈），每帧记录"当前朝向 + 画面像素偏移"，
       得到每个标签相对扫描起点的方位；同一标签转圈中会被录到很多次，
       取最靠近画面中心的那批样本求平均（边缘畸变最小）。
    2. 同一批样本里记录标签的像素边长，按"距离与像素边长成反比"估算远近。
       不知道标签实际打印边长时，用旧地图的 distance_cm 反推统一刻度 K
       （距离 = K / 像素边长），测出的距离与旧值同一量纲，相对远近是实测的。
    3. 以 4 号标签为 0° 重定基准，打印可直接替换进 field_map.py 的
       TARGETS 代码，并列出与旧地图（同样相对 4 号）的差值，用来核对场地。

为什么不用 1/2/3 号做基准：这三个标签贴在同一个方向，方位完全重合，
既区分不出彼此也定不准朝向，所以校准时也不使用它们
（见 field_map.CALIBRATION_TAGS）。

按键（摄像头窗口上操作）：
    W / S   扫描前调整俯仰角（抬高 / 降低）
    Y       开始扫描
    空格    扫描中提前结束，用已采到的样本生成结果
    Q / Esc 退出

运行：uv run python tools/test_tag_relative.py
"""

import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # 项目根目录加入模块搜索路径

from board import board
from vision import vision
from config import SERIAL_PORT, GIMBAL_SPEED_REF, GIMBAL_DEG_PER_SEC
from field_map import TARGETS as OLD_TARGETS, TAG_ANGLE, shortest_delta
from track_fire import px_to_deg

# ============================== 参数 ==============================
SCAN_SPEED = 150        # 扫描时云台速度：太快画面拖影识别不到，太慢一圈太久
SCAN_PITCH = 100        # 扫描俯仰角（任务里的 angle 在 92～111 之间）
SCAN_LOOPS = 2          # 转几圈：两圈可以顺便看出角度漂移
REF_TAG = 4             # 生成地图时的 0° 基准标签

PITCH_STEP = 2          # 每按一次 W/S 升降的度数
PITCH_MIN, PITCH_MAX = 60, 140

_pan_speed = 0          # 当前云台速度指令（含符号，正 = 顺时针）
_pan_angle = 0.0        # 从扫描起点算起的累计转角（度）
_pan_time = 0.0         # 上次结算时刻


# ============================== 转角记账 ==============================
def _integrate():
    """把"上次结算到现在"的转动量累加进累计转角。"""

    global _pan_angle, _pan_time
    now = time.monotonic()
    _pan_angle += (_pan_speed / GIMBAL_SPEED_REF * GIMBAL_DEG_PER_SEC
                   * (now - _pan_time))
    _pan_time = now


def pan(speed):
    """设置云台速度，同时把转动量记进累计转角。"""

    global _pan_speed
    _integrate()
    _pan_speed = int(speed)
    if _pan_speed:
        board.set_motor_b(_pan_speed)
    else:
        board.stop_motor_b()


def scan_angle():
    """当前累计转角（度，从扫描起点算起，顺时针为正）。"""

    _integrate()
    return _pan_angle


# ============================== 扫描 ==============================
def wait_start(camera, detector, pitch):
    """预览画面，等用户确认场地状态。

    输入：camera / detector；pitch 当前俯仰角
    输出：(是否开始扫描, 调整后的俯仰角)
    """

    print("准备扫描：W/S 调俯仰，Y 开始，Q 退出。")
    while True:
        frame, tags = vision.read_tags(camera, detector)
        if frame is None:
            continue
        ids = " ".join(str(tag.tag_id) for tag in tags) or "-"
        vision.show_frame(frame, tags,
                          f"俯仰 {pitch}°  可见 {ids}  W/S 调俯仰，Y 开始")
        key = vision.wait_key(10)
        if key in (ord("w"), ord("W")):
            pitch = max(PITCH_MIN, pitch - PITCH_STEP)
            board.set_servo_angle(pitch)
        elif key in (ord("s"), ord("S")):
            pitch = min(PITCH_MAX, pitch + PITCH_STEP)
            board.set_servo_angle(pitch)
        elif key in (ord("y"), ord("Y")):
            return True, pitch
        elif vision.is_quit_key(key):
            return False, pitch


def scan(camera, detector):
    """转满 SCAN_LOOPS 圈，采集每个标签的方位和像素边长样本。

    输入：camera / detector
    输出：{标签号: [(离画面中心的距离, 方位角, 圈号, 像素边长), ...]}；
          A 键锁定导致扫描中断时返回 None
    """

    global _pan_angle, _pan_time

    _pan_angle = 0.0
    _pan_time = time.monotonic()
    samples = {}
    total = 360.0 * SCAN_LOOPS

    pan(SCAN_SPEED)
    try:
        while True:
            before = scan_angle()
            if before >= total:
                break
            frame, tags = vision.read_tags(camera, detector)
            after = scan_angle()          # 取读帧前后的中点，抵消识别耗时
            if frame is None:
                continue

            width = frame.shape[1]
            heading = (before + after) / 2.0
            loop = int(before // 360) + 1
            for tag in tags:
                dx = tag.center[0] - width / 2.0
                angle = (heading + px_to_deg(dx, width)) % 360
                samples.setdefault(tag.tag_id, []).append(
                    (abs(dx), angle, loop, tag_side_px(tag)))

            ids = " ".join(str(tag.tag_id) for tag in tags) or "-"
            vision.show_frame(
                frame, tags,
                f"扫描 {before:.0f}/{total:.0f}°  圈 {loop}/{SCAN_LOOPS}  可见 {ids}")

            key = vision.wait_key(1)
            if vision.is_quit_key(key) or key == ord(" "):
                break
            if board.motor_b_locked():
                print("A 键锁定，云台被停转，本次扫描作废。")
                return None
    finally:
        pan(0)
    return samples


# ============================== 数据处理 ==============================
def tag_side_px(tag):
    """标签四条边的平均像素长度（不受画面内旋转影响）。

    输入：tag 检测到的标签
    输出：平均边长（像素）
    """

    corners = tag.corners
    return sum(math.dist(corners[i], corners[(i + 1) % 4])
               for i in range(4)) / 4.0


def near_center_items(items):
    """挑出最靠近画面中心的一部分样本（边缘畸变最小）。

    输入：items [(离中心像素距离, 方位角, 圈号, 像素边长), ...]
    输出：筛选后的样本列表
    """

    near = sorted(items, key=lambda item: item[0])
    return near[:max(3, len(near) // 5)]


def near_center_mean(items):
    """取最靠近画面中心的一部分样本求圆周平均（避开 0/360 跨界）。

    输入：items [(离中心像素距离, 方位角, 圈号, 像素边长), ...]
    输出：平均方位角（度）
    """

    values = [item[1] for item in near_center_items(items)]
    sin_sum = sum(math.sin(math.radians(value)) for value in values)
    cos_sum = sum(math.cos(math.radians(value)) for value in values)
    return math.degrees(math.atan2(sin_sum, cos_sum)) % 360


def loop_drift(samples):
    """两圈之间同一标签的方位差，用来判断转速标定是否偏了。"""

    diffs = []
    for items in samples.values():
        by_loop = {}
        for item in items:
            by_loop.setdefault(item[2], []).append(item)
        if 1 in by_loop and 2 in by_loop:
            diffs.append(shortest_delta(near_center_mean(by_loop[1]),
                                        near_center_mean(by_loop[2])))
    return sum(diffs) / len(diffs) if diffs else None


def measure_distances(samples):
    """用旧地图距离标定像素→厘米刻度，再换算每个标签的距离。

    标签像素边长 side 与距离 d 近似满足 d = K / side（K 由焦距和标签实际
    边长决定）。不知道打印边长时，用旧地图距离反推统一刻度：
    K = median(旧距离 × 本次像素边长)，再用同一个 K 算所有标签。
    输入：scan() 采集到的样本
    输出：({标签号: 距离cm}, K)；样本不足以标定时返回 ({}, None)
    """

    side_by_tag = {}
    scales = []
    for tag_id, items in samples.items():
        side = statistics.median(item[3] for item in near_center_items(items))
        if side <= 0:
            continue
        side_by_tag[tag_id] = side
        old_distance = OLD_TARGETS.get(tag_id, {}).get("distance_cm", 0)
        if old_distance:
            scales.append(old_distance * side)
    if not scales:
        return {}, None

    scale = statistics.median(scales)
    return ({tag_id: scale / side for tag_id, side in side_by_tag.items()},
            scale)


def report(samples):
    """按 4 号标签重定基准并打印结果与可直接粘贴的地图数据。

    输入：scan() 采集到的样本
    输出：{标签号: 相对 4 号的方位角}；没测到 4 号标签时返回 None
    """

    means = {tag_id: near_center_mean(items)
             for tag_id, items in samples.items()}
    if REF_TAG not in means:
        print(f"没测到 {REF_TAG} 号标签，无法以它定基准。"
              "请调整俯仰角或确认标签在场地内。")
        return None

    base = means[REF_TAG]
    measured = {tag_id: (value - base) % 360 for tag_id, value in means.items()}

    print(f"\n==== 测量结果（以 {REF_TAG} 号标签为 0°）====")
    old_header = f"旧地图(相对{REF_TAG}号)"
    print(f"{'标签':>4}  {'实测方位':>8}  {'样本':>4}  {'离散':>6}"
          f"  {old_header}  {'差':>7}")
    old_base = TAG_ANGLE.get(REF_TAG, 0)
    for tag_id in sorted(measured):
        items = samples[tag_id]
        spread = max(abs(shortest_delta(means[tag_id], angle))
                     for _, angle, _ in items)
        old_rel = (TAG_ANGLE.get(tag_id, 0) - old_base) % 360
        diff = shortest_delta(old_rel, measured[tag_id])
        print(f"{tag_id:>4}  {measured[tag_id]:7.1f}°  {len(items):>4}"
              f"  {spread:5.1f}°  {old_rel:15.1f}°  {diff:+6.1f}°")

    drift = loop_drift(samples)
    if drift is not None:
        print(f"\n两圈之间的方位漂移：{drift:+.1f}°"
              "（绝对值大说明 GIMBAL_DEG_PER_SEC 需要重新标定）")

    print("\n按下面这段替换 field_map.py 的 TARGETS（距离沿用旧值）：")
    print("TARGETS = {")
    for tag_id in sorted(measured):
        distance = OLD_TARGETS.get(tag_id, {}).get("distance_cm", 0)
        angle = round(measured[tag_id]) % 360
        print(f'    {tag_id:>2}: {{"angle": {angle:>3}, '
              f'"distance_cm": {distance}}},')
    print("}")
    return measured


# ============================== 主流程 ==============================
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
            "      2) 查看实际串口号；\n"
            "      3) 把串口号填入 config.py 的 SERIAL_PORT。"
        )

    try:
        board.stop_motor_b()
        board.set_servo_angle(SCAN_PITCH)

        start, pitch = wait_start(camera, detector, SCAN_PITCH)
        if not start:
            print("用户取消。")
            return

        print(f"开始扫描：俯仰 {pitch}°，速度 {SCAN_SPEED}，共 {SCAN_LOOPS} 圈……")
        samples = scan(camera, detector)
        if samples is None:
            return
        if not samples:
            print("一圈下来没识别到任何标签，检查俯仰角、光照和标签朝向。")
            return
        report(samples)
    finally:
        if board.is_connected():
            pan(0)
            board.disconnect()
        camera.release()
        vision.close_windows()


if __name__ == "__main__":
    main()
