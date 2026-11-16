"""测试工具：用摄像头实测云台水平"速度 → 角速度"，不做俯仰、不发舵机指令。

做法（每个档位重复几次）：
    1. 把参照标签转到画面中心（只动水平电机 B）；
    2. 静止连拍几帧，记下起始像素偏移；
    3. 以该速度转，**边转边跟踪**同一个标签，直到累计转角达到
       TARGET_SWEEP_DEG（默认 15°）或超时（默认 2 秒）就停；
    4. 用"已转角 ÷ 已用时间"得到平均角速度（含启停，和导航定时转角同一口径），
       同时用最小二乘拟合匀速段得到瞬时角速度。

为什么转固定角度而不是固定时间：固定时间高速档会把标签甩出画面、
低速档又几乎测不到；转 15° 既保证标签始终在画面里，又不用停下后再认一次。

按键（摄像头窗口上操作）：
    Q / Esc 随时退出（已测到的档位仍会打印）
"""

import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # 项目根目录加入模块搜索路径

from board import board
from vision import vision
from config import SERIAL_PORT, GIMBAL_SPEED_REF, GIMBAL_CLOCKWISE_SIGN
from track_fire import motor_speed, px_to_deg

# ============================== 参数 ==============================
SPEEDS = (60, 90, 120, 150, 180, 210, 255)  # 要测的速度档位
REPEATS = 3              # 每档重复次数（取中位数）
TARGET_SWEEP_DEG = 15.0  # 每次转到这个累计角度就停（保证标签不出画面）
MAX_RUN_SECONDS = 2.0    # 转不到目标角度也停（用来测低速/死区）
SETTLE_SECONDS = 0.25    # 停下后等一下，避免启动瞬间读到残影
CENTER_SPEED = 180       # 回中时的云台速度（与主程序任务 1/2/3 一致，低速可能低于死区）
CENTER_TOLERANCE_PX = 40  # 偏差小于这个像素就算已居中（留死区，避免来回抖）
SEARCH_SPEED = 180       # 找不到参照标签时的搜索速度
SEARCH_TIMEOUT = 10.0    # 搜索参照标签的最长时间（秒）
CENTER_TIMEOUT = 5.0     # 看到标签后回中的最长时间（秒）
DEAD_ZONE_DEG = 1.5      # 转过不到这个角度就认为没动（噪声量级）
MOVE_THRESHOLD_DEG = 0.3  # 判定"开始转动"的角度阈值
SELF_CHECK_SPEED = 150   # 自检转动速度（满速拖影重，容易识别不到标签）
SELF_CHECK_SPIN = 0.4    # 自检转动时长（秒）
SELF_CHECK_DEG = 1.0     # 自检转过超过这个角度就算云台正常
STABLE_PX = 2.0          # 相邻两帧偏移差小于该值就算停稳（像素）
STABLE_TIMEOUT = 2.0     # 等画面停稳的最长时间（秒）
MAX_FAILURES = 3         # 连续几次回中失败就结束（多半是标签丢了）


# ============================== 基础工具 ==============================
def quit_pressed():
    """每帧调用一次，刷新窗口并返回是否按了退出键。"""

    return vision.is_quit_key(vision.wait_key(1))


def wait_stable_offset(camera, detector, tag_id):
    """等画面停稳后，取标签此刻的实际位置（回中后会因惯性继续滑一段）。

    输入：camera / detector；tag_id 参照标签
    输出：(像素偏移, 画面宽度)；标签一直不在画面里返回 None
    """

    last = None
    width = None
    deadline = time.monotonic() + STABLE_TIMEOUT
    while time.monotonic() < deadline:
        frame, tags = vision.read_tags(camera, detector)
        if frame is None:
            continue
        width = frame.shape[1]
        tag = next((item for item in tags if item.tag_id == tag_id), None)
        vision.show_frame(frame, tags, "等云台停稳……")
        vision.wait_key(1)
        if tag is None:
            continue
        offset = tag.center[0] - width / 2.0
        if last is not None and abs(offset - last) <= STABLE_PX:
            return offset, width    # 相邻两帧几乎没动，就是当前真实位置
        last = offset
    if last is None:
        return None
    return last, width              # 惯性还没完全停住，用最后一次实测值


def check_motion(camera, detector, tag_id):
    """启动自检：短促转一下再停稳测量，确认云台确实收到指令并会动。

    转动中画面拖影重、常识别不到标签，所以不在转动中判读，
    而是"转一段 → 停稳 → 比较前后角度"。
    输入：camera / detector；tag_id 参照标签
    输出：转过的角度（度）；参照标签不在画面里返回 None
    """

    info = wait_stable_offset(camera, detector, tag_id)
    if info is None:
        return None
    before, width = info

    board.set_motor_b(SELF_CHECK_SPEED * GIMBAL_CLOCKWISE_SIGN)
    deadline = time.monotonic() + SELF_CHECK_SPIN
    while time.monotonic() < deadline:
        frame, tags = vision.read_tags(camera, detector)
        if frame is not None:
            vision.show_frame(frame, tags, "启动自检：转动中……")
        vision.wait_key(1)
    board.stop_motor_b()

    info = wait_stable_offset(camera, detector, tag_id)
    if info is None:
        return None
    after, after_width = info
    return abs(px_to_deg(after, after_width) - px_to_deg(before, width))


def wait_unlocked():
    """A 键锁定期间云台不动，等解锁后再继续测；用户退出返回 False。"""

    while board.motor_b_locked():
        print("A 键锁定中，请按一下 A 键解锁（Q 退出）……")
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if quit_pressed():
                return False
    return True


def center_tag(camera, detector, tag_id):
    """把参照标签转到画面中心；找不到时慢速搜索。

    输入：camera / detector；tag_id 参照标签
    输出：True 到位；False 用户退出或超时
    """

    deadline = time.monotonic() + SEARCH_TIMEOUT
    center_deadline = None
    try:
        while True:
            frame, tags = vision.read_tags(camera, detector)
            if frame is None:
                continue
            tag = next((item for item in tags if item.tag_id == tag_id), None)
            if tag is None:
                center_deadline = None          # 标签丢了，退回搜索计时
                board.set_motor_b(SEARCH_SPEED * GIMBAL_CLOCKWISE_SIGN)
                vision.show_frame(frame, tags, f"搜索标签 {tag_id}……")
                if quit_pressed():
                    return False
                if time.monotonic() > deadline:
                    print(f"找不到参照标签 {tag_id}（{SEARCH_TIMEOUT:.0f} 秒）。"
                          "检查标签是否被挡住、云台是否被 A 键锁定。")
                    return False
                continue

            deadline = time.monotonic() + SEARCH_TIMEOUT   # 找到了，重置搜索计时
            if center_deadline is None:
                center_deadline = time.monotonic() + CENTER_TIMEOUT
            error = tag.center[0] - frame.shape[1] / 2.0
            if abs(error) <= CENTER_TOLERANCE_PX:   # 差不多居中就够了
                board.stop_motor_b()
                return True
            board.set_motor_b(motor_speed(error, CENTER_SPEED))
            vision.show_frame(frame, tags,
                              f"回中：标签 {tag_id} 偏移 {error:+.0f}px")
            if time.monotonic() > center_deadline:
                print(f"回中超时（{CENTER_TIMEOUT:.0f} 秒）：云台几乎没动。\n"
                      f"  常见原因：A 键锁定（旧固件不会上报，主机看不到，请按一下 A 键）；\n"
                      f"  或速度 {CENTER_SPEED} 仍低于死区、云台被卡住/断电。\n"
                      "  先按当前画面继续测，结果里能看出是否所有档位都不转。")
                return True     # 继续测：测出来的 0°/s 本身就是有效信息
            if quit_pressed():
                return False
    finally:
        board.stop_motor_b()


def pick_reference(camera, detector):
    """等画面里出现标签，选离中心最近的那个作参照。

    输入：camera / detector
    输出：参照标签编号；用户退出返回 None
    """

    print("请把场地里的二维码放在画面中（任意一个即可），按 Q 退出。")
    while True:
        frame, tags = vision.read_tags(camera, detector)
        if frame is not None:
            ids = " ".join(str(item.tag_id) for item in tags) or "-"
            vision.show_frame(frame, tags, f"选择参照标签：可见 {ids}  Q 退出")
        if frame is not None and tags:
            tag = min(tags, key=lambda item: abs(item.center[0] - frame.shape[1] / 2))
            print(f"参照标签：{tag.tag_id} 号")
            return tag.tag_id
        if quit_pressed():
            return None


def fit_slope(samples):
    """用匀速段样本最小二乘拟合瞬时角速度。

    输入：samples [(相对时间, 已转角), ...]
    输出：角速度（度/秒）；样本不足返回 None
    """

    points = list(samples)
    if len(points) < 5:
        return None
    mean_t = sum(t for t, _ in points) / len(points)
    mean_a = sum(angle for _, angle in points) / len(points)
    denom = sum((t - mean_t) ** 2 for t, _ in points)
    if denom < 1e-6:
        return None
    return sum((t - mean_t) * (angle - mean_a) for t, angle in points) / denom


# ============================== 测量 ==============================
def run_once(camera, detector, tag_id, speed):
    """以指定速度转过 TARGET_SWEEP_DEG，返回 (平均角速度, 瞬时角速度, 样本数)。"""

    start_info = wait_stable_offset(camera, detector, tag_id)
    if start_info is None:
        print("  参照标签不在画面里，跳过本次。")
        return None
    start, width = start_info
    start_angle = px_to_deg(start, width)

    samples = []
    # 让标签朝"穿过画面中心"的方向走，起始偏左/偏右都留足视野余量
    direction = 1 if start >= 0 else -1
    board.set_motor_b(direction * speed * GIMBAL_CLOCKWISE_SIGN)
    t0 = time.monotonic()
    try:
        while True:
            frame, tags = vision.read_tags(camera, detector)
            now = time.monotonic()
            if frame is not None:
                tag = next((item for item in tags if item.tag_id == tag_id), None)
                if tag is not None:
                    swept = px_to_deg(tag.center[0] - frame.shape[1] / 2.0,
                                      frame.shape[1]) - start_angle
                    samples.append((now - t0, swept))
                    vision.show_frame(frame, tags,
                                      f"测速 {speed}：{now - t0:.2f}s，"
                                      f"已转 {abs(swept):.1f}°")
                    if abs(swept) >= TARGET_SWEEP_DEG:
                        break
            if now - t0 >= MAX_RUN_SECONDS:
                break
            if quit_pressed():
                break
    finally:
        board.stop_motor_b()

    if not samples:
        print("  转动中一帧都没识别到参照标签（可能拖影太重），跳过本次。")
        return None
    elapsed, swept = samples[-1]
    if elapsed <= 0:
        return None
    instant = fit_slope(samples)
    delay = next((t for t, angle in samples
                  if abs(angle) >= MOVE_THRESHOLD_DEG), None)
    return (abs(swept) / elapsed,
            abs(instant) if instant is not None else None,
            len(samples), delay)


def report(results):
    """打印速度→角速度汇总并给出参数建议。

    输入：results {速度: [(平均角速度, 瞬时角速度, 样本数), ...]}
    输出：无
    """

    print("\n==== 测速结果（摄像头实测水平转角，含启停）====")
    print(f"{'速度':>4}  {'平均角速度':>10}  {'瞬时角速度':>10}  {'次数':>4}")

    table = []
    for speed in SPEEDS:
        rows = results.get(speed)
        if not rows:
            continue
        average = statistics.median(row[0] for row in rows)
        instant_values = [row[1] for row in rows if row[1] is not None]
        instant = statistics.median(instant_values) if instant_values else None
        table.append((speed, average))
        instant_text = f"{instant:9.1f}°/s" if instant is not None else "        --"
        print(f"{speed:>4}  {average:9.1f}°/s  {instant_text}  {len(rows):>4}")

    if not table:
        print("没有有效数据。")
        return

    moved = [speed for speed, average in table if abs(average) * MAX_RUN_SECONDS
             >= DEAD_ZONE_DEG]
    if moved:
        lowest = min(speed for speed, _ in table)
        if moved[0] > lowest:
            print(f"\n实测：{moved[0]} 及以上能转，{lowest} 档基本不动（死区）。")
        else:
            print(f"\n实测：最低档 {lowest} 也能转，没测到明显死区。")
    else:
        print(f"\n所有档位都几乎没转（<{DEAD_ZONE_DEG}°）：云台可能没收到指令"
              "或被 A 键锁定。")

    reference = next((row for row in table if row[0] == GIMBAL_SPEED_REF), None)
    if reference is not None:
        print(f"当前 GIMBAL_SPEED_REF = {GIMBAL_SPEED_REF}，"
              f"实测建议 GIMBAL_DEG_PER_SEC = {reference[1]:.1f}")

    ratios = [(speed, average / speed) for speed, average in table if speed]
    if len(ratios) >= 2:
        mean_ratio = sum(value for _, value in ratios) / len(ratios)
        worst = max(ratios, key=lambda item: abs(item[1] / mean_ratio - 1))
        print("各档 角速度/速度：" + "  ".join(
            f"{speed}:{value:.3f}" for speed, value in ratios))
        print(f"最大偏离 {abs(worst[1] / mean_ratio - 1) * 100:.1f}%（{worst[0]} 档）；"
              "偏离大说明不能简单按比例换算，导航应按实测表插值。")


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

        tag_id = pick_reference(camera, detector)
        if tag_id is None:
            print("用户取消。")
            return

        moved = check_motion(camera, detector, tag_id)
        if moved is None:
            print("自检时参照标签跑出画面了，请把二维码放到画面中央后重跑。")
            return
        if moved < SELF_CHECK_DEG:
            print(f"自检：{SELF_CHECK_SPIN:.1f} 秒只转了 {moved:.1f}°，云台没怎么动。\n"
                  "  1) 按一下 A 键（锁定期间固件会丢弃 @MB；旧固件不上报锁状态）；\n"
                  "  2) 检查云台电机供电与接线；\n"
                  "  3) 先在主程序里按数字键，确认云台本身能转。")
            return
        print(f"云台响应正常：{SELF_CHECK_SPIN:.1f} 秒转了 {moved:.1f}°。")

        print(f"开始测速：每档 {REPEATS} 次，每次转 {TARGET_SWEEP_DEG:.0f}°。")
        time.sleep(SETTLE_SECONDS)
        results = {}
        failures = 0
        aborted = False
        for speed in SPEEDS:
            for index in range(REPEATS):
                if not wait_unlocked():
                    print("用户取消。")
                    aborted = True
                    break
                if not center_tag(camera, detector, tag_id):
                    failures += 1
                    print(f"  回中失败（连续 {failures} 次），跳过本次。")
                    if failures >= MAX_FAILURES:
                        print("连续回中失败，结束测速。")
                        aborted = True
                        break
                    continue
                failures = 0

                row = run_once(camera, detector, tag_id, speed)
                if row is None:
                    continue
                results.setdefault(speed, []).append(row)
                instant_text = f"{row[1]:.1f}" if row[1] is not None else "--"
                delay_text = f"{row[3] * 1000:.0f}ms" if row[3] is not None else "没动"
                print(f"  {speed:>3} 档 第 {index + 1} 次："
                      f"平均 {row[0]:.1f}°/s，瞬时 {instant_text}°/s，"
                      f"启动 {delay_text}，{row[2]} 帧")
                time.sleep(SETTLE_SECONDS)
            if aborted:
                break
        report(results)
    finally:
        if board.is_connected():
            board.stop_motor_b()
            board.disconnect()
        camera.release()
        vision.close_windows()


if __name__ == "__main__":
    main()
