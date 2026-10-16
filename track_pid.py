"""PD 稳定跟踪：用云台（电机 B）跟踪 AprilTag

策略：
    没发现标签 → 云台停止
    发现标签   → 云台转动（PD 控制：偏差越大转得越快，越接近中心刹车越早）
    标签居中   → 云台停止

相比纯 P 控制多了两项稳定措施：
    低通滤波 —— 滤掉检测坐标的帧间抖动，速度不再忽大忽小
    D 微分项 —— 接近中心时提前减速，减少过冲和来回摆动

操作：在摄像头窗口按 Q 或 Esc 退出。
"""

from board import board
from vision import vision
from control import control
from config import SERIAL_PORT, VOLTAGE_POLARITY

CENTER_HALF_WIDTH = 15  # 标签离画面中心多少像素以内算"居中"
KP = 3.0                # P 比例系数：偏差每 1 像素对应的转速（越大越灵敏）
KD = 0.5                # D 微分系数：偏差每秒收敛 1 像素对应的刹车力（设为 0 退回纯 P 控制）
ALPHA = 0.4             # 低通滤波系数 0～1（越小越平滑、响应越迟钝）
MAX_SPEED = 255         # 云台最大速度（转速上限）
MIN_SPEED = 150         # 云台最小速度（太低电机带不动，太高会过冲，取能爬行的最低值）
D_LIMIT = 150           # 刹车力上限：接近中心时最多减速多少，防止刹车过猛或不刹车

# 稳定控制器在 control/control.py 中实现，track_fire.py 与本文件共用
stabilizer = control.Stabilizer(
    kp=KP,
    kd=KD,
    alpha=ALPHA,
    max_speed=MAX_SPEED,
    min_speed=MIN_SPEED,
    deadzone=CENTER_HALF_WIDTH,
    d_limit=D_LIMIT,
)


def motor_speed(error):
    """根据偏差算出云台速度（低通滤波 + PD 控制）。

    输入：error 偏差（偏右为正，偏左为负，越远数字越大）
    输出：速度；偏差在居中范围内时返回 0。
    """

    return stabilizer.update(error) * VOLTAGE_POLARITY


def main():
    detector = vision.create_detector() # 创建标签检测器
    camera = vision.open_camera() # 打开摄像头

    try:
        board.connect(SERIAL_PORT) # 连接控制板
        board.set_servo_angle(90)  # 云台初始角度
        board.stop_motor_b()   # 一开始云台不动
        last_speed = 0         # 上一次发送的速度（0 表示停止）
        print("云台已就绪：发现标签才开始转动。")

        while True:
            frame, tags = vision.read_tags(camera, detector)
            if frame is None:
                continue

            center_x = frame.shape[1] // 2   # 画面中心

            # 有标签：按偏差算速度；没标签：直接停止
            if tags:
                # 偏差 = 测量值（标签位置）− 期望值（画面中心 0）
                error = int(tags[0].center[0] - center_x)
                speed = motor_speed(error)   # PD 稳定控制
                status=f"偏差：{error}"
            else:
                speed = 0                   # 没发现标签，云台停止
                status="未发现标签"

            status=f"{status} |  云台速度：{speed}"

            # 只在速度变化时才发送命令
            if speed != last_speed:
                board.set_motor_b(speed)
                last_speed = speed
                status= f"{status} |=> 发送指令"
                print(status)

            vision.show_frame(frame, tags, status)
            if vision.is_quit_key(vision.wait_key()):
                print("按 Q 或 Esc 退出。")
                break
    finally:
        # 收尾：不管正常退出还是出错，都会执行
        if board.is_connected():   # 已连接才需要停机和断开
            board.stop_motor_b()
            board.disconnect()
        camera.release()           # 关掉摄像头
        vision.close_windows()     # 关掉窗口


if __name__ == "__main__":
    main()
