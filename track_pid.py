"""PID 比例控制：用云台（电机 B）跟踪 AprilTag

策略：
    没发现标签 → 云台停止
    发现标签   → 云台转动（比例控制：偏差越大转速越大）
    标签居中   → 云台停止

操作：在摄像头窗口按 Q 或 Esc 退出。
"""

from board import board
from vision import vision
from config import SERIAL_PORT, VOLTAGE_POLARITY

CENTER_HALF_WIDTH = 15 # 标签离画面中心多少像素以内算"居中"
MAX_SPEED = 255        # 云台最大速度（转速上限）
MIN_SPEED = 150        # 云台最小速度（太低电机带不动，太高会过冲，取能爬行的最低值）
KP = 3                 # PID 比例系数：偏差每 1 像素对应的转速

def motor_speed(error):
    """根据偏差算出云台速度（比例控制：偏差越大，转得越快）。

    偏差：偏右为正，偏左为负，越远数字越大。
    举例：偏差 30 像素 × 比例系数 3 = 速度 90
    """
    # 在偏差范围内，speed 设置为 0
    if abs(error) < CENTER_HALF_WIDTH:
        return 0

    # 按比例算速度，负号让云台朝标签方向转
    speed = int(-error * KP)

    # 限幅：太快会转过头，太慢电机带不动
    if speed >= 0:
        direction = 1
    else:
        direction = -1
    magnitude = max(MIN_SPEED, min(MAX_SPEED, abs(speed)))
    return direction * magnitude * VOLTAGE_POLARITY


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
                speed = motor_speed(error)   # PID 比例控制
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
