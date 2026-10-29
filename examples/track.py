"""云台（电机 B）跟踪 AprilTag

策略：
    没发现标签 → 云台停止
    发现标签   → 云台转动
    标签居中   → 云台停止

操作：在摄像头窗口按 Q 或 Esc 退出。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # 项目根目录加入模块搜索路径

from board import board
from vision import vision
from config import SERIAL_PORT, VOLTAGE_POLARITY

MOTOR_SPEED = 200        # 云台转速度
CENTER_HALF_WIDTH = 15   # 标签离画面中心多少像素以内算"居中"

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
                speed = motor_speed(error)   # 根据偏差算出云台速度
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
