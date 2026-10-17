"""控制板示例：按键测试电机、舵机、激光。"""

import sys

from board import board
from config import SERIAL_PORT, resolve_serial_port

MOTOR_SPEED = 200
MOTOR_DURATION_MS = 500


def read_key():
    """读取一个按键（不需要回车、不回显），跨平台实现。

    Windows 使用 msvcrt；macOS/Linux 使用 termios 进入 cbreak 模式
    读取单个字符，读取后恢复终端原有设置。
    输入：无
    输出：按下的字符（str）
    """

    if sys.platform.startswith("win"):
        import msvcrt

        return msvcrt.getwch()

    import termios
    import tty

    file_descriptor = sys.stdin.fileno()
    old_settings = termios.tcgetattr(file_descriptor)
    try:
        tty.setcbreak(file_descriptor)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(file_descriptor, termios.TCSADRAIN, old_settings)

def print_help():
    print(
        """
控制板测试已启动，按以下按键执行动作：
  W / S  舵机上下转动
  A / D  云台左右转动
  O / P  打开 / 关闭激光
  F / L  发射 / 装填
  0      停止全部电机
  1 / 2  电机 A 正转 / 反转 500 ms
  3 / 4  电机 B 正转 / 反转 500 ms
  6 / 9  舵机转到 60 / 90 度
  Q      退出
"""
    )

def main():
    # 命令行传入的串口优先；否则用 config.SERIAL_PORT（为 None 时自动搜索）
    port_arg = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    port = resolve_serial_port(port_arg)
    board.connect(port)
    print_help()

    pitch = 90  # 舵机初始角度

    try:
        board.stop_all()

        while True:
            key = read_key().lower()

            if key == "q":
                print("退出控制板测试")
                break
            elif key == "0":
                board.stop_all()
                print("全部电机停止转动")
            elif key == "1":
                board.run_motor_a_for(MOTOR_SPEED, MOTOR_DURATION_MS)
                print("电机 A 逆时针转动")
            elif key == "2":
                board.run_motor_a_for(-MOTOR_SPEED, MOTOR_DURATION_MS)
                print("电机 A 顺时针转动")
            elif key == "3" or key == "d":
                board.run_motor_b_for(MOTOR_SPEED, MOTOR_DURATION_MS)
                print("电机 B 顺时针转动")
            elif key == "4" or key == "a":
                board.run_motor_b_for(-MOTOR_SPEED, MOTOR_DURATION_MS)
                print("电机 B 逆时针转动")
            elif key == "6":
                board.set_servo_angle(60)
                print("舵机：60 度")
            elif key == "9":
                board.set_servo_angle(90)
                print("舵机：90 度")
            elif key == "w":
                pitch -= 5
                if pitch < 60:
                    pitch = 60
                board.set_servo_angle(pitch)
                print(f"舵机：{pitch} 度")
            elif key == "s":
                pitch += 5
                if pitch > 120:
                    pitch = 120
                board.set_servo_angle(pitch)
                print(f"舵机：{pitch} 度")
            elif key == "o":
                board.laser_on()
                print("激光已打开")
            elif key == "p":
                board.laser_off()
                print("激光已关闭")
            elif key == "l":
                board.load_once()
                print("装填")
            elif key == "f":
                board.fire_once()
                print("发射")
            else:
                print(f"无效按键：{key!r}")
    finally:
        try:
            if board.is_connected():
                board.stop_all()
                board.laser_off()
        finally:
            board.disconnect()

if __name__ == "__main__":
    main()