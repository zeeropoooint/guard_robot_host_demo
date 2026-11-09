"""云台电机（电机 B）单独测试：不碰摄像头、不识别，只验证电机听不听使唤。

用法：uv run tools/test_motor.py
  每步会在终端打印预期动作，请盯着云台看实际是否一致：
    1. 正转 2 秒（速度 +160）
    2. 停 1 秒
    3. 反转 2 秒（速度 -160）
    4. 停 1 秒
    5. 低速爬坡：60 → 100 → 200 → 255，各 1 秒（找出电机的"死区"：
       低于某个速度电机可能根本不转，对准抖动排查要用到这个值）

判断标准：
  - 全程纹丝不动 → 接线/串口/供电问题
  - 高速转、低速不转 → 有死区，低速对齐时速度不能低于它
  - 方向和打印的相反 → VOLTAGE_POLARITY 或接线反了
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # 项目根目录加入模块搜索路径

from board import board
from config import SERIAL_PORT


def step(speed, seconds, note):
    print(f"  → {note}（速度 {speed:+d}，{seconds} 秒）……", flush=True)
    board.set_motor_b(speed)
    time.sleep(seconds)
    board.stop_motor_b()


def main():
    print("云台电机（电机 B）单独测试")
    print("请确保云台周围没有线缆缠绕，回车后开始……")
    input()

    board.connect(SERIAL_PORT)
    print("串口已连接，开始测试：")
    try:
        step(160, 2.0, "第 1 步：正转")
        print("  → 停 1 秒……")
        time.sleep(1.0)

        step(-160, 2.0, "第 2 步：反转")
        print("  → 停 1 秒……")
        time.sleep(1.0)

        print("第 3 步：低速爬坡（找出电机死区）")
        for speed in (60, 100, 200, 255):
            step(speed, 1.0, f"      速度 {speed}")
            time.sleep(0.5)
    finally:
        board.stop_all()
        board.disconnect()

    print("\n测试结束。")
    print("请回报：每步云台有没有转？方向和打印的一致吗？从哪个速度开始转得动？")


if __name__ == "__main__":
    main()
