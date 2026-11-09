"""云台转速标定：实测"电机速度 → 云台角速度"的换算系数和转向符号。

云台电机是速度控制（无位置反馈），track_fire.py 的开局定位靠
"定时转动"估算角度，所以必须先实测：满速（255）时云台每秒转多少度。

用法：uv run tools/calibrate_rotation.py
  1. 把云台对准一个明显参照物（或在地面上贴个标记），记好起始朝向
  2. 第一次旋转：云台以最大速度（255）旋转 TURN_SECONDS 秒后停止，
     量出实际转过的角度（建议用手机指南针/量角器，转整圈最准），
     以及从顶上往下看是顺时针还是逆时针 → 算出粗略系数
  3. 验证微调：按粗略系数定时旋转一整圈（360°），回报实际位置
     比起始朝向差了多少度 → 自动修正系数
  4. 脚本算出 GIMBAL_DEG_PER_SEC 和 GIMBAL_CLOCKWISE_SIGN，
     把打印的三行抄进 config.py 即可

提示：验证环节差得越少系数越准；差得还多就再跑一次整个标定。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # 项目根目录加入模块搜索路径

import time

from board import board
from config import SERIAL_PORT, VOLTAGE_POLARITY

TURN_SECONDS = 3.0   # 第一次旋转的时长（秒）；嫌量角度不方便可以改成转更久
SPEED = 255          # 标定用的电机速度，取电机最大值（board.set_motor_b 上限 ±255）


def rotate_timed(seconds):
    """以标定速度定时旋转，转完立即停止。"""

    board.set_motor_b(SPEED * VOLTAGE_POLARITY)
    time.sleep(seconds)
    board.stop_motor_b()


def main():
    print("云台转速标定")
    print("请先把云台对准一个明显参照物（或贴个标记），记好起始朝向。")
    input(f"\n【第一次旋转】回车后以最大速度 {SPEED} 旋转 {TURN_SECONDS} 秒……")

    board.connect(SERIAL_PORT)
    try:
        rotate_timed(TURN_SECONDS)

        print("\n旋转结束。请测量云台实际转过的角度")
        print("（俯视判断方向；转过整圈的话按 360 的倍数 + 零头计算）。")
        degrees = float(input("转过的角度（度）：").strip())
        clockwise = input("从顶上往下看是顺时针吗？(y/n)：").strip().lower() == "y"

        if degrees <= 0:
            print("角度必须大于 0，标定失败，请重跑一次。")
            return

        deg_per_sec = degrees / TURN_SECONDS
        sign = 1 if clockwise else -1
        print(f"粗略系数：{deg_per_sec:.1f} °/秒")

        # 第二次：按粗略系数定时转一整圈，验证并微调
        circle_seconds = 360 / deg_per_sec
        input(f"\n【验证微调】回车后按粗略系数旋转一整圈（{circle_seconds:.1f} 秒），"
              f"理论上应正好回到起始朝向……")
        rotate_timed(circle_seconds)

        print("\n旋转结束。看看云台相对起始朝向差了多少：")
        print("  多转了（越过起始朝向）填正数，没转够填负数，正好对上填 0。")
        error = float(input("相差角度（度）：").strip())

        if error:
            # 实际转角 = 360 + 误差，系数按同比例修正
            deg_per_sec *= (360 + error) / 360
            print(f"修正后系数：{deg_per_sec:.1f} °/秒（误差 {error:+.1f}° 已计入）")
            if abs(error) > 20:
                print("⚠️ 误差偏大，建议把修正后的系数填入后再跑一次标定复核。")
        else:
            print("正好一整圈，系数无需修正。")
    finally:
        board.stop_all()
        board.disconnect()

    print("\n=== 标定结果 ===")
    print(f"角速度：{deg_per_sec:.1f} °/秒（速度 {SPEED} 时）")
    print("把下面三行抄进 config.py：")
    print(f"GIMBAL_SPEED_REF = {SPEED}")
    print(f"GIMBAL_DEG_PER_SEC = {deg_per_sec:.1f}")
    print(f"GIMBAL_CLOCKWISE_SIGN = {sign}")
    print("\n换电池电压/场地后需要重新标定。")


if __name__ == "__main__":
    main()
