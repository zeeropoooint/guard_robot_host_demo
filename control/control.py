"""云台稳定控制：低通滤波 + PD 速度控制。

供 track_pid.py、track_fire.py 等跟踪示例共用：
    stabilizer = control.Stabilizer(kp=3, kd=0.5, alpha=0.4, ...)
    speed = stabilizer.update(error)     # error 为标签偏差（像素）

内部两步：
    1. 低通滤波（指数滑动平均）：滤掉检测坐标的帧间抖动，
       避免检测噪声直接变成电机速度的忽大忽小
    2. PD 控制：
       P 项 —— 偏差越大转速越快（和纯 P 控制一样）
       D 项 —— 偏差收敛越快"刹车"越狠，接近中心时提前减速，抑制过冲
"""

import time


class Stabilizer:
    """把"标签偏差（像素）"变成"云台速度"的稳定控制器。

    输出的速度不包含方向极性（VOLTAGE_POLARITY），由调用方决定。
    """

    def __init__(self, kp, kd, alpha, max_speed, min_speed, deadzone, d_limit=150):
        """创建控制器。

        输入：kp 比例系数（偏差每 1 像素对应的转速）；
             kd 微分系数（偏差每秒收敛 1 像素对应的刹车力，0 即退回纯 P 控制）；
             alpha 低通滤波系数 0～1（越小越平滑、响应越迟钝）；
             max_speed 转速上限（太快会转过头）；
             min_speed 转速下限（太低电机带不动）；
             deadzone 死区（偏差在该像素范围内视为居中，输出 0）；
             d_limit 刹车力上限（防止过冲，也防止单帧噪声产生速度尖峰）。
        输出：无。
        """

        self.kp = kp
        self.kd = kd
        self.alpha = alpha
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.deadzone = deadzone
        self.d_limit = d_limit
        self.filtered_error = None   # 上一次滤波后的偏差（None 表示还没收到数据）
        self.last_time = None        # 上一次更新的时间（算微分用）

    def update(self, error):
        """输入本次原始偏差，输出云台速度。

        输入：error 偏差（偏右为正，偏左为负，单位像素）
        输出：速度（不含极性，整数）；偏差在死区内时返回 0。
        """

        now = time.perf_counter()

        if self.filtered_error is None:
            # 第一帧：没有历史数据，滤波值直接取原始值，微分项为 0
            self.filtered_error = float(error)
            d_term = 0.0
        else:
            # 低通滤波：新滤波值 = 旧值 + alpha ×（新测量 − 旧值）
            previous = self.filtered_error
            self.filtered_error += self.alpha * (error - self.filtered_error)
            # D 项刹车力：与滤波后偏差的收敛速度（像素/秒）成正比，并限幅
            dt = max(now - self.last_time, 1e-3)   # 距上帧的时间（秒），防除零
            d_term = self.kd * (self.filtered_error - previous) / dt
            d_term = max(-self.d_limit, min(self.d_limit, d_term))
        self.last_time = now

        if abs(self.filtered_error) < self.deadzone:
            return 0   # 偏差进入死区，视为居中

        # PD 合成速度；负号让云台朝偏差减小的方向转
        p_term = self.kp * self.filtered_error
        speed = -(p_term + d_term)

        # D 项只刹车不反向：刹车力超过驱动力时直接停车滑行，等下一帧再算
        if (speed > 0) != (p_term < 0):
            return 0

        # 限幅：保留方向，把大小限制在 min～max 之间
        direction = 1 if speed >= 0 else -1
        magnitude = max(self.min_speed, min(self.max_speed, abs(speed)))
        return int(direction * magnitude)   # 取整：电机速度必须是整数
