"""项目统一配置：串口、电压极性、摄像头编号。

多个示例共用这里的参数，
更换硬件连接时只需修改此文件。
"""

SERIAL_PORT = "COM11"   # 主控板串口号
VOLTAGE_POLARITY = 1   # 默认为 1，云台转向与画面修正方向相反时设为 -1
CAMERA_INDEX = 1      # 摄像头编号

# 云台电机"速度 → 转角"开环换算：转动角度 = 角速度 × 时间。
# 预置地图导航靠这两个系数定时转到目标方位，换电池后建议用
# tools/calibrate_rotation.py 重新标定。
GIMBAL_SPEED_REF = 180        # 标定时使用的电机速度
GIMBAL_DEG_PER_SEC = 25.9     # 该速度下云台每秒转过的角度
GIMBAL_CLOCKWISE_SIGN = 1     # 正速度对应俯视顺时针（角度增大）取 +1，反之 -1
