"""项目统一配置：串口、电压极性、摄像头编号。

多个示例共用这里的参数，
更换硬件连接时只需修改此文件。
"""

import sys

# ============================ 串口设置 ============================
# Windows：写端口号，例如 "COM12"
# macOS  ：写设备路径，例如 "/dev/cu.usbmodem1101"
#          （Arduino UNO 原版通常是 /dev/cu.usbmodem*，
#           CH340 等 USB 转串口通常是 /dev/cu.usbserial-* 或 /dev/cu.wchusbserial*）
# Linux  ：写设备路径，例如 "/dev/ttyACM0"、"/dev/ttyUSB0"
# 设为 None 时，程序会自动搜索第一个可用的 USB 串口（推荐 Mac/Linux 使用）。
if sys.platform.startswith("win"):
    SERIAL_PORT = "COM12"   # Windows 主控板串口号
else:
    SERIAL_PORT = None      # macOS/Linux：自动搜索 USB 串口

VOLTAGE_POLARITY = -1   # 默认为 1，云台转向与画面修正方向相反时设为 -1
CAMERA_INDEX = 0        # 摄像头编号

# 自动搜索串口时优先匹配的设备名关键字（按优先级排序）
_PREFERRED_PORT_KEYWORDS = (
    "usbmodem",       # macOS：Arduino UNO（ATmega16U2）
    "wchusbserial",   # macOS：CH340 转接板
    "usbserial",      # macOS：其他 USB 转串口
    "ttyACM",         # Linux：Arduino CDC 设备
    "ttyUSB",         # Linux：CH340/CP2102/FTDI 等转接板
)


def resolve_serial_port(port=None):
    """解析实际使用的串口号。

    输入：port 串口名（如 "COM12"、"/dev/cu.usbmodem1101"）；
         传入 None 时使用 config.SERIAL_PORT，仍为 None 则自动搜索。
    输出：可用的串口号（str）；找不到时抛出 RuntimeError。
    """

    port = port if port is not None else SERIAL_PORT
    if port:
        return port

    # 延迟导入：未安装 pyserial 的纯单元测试环境不会触发依赖
    from serial.tools import list_ports

    ports = list(list_ports.comports())

    # 1) 设备名包含常见 USB 转串口关键字（Arduino / CH340 等）
    for keyword in _PREFERRED_PORT_KEYWORDS:
        for item in ports:
            if keyword in item.device:
                return item.device

    # 2) 任意 USB 串口设备：硬件 ID 中带有 VID:PID
    #    （覆盖命名不规范的 USB 转串口模块）
    for item in ports:
        if "VID:PID" in (item.hwid or "").upper():
            return item.device

    # 3) macOS 的端口列表会混入蓝牙、系统调试等非 USB 端口（如
    #    /dev/cu.Bluetooth-*、/dev/cu.wlan-debug），不能误选；
    #    找不到 USB 串口时直接报错，提示用户手动指定。
    if sys.platform == "darwin":
        raise RuntimeError(
            "未找到 USB 串口设备。请确认控制板已通过 USB 连接、CH340 等驱动已安装，"
            "或在 config.py 中手动指定 SERIAL_PORT（如 \"/dev/cu.usbmodem1101\"）。"
            "可用命令查看端口：ls /dev/cu.*"
        )

    # 4) Windows/Linux 退化为取第一个可用端口
    if ports:
        return ports[0].device

    raise RuntimeError(
        "未找到任何串口设备。请确认控制板已通过 USB 连接、驱动已安装，"
        "或在 config.py 中手动指定 SERIAL_PORT"
        "（Windows 如 \"COM12\"，Linux 如 \"/dev/ttyACM0\"）。"
    )
