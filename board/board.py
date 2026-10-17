"""机器人控制板模块。

控制板连接、断开、检查连接状态、发送自定义指令
connect()：连接串口
disconnect()：断开串口
is_connected()：检查是否已连接
send_command(command: str)：发送自定义指令

功能函数与指令对照表（{...} 为参数占位符）：
    函数名称                        发送指令                  说明
    set_servo_angle(angle)         @S{angle}                舵机转到 0～180 度
    set_motor_a(speed)             @MA{speed}               电机 A 速度 -255～255
    set_motor_b(speed)             @MB{speed}               电机 B 速度 -255～255
    stop_motor_a()                 @MA0                     停止电机 A
    stop_motor_b()                 @MB0                     停止电机 B
    stop_all()                     @MA0 + @MB0              停止全部电机
    run_motor_a_for(speed, ms)     @MTA{speed},{ms}         电机 A 运行指定毫秒
    run_motor_b_for(speed, ms)     @MTB{speed},{ms}         电机 B 运行指定毫秒
    fire_once()                    @FS                      发射一次
    load_once()                    @ZZ                      装填一次
    laser_on()                     @JGON                    打开激光
    laser_off()                    @JGOFF                   关闭激光

组合功能函数：
    laser_on_for(seconds)        @JGON + 等待 + @JGOFF 的组合，指定秒数

典型调用流程：
    board.connect("COM3")        # 1. 连接串口
    board.set_servo_angle(90)    # 2. 控制舵机
    board.fire_once()            # 3. 发射一次
    board.disconnect()           # 4. 断开串口
每个函数的输入/输出见函数注释。
"""

import atexit
import time
import threading
from typing import Any, Callable, Optional

# ============================ 异常类型 ============================

class BoardError(Exception):
    """板卡模块所有错误的基础类。"""


class BoardConnectionError(BoardError):
    """串口未连接或串口操作失败。"""


_serial: Optional[Any] = None
_port: Optional[str] = None
_write_lock = threading.Lock()


def _emergency_stop_on_exit() -> None:
    """解释器退出时的紧急保护：若串口仍打开，直接写入停机指令。

    覆盖未正常走 disconnect() 的退出路径（如未捕获异常导致的退出）；
    直接写底层串口，避免退出过程中其他状态已被清理。
    """

    serial_obj = _serial
    if serial_obj is None:
        return
    try:
        serial_obj.write(b"@MB0\n@MA0\n")
        serial_obj.flush()
    except Exception:
        pass  # 退出阶段忽略一切错误


atexit.register(_emergency_stop_on_exit)

# ============================ 连接和断开控制器 ============================

def connect(
    port: str,
    baudrate: int = 115200,
    timeout: float = 1.0,
    reset_delay: float = 2.0,
    serial_factory: Optional[Callable[..., Any]] = None,
) -> None:
    """打开控制器串口；如果已经连接则不重复打开。"""

    global _serial, _port

    if not isinstance(port, str) or not port.strip():
        raise ValueError(
            "port 必须是非空字符串，例如 'COM3'（Windows）"
            "或 '/dev/cu.usbmodem1101'（macOS）"
        )
    if baudrate <= 0:
        raise ValueError("baudrate 必须大于 0")
    if timeout < 0:
        raise ValueError("timeout 不能小于 0")
    if reset_delay < 0:
        raise ValueError("reset_delay 不能小于 0")
    if is_connected():
        return

    normalized_port = port.strip()
    try:
        factory = serial_factory
        if factory is None:
            # 延迟导入，使用模拟串口测试时不要求预先安装 pyserial。
            import serial

            factory = serial.Serial

        _serial = factory(
            port=normalized_port,
            baudrate=int(baudrate),
            timeout=float(timeout),
            write_timeout=float(timeout),
        )
        _port = normalized_port
        if reset_delay:
            time.sleep(float(reset_delay))
    except Exception as exc:
        _serial = None
        _port = None
        raise BoardConnectionError(f"无法打开串口 {normalized_port}: {exc}") from exc


def is_connected() -> bool:
    """检查串口是否已连接。

    输入：无
    输出：bool，True 表示已连接。
    """

    return bool(_serial is not None and _serial.is_open)


def disconnect() -> None:
    """断开控制器串口；重复调用是安全的。"""

    global _serial, _port

    if _serial is None:
        return

    port = _port
    try:
        _serial.close()
    except Exception as exc:
        raise BoardConnectionError(f"关闭串口 {port} 失败: {exc}") from exc
    finally:
        _serial = None
        _port = None


# ============================ 底层命令 ============================

def send_command(command: str) -> None:
    """发送一条原始 ASCII 命令（自动补换行符）。

    输入：command 命令字符串，如 "@S90"
    输出：无；串口未连接或发送失败抛出 BoardConnectionError。
    """

    if not isinstance(command, str) or not command.strip():
        raise ValueError("command 必须是非空字符串")
    if "\n" in command or "\r" in command:
        raise ValueError("command 内部不能包含换行符")
    if not is_connected():
        raise BoardConnectionError("串口尚未连接，请先调用 connect()")

    payload = (command.strip() + "\n").encode("ascii")
    try:
        with _write_lock:
            written = _serial.write(payload)
            _serial.flush()
        if written != len(payload):
            raise BoardConnectionError(
                f"串口只写入了 {written}/{len(payload)} 字节"
            )
    except BoardConnectionError:
        raise
    except Exception as exc:
        raise BoardConnectionError(f"向串口 {_port} 发送命令失败: {exc}") from exc


# ============================ 动作命令：舵机 ============================

def set_servo_angle(angle: int) -> None:
    """让舵机转到指定角度。

    输入：angle 角度，0～180 之间
    输出：无；参数非法时抛出 TypeError/ValueError。
    """

    angle = _validate_integer("angle", angle, 0, 180)
    send_command(f"@S{angle}")


# ============================ 动作命令：电机 ============================

def set_motor_a(speed: int) -> None:
    """设置电机 A 的速度。

    输入：speed 速度，-255～255（负数反向）
    输出：无；参数非法时抛出 TypeError/ValueError。
    """

    speed = _validate_speed(speed)
    send_command(f"@MA{speed}")


def set_motor_b(speed: int) -> None:
    """设置电机 B 的速度。

    输入：speed 速度，-255～255（负数反向）
    输出：无；参数非法时抛出 TypeError/ValueError。
    """

    speed = _validate_speed(speed)
    send_command(f"@MB{speed}")


def stop_motor_a() -> None:
    """停止电机 A。"""

    set_motor_a(0)


def stop_motor_b() -> None:
    """停止电机 B。

    输入：无
    输出：无
    """

    set_motor_b(0)


def stop_all() -> None:
    """依次停止电机 A 和电机 B。

    输入：无
    输出：无
    """

    stop_motor_a()
    stop_motor_b()


def run_motor_a_for(speed: int, duration_ms: int) -> None:
    """让电机 A 以指定速度运行指定时长后自动停止。

    输入：speed 速度 -255～255；duration_ms 时长（毫秒）
    输出：无；参数非法时抛出 TypeError/ValueError。
    """

    speed = _validate_speed(speed)
    duration_ms = _validate_duration(duration_ms)
    send_command(f"@MTA{speed},{duration_ms}")


def run_motor_b_for(speed: int, duration_ms: int) -> None:
    """让电机 B 以指定速度运行指定时长后自动停止。

    输入：speed 速度 -255～255；duration_ms 时长（毫秒）
    输出：无；参数非法时抛出 TypeError/ValueError。
    """

    speed = _validate_speed(speed)
    duration_ms = _validate_duration(duration_ms)
    send_command(f"@MTB{speed},{duration_ms}")


# ============================ 动作命令：发射与装填 ============================

def fire_once() -> None:
    """执行一次发射动作。

    输入：无
    输出：无
    """

    send_command("@FS")


def load_once() -> None:
    """执行一次装填动作。

    输入：无
    输出：无
    """

    send_command("@ZZ")


# ============================ 动作命令：激光 ============================

def laser_on() -> None:
    """打开激光。

    输入：无
    输出：无
    """

    send_command("@JGON")


def laser_off() -> None:
    """关闭激光。

    输入：无
    输出：无
    """

    send_command("@JGOFF")


def laser_on_for(duration_seconds: float) -> None:
    """打开激光指定秒数，随后自动关闭。

    输入：duration_seconds 秒数（可带小数）
    输出：无；参数非法时抛出 TypeError/ValueError。
    """

    if isinstance(duration_seconds, bool) or not isinstance(
        duration_seconds, (int, float)
    ):
        raise TypeError("duration_seconds 必须是数字")
    if duration_seconds < 0:
        raise ValueError("duration_seconds 不能小于 0")

    laser_on()
    try:
        time.sleep(float(duration_seconds))
    finally:
        laser_off()


# ============================ 内部校验函数 ============================

def _validate_integer(name: str, value: int, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} 必须是整数")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} 必须在 {minimum}～{maximum} 之间")
    return value


def _validate_speed(speed: int) -> int:
    """校验电机速度参数，范围 -255～255。"""

    return _validate_integer("speed", speed, -255, 255)


def _validate_duration(duration_ms: int) -> int:
    """校验时长参数（毫秒）。"""

    return _validate_integer("duration_ms", duration_ms, 0, 4_294_967_295)
