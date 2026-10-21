# 环境配置

> 系统：Windows
> Python 版本：3.11（由 uv 自动下载，无需预装 Python）

## 1. 安装 uv

从 [uv releases](https://github.com/astral-sh/uv/releases) 下载 `uv-x86_64-pc-windows-msvc.zip`，解压后将 `uv.exe` 放到 `d:\software\uv` 目录
将 `d:\software\uv` 添加到环境变量 Path 中
具体操作如下：

1. 打开 **系统属性**（右键点击 **此电脑** 或 **计算机**，选择 **属性**）。
2. 点击 **高级系统设置**。
3. 点击 **环境变量**。
4. 在 **系统变量** 中，找到 **Path** 变量，点击 **编辑**。
5. 点击 **新建**，输入 `d:\software\uv`。
6. 点击 **确定** 关闭所有弹窗。

## 2. 同步依赖

打开终端，进入项目目录：

```bash
cd <项目根目录>
```

执行：

```bash
uv sync
```

uv 会自动下载 Python 3.11、创建 `.venv` 虚拟环境，并安装 `pyproject.toml` 中声明的全部依赖。后续每次拉取新代码后，再次执行 `uv sync` 即可保持依赖与 `uv.lock` 完全一致。

> 依赖源：项目已在 `pyproject.toml` 的 `[tool.uv.index]` 中配置 **阿里云 PyPI 镜像**（`https://mirrors.aliyun.com/pypi/simple/`）作为默认源，`uv sync` 时自动生效、无需额外操作；如需临时切换官方源，可在命令后追加 `--default-index https://pypi.org/simple/`。

# 配置说明

运行前请根据实际硬件修改以下参数。三个关键参数（串口、电压极性、摄像头索引）统一放在项目根目录的 **`config.py`** 中，所有示例（track.py、track\_pid.py、track\_fire.py、test\_board.py 等）共用，**只需修改这一处**。

**文件**：`config.py`

```python
SERIAL_PORT = "COM12"   # 控制板串口号
VOLTAGE_POLARITY = -1   # 云台转向与画面修正方向相反时设为 -1
CAMERA_INDEX = 0        # 摄像头索引（Windows DirectShow 枚举顺序）
```

## 1. 串口编号设置（SERIAL\_PORT）

**查看可用串口**：

- 方法 A：设备管理器 → **端口 (COM 和 LPT)** → 查看单片机对应的端口号
- 方法 B：在终端执行 `mode` 命令列出所有 COM 端口
- 方法 C：用 Python 列出（需已安装 pyserial）：
  ```powershell
  uv run python -c "import serial.tools.list_ports; [print(p.device) for p in serial.tools.list_ports.comports()]"
  ```

将 `COM12` 改为你查到的实际端口号，例如：

```python
SERIAL_PORT = "COM13"
```

> 若运行时报 `PermissionError(13, '拒绝访问。')`，通常是端口号错误、被其他程序占用，或 USB 转串口驱动未正确安装。

## 2. 摄像头编号设置（CAMERA\_INDEX）

**编号含义**：

- `0` → 系统默认摄像头（通常是笔记本内置摄像头）
- `1` → 第二个摄像头（通常是外接 USB 摄像头）
- `2、3...` → 更多摄像头，按接入顺序递增

**如何确定编号**：依次尝试 `0`、`1`、`2` 直到画面正常出现。如果运行时报 `无法通过 DirectShow 打开摄像头`，说明编号不匹配或摄像头被占用。

## 3. 电压极性设置（VOLTAGE\_POLARITY）

- `1` → 云台旋转方向与画面修正方向一致（默认）
- `-1` → 云台旋转方向与画面修正方向相反时使用（当前值）

若出现"标签偏左，云台却向左转"这类方向相反的现象，把 `1` 改成 `-1`（或把 `-1` 改成 `1`）。

> 说明：`track.py` / `track_pid.py` 同样使用 `config.py` 里的 `VOLTAGE_POLARITY`，与 `test_board.py` 等示例一致，统一在配置文件中调整。

# 运行程序

打开终端，执行：

```powershell
cd <项目根目录>
uv run track_fire.py
```

`uv run` 会自动使用 `.venv` 中的环境运行脚本，无需手动激活虚拟环境。若依赖尚未同步，uv 会先自动执行 `uv sync`。

## 运行后操作

主程序会弹出摄像头窗口，自动按顺序执行 `TASKS` 列表中的任务：舵机转到任务指定角度 → 云台跟踪指定编号的 AprilTag 直到到达期望位置 → 执行动作（激光 / 发射）。**在摄像头窗口中（不是终端）**按键：

- 按 **Q** 或 **Esc** → 退出（未执行的任务将被跳过）

# 下位机固件（控制板 Arduino 程序）

上位机 Python 代码通过串口控制控制板，控制板上运行的固件位于 `firmware/guard_robot/guard_robot.ino`（Arduino UNO）。使用前需要先把固件烧录到控制板。

## 烧录固件

1. 安装 [Arduino IDE](https://www.arduino.cc/en/software)。
2. 用 USB 线连接控制板，在 IDE 中打开 `firmware/guard_robot/guard_robot.ino`。
3. 选择板卡 **Arduino UNO** 和对应的端口，点击"上传"。
4. 烧录成功后，控制板以 **115200** 波特率与上位机通信（与 `board.py` 的默认值一致）。

## 串口命令协议

固件通过文本串口协议接收命令，每条命令以换行符结尾：

| 命令                 | 说明                       |
| ------------------ | ------------------------ |
| `@S<角度>`           | 舵机转到指定角度（0～180）          |
| `@MA<速度>`          | 设置 A 电机速度（-255～255，0 停止） |
| `@MB<速度>`          | 设置 B 电机速度（-255～255，0 停止） |
| `@MTA<速度>,<毫秒>`    | A 电机定时运行，到时自动停止          |
| `@MTB<速度>,<毫秒>`    | B 电机定时运行，到时自动停止          |
| `@FS`              | 发射一次                     |
| `@ZZ`              | 装填一次                     |
| `@JGON` / `@JGOFF` | 打开 / 关闭激光                |

## 引脚对应

| 引脚    | 用途               |
| ----- | ---------------- |
| D2    | 舵机（俯仰）           |
| D3    | 发射机构位置传感器        |
| D4、D5 | 发射电机（A 电机）       |
| D6、D7 | 云台电机（B 电机）       |
| D9    | 按键A：启停云台电机（B 电机） |
| D10   | 按键B：启停发射电机（A 电机） |
| D11   | 激光发射器            |
| D12   | 无源蜂鸣器            |
| D13   | 手动发射键            |

# 上位机示例程序

项目根目录下提供了多个独立示例，可以先分别验证摄像头、AprilTag 检测和控制板，再运行主程序。

## 1. 测试摄像头

```powershell
uv run test_camera.py
```

打开实时画面，在画面窗口按 **Q** 或 **Esc** 退出。

## 2. 测试 AprilTag 检测

```powershell
uv run test_detector.py
```

程序会显示检测到的标签边框、ID 以及相对画面中心的坐标。在画面窗口按 **Q** 或 **Esc** 退出。

## 3. 测试控制板

```powershell
uv run test_board.py
```

也可以把串口作为第一个参数传入：

```powershell
uv run test_board.py COM3
```

控制板示例按键：

- `W` / `S`：舵机上下转动（角度范围 60～120）
- `A` / `D`：云台左右转动
- `O` / `P`：打开 / 关闭激光
- `F` / `L`：发射 / 装填
- `0`：停止全部电机
- `1` / `2`：电机 A 正转 / 反转 500 毫秒
- `3` / `4`：电机 B 正转 / 反转 500 毫秒
- `6` / `9`：舵机转到 60 / 90 度
- `Q`：退出

其中 `F`、`L`、`O`、电机和激光按键会操作真实硬件，请确认设备处于安全状态后再使用。

## 4. 云台跟踪（track.py）

```powershell
uv run track.py
```

开关式跟踪：发现 AprilTag 后，云台按**固定速度**转动——标签偏右就右转、偏左就左转，标签进入画面中心范围（`CENTER_HALF_WIDTH`）就停止；没发现标签则云台停住。转速固定，不随偏差变化，逻辑最简单，适合先理解"让云台自己追标签"的基本思路。

## 5. PID 比例跟踪（track\_pid.py）

```powershell
uv run track_pid.py
```

PID 比例控制（P 控制）：同样是发现标签后云台跟踪，但转速与偏差成正比——**偏差越大转得越快，越靠近中心转得越慢**，对准后停止。比开关式更平滑，不易在中心附近来回摆动。可调参数都在文件顶部：

- `KP`：比例系数（偏差每 1 像素对应的转速，越大越灵敏）
- `MAX_SPEED` / `MIN_SPEED`：速度限幅（太快会转过头，太慢电机带不动）
- `CENTER_HALF_WIDTH`：居中范围（标签进入该范围即停止）

> 注意：`track.py` / `track_pid.py` 都会操作真实硬件（云台电机 B），运行时请确认设备处于安全状态。两个程序都弹出摄像头窗口，按 **Q** 或 **Esc** 退出。

## 6. 多任务跟踪发射（track\_fire.py）

```powershell
uv run track_fire.py
```

按顺序执行多个任务：每个任务对应一个指定编号的 AprilTag。先把舵机（俯仰）转到任务指定角度，随后云台跟踪该标签直到到达期望位置（`offset` 指定的画面位置），最后执行动作（发射激光或发射一次），再进入下一个任务；全部完成后程序结束。

任务在文件顶部的 `TASKS` 列表中配置，每条包含四个字段：

- `tag`：要跟踪的标签编号
- `offset`：标签期望位置，相对画面中心的像素偏移（右正左负，0 即画面中心）
- `angle`：跟踪该标签时舵机的俯仰角度（0～180）
- `action`：标签到位后执行的动作，`"laser"` 发射激光，`"fire"` 发射一次

默认按 tag 2 → 6 → 4 的顺序执行：

```python
TASKS = [
    {"tag": 2, "offset": -20, "angle": 102, "action": "laser"},
    {"tag": 6, "offset": -24, "angle": 86, "action": "laser"},
    {"tag": 4, "offset": -20, "angle": 100, "action": "fire"},
]
```

标签附近找圆对准（默认开启）：`track_fire.py` 顶部的 `AIM_CIRCLE_NEAR_TAG = True` 时，标签任务在锁定标签后会自动在标签附近（圆心距离不超过标签对角线的 2.5 倍、至少 100 像素）找圆，把圆心对准到 `offset` 位置后才执行动作；标签附近暂时没有圆时，云台原地等待圆出现。某个任务不需要找圆时，可在该任务里加 `"aim_circle": False`。

> 注意：程序会操作真实硬件（云台电机 B、舵机、激光和发射机构），运行时请确认设备处于安全状态。程序弹出摄像头窗口，按 **Q** 或 **Esc** 退出（退出后未执行的任务将被跳过）。

