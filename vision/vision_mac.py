"""摄像头和 AprilTag 检测功能。

使用摄像头识别 AprilTag 三步：
    1. detector = create_detector()               创建检测器
    2. camera   = open_camera()                   打开摄像头
    3. frame, tags = read_tags(camera, detector)  读画面并识别标签
其他函数（画图、按键、流程控制）是辅助功能。
"""
import sys

import cv2
import numpy as np
from pupil_apriltags import Detector
from config import CAMERA_INDEX

# ============================== 摄像头参数 ==============================
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 60
CAMERA_AUTO_EXPOSURE = 0.75

# ============================== 暗光 / 画质增强 ==============================
# 1) 检测去噪：暗光下摄像头噪点（含椒盐亮点）很多，先做 3x3 中值滤波，
#    能在不糊掉标签黑白硬边的前提下大幅压噪，再交给后面的均衡和锐化。
ENABLE_DETECTION_DENOISING = True
DENOISE_KSIZE = 3             # 中值滤波核大小，只能取奇数（3/5）；5 更狠但会糊边

# 2) 检测增强：对送入 AprilTag 检测器的灰度图做 CLAHE 局部对比度均衡，
#    暗光下能明显提升标签识别率，不影响窗口中显示的彩色画面。
ENABLE_DETECTION_CLAHE = True
CLAHE_CLIP_LIMIT = 2.0          # 对比度增强强度（越大越强烈，过大会放大噪点）

# 3) 显示提亮：画面平均亮度低于阈值时，对显示画面做伽马提亮，
#    只用于让人看得清，不参与标签检测；亮度正常的画面不做任何处理。
ENABLE_DISPLAY_BRIGHTENING = True
BRIGHTENING_TRIGGER_MEAN = 90  # 灰度均值低于该值（0～255）才提亮
DISPLAY_GAMMA = 1.7            # 伽马值越大画面越亮（1.0 为不处理）

# 显示降噪：提亮会同时放大暗部噪点，可在伽马提亮前对显示画面做一次轻中值滤波。
# 默认关闭，由需要的程序（如 track_fire）在本进程内打开，不影响其他程序。
ENABLE_DISPLAY_DENOISE = False
DISPLAY_DENOISE_KSIZE = 3      # 显示降噪中值核大小（奇数）

# 4) 开机自检：打开摄像头后读取几帧，报告实际分辨率、亮度、清晰度，
#    并在分辨率不足（画面会被放大而模糊）或画面过暗时给出提示。
ENABLE_STARTUP_DIAGNOSTICS = True
DARK_FRAME_MEAN = 60           # 平均亮度低于该值时提示“画面偏暗”


def _camera_backend():
    """按操作系统选择摄像头采集后端。

    Windows 用 DirectShow，macOS 用 AVFoundation，Linux 用 V4L2；
    找不到对应后端常量时返回 None，交给 OpenCV 使用默认后端。
    """

    if sys.platform.startswith("win"):
        return getattr(cv2, "CAP_DSHOW", None)
    if sys.platform == "darwin":
        return getattr(cv2, "CAP_AVFOUNDATION", None)
    if sys.platform.startswith("linux"):
        return getattr(cv2, "CAP_V4L2", None)
    return None


CAMERA_BACKEND = _camera_backend()
# 各后端的名称，仅用于错误提示
CAMERA_BACKEND_NAMES = {
    "win32": "DirectShow",
    "darwin": "AVFoundation",
    "linux": "V4L2",
}

# 锐化仅作用于送入检测器的灰度图，不会改变窗口中显示的彩色图像。
# 暗光噪点大，锐化过强会把噪点也放大成假边缘，强度取 0.6 的轻度锐化即可。
ENABLE_DETECTION_SHARPENING = True
SHARPEN_SIGMA = 1.2
SHARPEN_AMOUNT = 0.6

# 复用对象，避免逐帧重复创建
_clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=(8, 8))
_gamma_table = np.array(
    [((i / 255.0) ** (1.0 / DISPLAY_GAMMA)) * 255 for i in range(256)],
    dtype=np.uint8,
)

WINDOW_NAME = "AprilTag Detector"
QUIT_KEYS = (ord("q"), ord("Q"), 27)  # q 或 Q 或 Esc 键

# ====================== 一、准备工作：摄像头与检测器 ======================

def open_camera():
    """打开并配置摄像头。

    输入：无
    输出：配置好的摄像头对象；打开失败时抛出 RuntimeError。
    """

    # 按索引打开摄像头；不同系统使用各自的采集后端
    if CAMERA_BACKEND is not None:
        camera = cv2.VideoCapture(CAMERA_INDEX, CAMERA_BACKEND)
    else:
        camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        backend_name = CAMERA_BACKEND_NAMES.get(sys.platform, "系统默认")
        raise RuntimeError(
            f"无法通过 {backend_name} 打开摄像头，请检查 CAMERA_INDEX 设置是否正确，"
            "并确认没有其他程序占用摄像头。"
            "（macOS 还需在“系统设置 → 隐私与安全性 → 摄像头”中允许终端访问摄像头）"
        )
    configure_camera(camera)  # 设置分辨率、帧率等参数
    if ENABLE_STARTUP_DIAGNOSTICS:
        _run_camera_diagnostics(camera)
    return camera

def configure_camera(camera):
    """配置摄像头参数（分辨率、帧率、自动曝光、自动对焦）。

    输入：camera 摄像头对象
    输出：无。并非所有摄像头/后端都支持这些参数，设置失败无需报错
         （例如 macOS 的 AVFoundation 不支持曝光调节，会自动忽略）。
    """

    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # 视频编码格式
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)   # 画面宽度
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)  # 画面高度
    camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)             # 帧率
    camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, CAMERA_AUTO_EXPOSURE)  # 自动曝光
    # 尝试开启连续自动对焦（支持对焦的摄像头在 Windows 上有效；不支持时忽略）
    camera.set(cv2.CAP_PROP_AUTOFOCUS, 1)


def _run_camera_diagnostics(camera):
    """打开摄像头后读取几帧，报告实际分辨率、亮度和清晰度。

    分辨率小于设置值时画面会被拉伸放大而模糊；亮度过低时提示补光。
    输入：camera 摄像头对象
    输出：无（仅在终端打印信息和警告）
    """

    frame = None
    for _ in range(10):  # 丢弃前几帧，等待曝光稳定
        success, candidate = camera.read()
        if success:
            frame = candidate
    if frame is None:
        return

    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    brightness = gray.mean()
    sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
    print(f"摄像头实际分辨率：{width}x{height}，平均亮度：{brightness:.0f}，清晰度：{sharpness:.0f}")

    if width < CAMERA_WIDTH or height < CAMERA_HEIGHT:
        print(
            f"  警告：摄像头实际输出 {width}x{height}，低于设置的 "
            f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}，画面会被放大而显得模糊。"
            "请更换支持 720p 的 USB 接口/摄像头，或调低 CAMERA_WIDTH/HEIGHT。"
        )
    if brightness < DARK_FRAME_MEAN:
        print(
            "  警告：画面偏暗。请增加环境灯光、检查镜头保护膜是否撕除；"
            "程序已对显示画面做伽马提亮、对检测画面做对比度增强。"
        )

# AprilTag 检测器参数（暗光/模糊画面调优）
DETECTOR_NTHREADS = 2
DETECTOR_QUAD_DECIMATE = 1.0   # 1.0=全分辨率检测，标签小/距离远时可试 1.5
DETECTOR_QUAD_SIGMA = 0.8      # 检测前轻度高斯模糊，抑制暗光放大后的噪点（0.0=不模糊）
DETECTOR_DECODE_SHARPENING = 0.4  # 解码锐化，画面模糊时从默认 0.25 提高到 0.4


def create_detector():
    """创建 AprilTag 检测器。

    输入：无
    输出：用于识别 tag36h11 家族标签的检测器对象。
    """

    return Detector(  # tag36h11 家族，2 线程检测
        families="tag36h11",
        nthreads=DETECTOR_NTHREADS,
        quad_decimate=DETECTOR_QUAD_DECIMATE,
        quad_sigma=DETECTOR_QUAD_SIGMA,
        refine_edges=1,
        decode_sharpening=DETECTOR_DECODE_SHARPENING,
        debug=0,
    )

# ====================== 二、识别标签：读画面并找出 AprilTag ======================

def read_tags(camera, detector):
    """读取一帧画面并识别其中的所有 AprilTag。

    输入：camera 摄像头对象；detector 检测器对象
    输出：frame 当前帧图像；tags 识别到的标签列表（可能为空）
    """

    success, frame = camera.read()  # 读取一帧
    if not success:
        return None, []  # 读取失败，返回空结果

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转灰度图
    detection_image = gray

    if ENABLE_DETECTION_DENOISING:  # 中值去噪：压暗光噪点和椒盐亮点，保留标签硬边
        detection_image = cv2.medianBlur(detection_image, DENOISE_KSIZE)

    if ENABLE_DETECTION_CLAHE:  # 局部对比度均衡：暗光下提升标签黑白边界
        detection_image = _clahe.apply(detection_image)

    if ENABLE_DETECTION_SHARPENING:  # 锐化仅用于检测，不改变显示画面
        blurred = cv2.GaussianBlur(detection_image, (0, 0), SHARPEN_SIGMA)
        detection_image = cv2.addWeighted(
            detection_image,
            1.0 + SHARPEN_AMOUNT,
            blurred,
            -SHARPEN_AMOUNT,
            0,
        )

    return frame, detector.detect(detection_image)  # 返回画面和识别结果


# ================== 三、画图与显示（先看整体，再拆细节） ==================

def _brighten_frame(frame):
    """画面偏暗时做降噪 + 伽马提亮，亮度正常则原样返回。

    只作用于窗口显示，不参与标签检测。
    输入：frame 原始画面
    输出：提亮后的画面（或原画面）
    """

    if not ENABLE_DISPLAY_BRIGHTENING:
        return frame

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if gray.mean() >= BRIGHTENING_TRIGGER_MEAN:
        return frame
    image = frame
    if ENABLE_DISPLAY_DENOISE:  # 先压噪再提亮，避免伽马把暗部噪点放大
        image = cv2.medianBlur(image, DISPLAY_DENOISE_KSIZE)
    return cv2.LUT(image, _gamma_table)


def show_frame(frame, tags, status):
    """在画面上绘制识别结果并显示。

    输入：frame 画面；tags 标签列表；status 左上角提示文字
    输出：无（画面直接显示在窗口里）
    """

    frame = _brighten_frame(frame)  # 暗光下先提亮显示画面
    height, width = frame.shape[:2]
    center_x = width // 2   # 画面中心 X 坐标
    center_y = height // 2  # 画面中心 Y 坐标

    draw_center_cross(frame, center_x, center_y)  # 画中心十字线
    for tag in tags:
        draw_tag(frame, tag, center_x, center_y)  # 画每个标签

    cv2.putText(
        frame,
        status,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2,
    )  # 左上角提示文字
    cv2.imshow(WINDOW_NAME, frame)  # 刷新并显示画面


def draw_tag(frame, tag, frame_center_x, frame_center_y):
    """在画面上绘制一个标签（边框、中心点、ID 和坐标）。

    输入：frame 画面；tag 标签对象；frame_center_x/frame_center_y 画面中心坐标
    输出：无（直接在 frame 上绘制）
    """

    corners = tag.corners.astype(int)
    for index in range(4):  # 依次连接四个角点，画标签边框
        cv2.line(
            frame,
            tuple(corners[index]),
            tuple(corners[(index + 1) % 4]),
            (0, 255, 0),
            2,
        )

    center_x, center_y = tag.center
    relative_x = center_x - frame_center_x  # 标签相对画面中心的 X
    relative_y = center_y - frame_center_y  # 标签相对画面中心的 Y
    center = (int(center_x), int(center_y))

    cv2.circle(frame, center, 5, (0, 0, 255), -1)  # 画标签中心点
    cv2.putText(
        frame,
        f"ID:{tag.tag_id} X:{relative_x:.0f} Y:{relative_y:.0f}",
        (center[0] + 10, center[1] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2,
    )  # 标签旁写 ID 和坐标


def draw_center_cross(frame, center_x, center_y):
    """在画面上画中心十字线。

    输入：frame 画面；center_x/center_y 画面中心坐标
    输出：无（直接在 frame 上绘制）
    """

    cv2.line(frame, (center_x, 0), (center_x, frame.shape[0]), (255, 0, 0), 2)  # 竖线
    cv2.line(frame, (0, center_y), (frame.shape[1], center_y), (255, 0, 0), 2)  # 横线


# ====================== 四、按键处理 ======================

def wait_key(delay=1):
    """等待按键并返回键值。

    输入：delay 等待毫秒数（默认 1），窗口画面靠它刷新
    输出：按键编码；无按键时为 -1
    """

    return cv2.waitKey(delay) & 0xFF  # 取低 8 位，消除系统位的影响


def is_quit_key(key):
    """判断按键是否为退出键（Q 或 Esc）。

    输入：key 键值
    输出：True 表示退出键；False 表示不是
    """

    return key in QUIT_KEYS  # 在退出键集合中查找


def is_window_closed(window_name=WINDOW_NAME):
    """检测画面窗口是否已被关闭（例如点击了窗口左上角红色关闭按钮）。

    窗口被关闭后 waitKey 无法再收到按键，必须靠本函数退出循环，
    否则程序会脱离按键控制持续运行。
    输入：window_name 窗口名称
    输出：True 表示窗口不存在或已关闭
    """

    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


# ====================== 五、流程控制：按 Y 开始 / 继续 ======================

def wait_for_start(camera, detector):
    """显示识别画面，等待用户按 Y 开始或按 Q/Esc 取消。

    输入：camera 摄像头对象；detector 检测器对象
    输出：True 表示开始任务；False 表示用户取消
    """

    print("视觉识别已启动。请在摄像头窗口按 Y 开始任务。")

    while True:
        frame, tags = read_tags(camera, detector)  # 读一帧并识别
        if frame is None:
            continue  # 读取失败，跳过本帧

        show_frame(frame, tags, "Vision ready - press Y to start")
        key = wait_key()

        if key in (ord("y"), ord("Y")):  # 按 Y 开始
            print("收到 Y，开始执行任务。")
            return True
        if is_quit_key(key):  # 按 Q/Esc 取消
            print("用户取消任务。")
            return False

# ====================== 六、收尾：关闭窗口 ======================

def close_windows():
    """关闭所有识别窗口。

    输入：无
    输出：无
    """

    cv2.destroyAllWindows()
