"""摄像头和 AprilTag 检测功能。

使用摄像头识别 AprilTag 三步：
    1. detector = create_detector()               创建检测器
    2. camera   = open_camera()                   打开摄像头
    3. frame, tags = read_tags(camera, detector)  读画面并识别标签
其他函数（画图、按键、流程控制）是辅助功能。
"""
import cv2
from pupil_apriltags import Detector
from config import CAMERA_INDEX

# ============================== 摄像头参数 ==============================
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 60
CAMERA_AUTO_EXPOSURE = 0.75

# 锐化仅作用于送入检测器的灰度图，不会改变窗口中显示的彩色图像。
ENABLE_DETECTION_SHARPENING = True
SHARPEN_SIGMA = 1.2
SHARPEN_AMOUNT = 1.0

WINDOW_NAME = "AprilTag Detector"
QUIT_KEYS = (ord("q"), ord("Q"), 27)  # q 或 Q 或 Esc 键

# ==================== 圆检测参数（HoughCircles，来自 simpleCV） ====================
CIRCLE_DP = 1            # 累加器分辨率，一般取 1
CIRCLE_MIN_DIST = 80     # 两个圆心的最小距离，太小会把同一个圆当成好几个
CIRCLE_PARAM1 = 100      # Canny 边缘检测阈值
CIRCLE_PARAM2 = 54       # 累加器阈值，用 simpleCV/camera_circle.py 实测标定后固定
CIRCLE_MIN_RADIUS = 25   # 允许的最小半径
CIRCLE_MAX_RADIUS = 150  # 允许的最大半径

# ====================== 一、准备工作：摄像头与检测器 ======================

def open_camera():
    """打开并配置摄像头。

    输入：无
    输出：配置好的摄像头对象；打开失败时抛出 RuntimeError。
    """

    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)  # 按索引打开摄像头
    if not camera.isOpened():
        raise RuntimeError(
            "无法通过 DirectShow 打开摄像头，请检查 CAMERA_INDEX 设置是否正确。"
            "并确认没有其他程序占用摄像头。"
        )
    configure_camera(camera)  # 设置分辨率、帧率等参数
    return camera

def configure_camera(camera):
    """配置摄像头参数（分辨率、帧率、自动曝光）。

    输入：camera 摄像头对象
    输出：无
    """

    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # 视频编码格式
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)   # 画面宽度
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)  # 画面高度
    camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)             # 帧率
    camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, CAMERA_AUTO_EXPOSURE)  # 自动曝光

def create_detector():
    """创建 AprilTag 检测器。

    输入：无
    输出：用于识别 tag36h11 家族标签的检测器对象。
    """

    return Detector(  # tag36h11 家族，2 线程检测
        families="tag36h11",
        nthreads=2,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
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

    if ENABLE_DETECTION_SHARPENING:  # 锐化仅用于检测，不改变显示画面
        blurred = cv2.GaussianBlur(gray, (0, 0), SHARPEN_SIGMA)
        detection_image = cv2.addWeighted(
            gray,
            1.0 + SHARPEN_AMOUNT,
            blurred,
            -SHARPEN_AMOUNT,
            0,
        )

    return frame, detector.detect(detection_image)  # 返回画面和识别结果


def read_frame(camera):
    """只读取一帧画面，不做 AprilTag 识别（找圆任务用）。

    输入：camera 摄像头对象
    输出：frame 当前帧图像；读取失败时返回 None
    """

    success, frame = camera.read()
    if not success:
        return None
    return frame


def detect_circles(frame):
    """检测画面中的圆（HoughCircles 圆变换）。

    输入：frame BGR 画面
    输出：圆列表 [(x, y, r), ...]；没检测到时返回空列表
    """

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # HoughCircles 只接受灰度图
    gray = cv2.medianBlur(gray, 5)                  # 中值滤波，抑制噪点

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=CIRCLE_DP,
        minDist=CIRCLE_MIN_DIST,
        param1=CIRCLE_PARAM1,
        param2=CIRCLE_PARAM2,
        minRadius=CIRCLE_MIN_RADIUS,
        maxRadius=CIRCLE_MAX_RADIUS,
    )
    if circles is None:
        return []

    return [(float(x), float(y), float(r)) for x, y, r in circles[0]]


# ================== 三、画图与显示（先看整体，再拆细节） ==================

def show_frame(frame, tags, status, circles=None):
    """在画面上绘制识别结果并显示。

    输入：frame 画面；tags 标签列表；status 左上角提示文字；
         circles 圆列表（可选，找圆任务使用）
    输出：无（画面直接显示在窗口里）
    """

    height, width = frame.shape[:2]
    center_x = width // 2   # 画面中心 X 坐标
    center_y = height // 2  # 画面中心 Y 坐标

    draw_center_cross(frame, center_x, center_y)  # 画中心十字线
    for tag in tags:
        draw_tag(frame, tag, center_x, center_y)  # 画每个标签
    for circle in circles or []:
        draw_circle(frame, circle, center_x, center_y)  # 画每个圆

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


def draw_circle(frame, circle, frame_center_x, frame_center_y):
    """在画面上绘制一个圆（外圈、圆心和坐标）。

    输入：frame 画面；circle 圆 (x, y, r)；
         frame_center_x/frame_center_y 画面中心坐标
    输出：无（直接在 frame 上绘制）
    """

    x, y, r = circle
    relative_x = x - frame_center_x  # 圆心相对画面中心的 X
    relative_y = y - frame_center_y  # 圆心相对画面中心的 Y
    center = (int(x), int(y))

    cv2.circle(frame, center, int(r), (0, 255, 0), 2)  # 绿色外圈
    cv2.circle(frame, center, 4, (0, 0, 255), -1)      # 红色圆心点
    cv2.putText(
        frame,
        f"R:{r:.0f} X:{relative_x:.0f} Y:{relative_y:.0f}",
        (center[0] + 10, center[1] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )  # 圆旁写半径和坐标


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
