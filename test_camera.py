"""摄像头示例：打开摄像头并显示实时画面。"""

import cv2

from vision import vision


def main():
    camera = vision.open_camera()

    try:
        print("摄像头已打开，请在画面窗口按 Q 或 Esc 退出。")
        while True:
            success, frame = camera.read()
            if not success:
                print("读取摄像头画面失败。")
                continue

            cv2.imshow("Camera Test", frame)
            if vision.is_quit_key(vision.wait_key()):
                break
    finally:
        camera.release()
        vision.close_windows()


if __name__ == "__main__":
    main()
