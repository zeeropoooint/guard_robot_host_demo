"""AprilTag 示例：检测标签并在窗口中显示识别结果。"""

from vision import vision


def main():
    detector = vision.create_detector()
    camera = vision.open_camera()

    try:
        print("AprilTag 检测已启动，请在画面窗口按 Q 或 Esc 退出。")
        while True:
            frame, tags = vision.read_tags(camera, detector)
            if frame is None:
                continue

            tag_ids = ", ".join(str(tag.tag_id) for tag in tags) or "无"
            status = f"检测到 {len(tags)} 个标签，ID: {tag_ids}"
            vision.show_frame(frame, tags, status)

            if vision.is_quit_key(vision.wait_key()):
                break
    finally:
        camera.release()
        vision.close_windows()


if __name__ == "__main__":
    main()
