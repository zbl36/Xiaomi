#!/usr/bin/env python3
"""
相机录制工具
订阅 /rgb_camera/image_raw，录制为 mp4 视频，并可拆分成帧图片

用法：
    python3.8 record_video.py              # 录制视频
    python3.8 record_video.py --split      # 录制完自动拆帧
    python3.8 record_video.py --fps 15     # 指定帧率（默认15）

保存路径：/home/cyberdog_sim/recordings/
"""

import cv2
import numpy as np
import os
import time
import threading
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

SAVE_DIR = '/home/cyberdog_sim/recordings'


class VideoRecorder(Node):
    def __init__(self, fps=15):
        super().__init__('video_recorder')
        self._frame = None
        self._lock = threading.Lock()
        self._fps = fps

        os.makedirs(SAVE_DIR, exist_ok=True)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(Image, '/rgb_camera/image_raw', self._cb, qos)

    def _cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape((msg.height, msg.width, -1))
        if msg.encoding in ('rgb8', 'RGB8'):
            arr = arr[:, :, ::-1].copy()
        with self._lock:
            self._frame = arr

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def record(self, label='', auto_split=False):
        ts = time.strftime('%Y%m%d_%H%M%S')
        name = f'{label}_{ts}' if label else f'video_{ts}'
        video_path = os.path.join(SAVE_DIR, f'{name}.mp4')

        writer = None
        frame_count = 0

        print(f'\n开始录制：{video_path}')
        print('按 Ctrl+C 停止录制\n')

        try:
            while True:
                frame = self.get_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue

                if writer is None:
                    h, w = frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    writer = cv2.VideoWriter(video_path, fourcc, self._fps, (w, h))
                    print(f'分辨率：{w}x{h}，帧率：{self._fps} fps')

                writer.write(frame)
                frame_count += 1

                if frame_count % self._fps == 0:
                    print(f'\r已录制 {frame_count // self._fps} 秒 / {frame_count} 帧', end='', flush=True)

                time.sleep(1.0 / self._fps)

        except KeyboardInterrupt:
            pass
        finally:
            if writer:
                writer.release()
            print(f'\n\n录制完成：{video_path}')
            print(f'共 {frame_count} 帧，约 {frame_count / self._fps:.1f} 秒')

            if auto_split and frame_count > 0:
                self.split_video(video_path, name)

        return video_path

    def split_video(self, video_path, name, interval=1):
        """
        将视频拆分为帧图片
        interval: 每隔几帧保存一张（默认每帧都保存）
        """
        frames_dir = os.path.join(SAVE_DIR, f'{name}_frames')
        os.makedirs(frames_dir, exist_ok=True)

        cap = cv2.VideoCapture(video_path)
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        saved = 0
        idx = 0

        print(f'\n开始拆帧：{frames_dir}')
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if idx % interval == 0:
                path = os.path.join(frames_dir, f'frame_{idx:05d}.jpg')
                cv2.imwrite(path, frame)
                saved += 1
            idx += 1

        cap.release()
        print(f'拆帧完成：共保存 {saved} 张图片到 {frames_dir}')
        return frames_dir


def main():
    parser = argparse.ArgumentParser(description='相机录制工具')
    parser.add_argument('--split', action='store_true', help='录制完自动拆帧')
    parser.add_argument('--fps', type=int, default=15, help='录制帧率（默认15）')
    parser.add_argument('--label', type=str, default='', help='视频标签（如 coke, football）')
    parser.add_argument('--split-only', type=str, default='', help='只拆帧，传入视频路径')
    args = parser.parse_args()

    # 只拆帧模式
    if args.split_only:
        rclpy.init()
        node = VideoRecorder()
        name = os.path.splitext(os.path.basename(args.split_only))[0]
        node.split_video(args.split_only, name)
        return

    rclpy.init()
    node = VideoRecorder(fps=args.fps)

    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # 等待第一帧
    print('等待相机图像...')
    for _ in range(50):
        if node.get_frame() is not None:
            break
        time.sleep(0.1)

    print('\n录制工具已启动')
    print(f'标签：{args.label or "无"}')
    print(f'帧率：{args.fps} fps')
    print(f'自动拆帧：{"是" if args.split else "否"}')

    try:
        node.record(label=args.label, auto_split=args.split)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
