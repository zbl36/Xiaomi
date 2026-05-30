"""
统一视觉识别模块
支持检测：可乐瓶(coke)、足球(football)、立方体障碍物(cube)、限高杆(ganzi)、蓝色小球(blue_ball)、橘色小球(orange_ball)
"""

import cv2
import time
from pathlib import Path
from ultralytics import YOLO

# ---------- 配置参数 ----------
CONF_THRESHOLD = 0.6
INFERENCE_INTERVAL = 0.1

MODULE_DIR = Path(__file__).resolve().parent
COKE_MODEL_PATH = str(MODULE_DIR / "coke_best.pt")
FOOTBALL_MODEL_PATH = str(MODULE_DIR / "football_best.pt")
CUBE_MODEL_PATH = str(MODULE_DIR / "cube_best.pt")
GANZI_MODEL_PATH = str(MODULE_DIR / "ganzi_best.pt")
BLUE_BALL_MODEL_PATH = str(MODULE_DIR / "blue_ball_best.pt")
ORANGE_BALL_MODEL_PATH = str(MODULE_DIR / "orange_ball_best.pt")
# ------------------------------

class VisionDetector:
    def __init__(self, enable_coke=True, enable_football=True, enable_cube=True,
                 enable_ganzi=True, enable_blue_ball=True, enable_orange_ball=True):
        self.enable_coke = enable_coke
        self.enable_football = enable_football
        self.enable_cube = enable_cube
        self.enable_ganzi = enable_ganzi
        self.enable_blue_ball = enable_blue_ball
        self.enable_orange_ball = enable_orange_ball

        self.coke_model = None
        self.football_model = None
        self.cube_model = None
        self.ganzi_model = None
        self.blue_ball_model = None
        self.orange_ball_model = None

        self._last_coke_time = 0
        self._last_football_time = 0
        self._last_cube_time = 0
        self._last_ganzi_time = 0
        self._last_blue_ball_time = 0
        self._last_orange_ball_time = 0

        if enable_coke:
            self.coke_model = self._load_model(COKE_MODEL_PATH, "coke")
        if enable_football:
            self.football_model = self._load_model(FOOTBALL_MODEL_PATH, "football")
        if enable_cube:
            self.cube_model = self._load_model(CUBE_MODEL_PATH, "cube")
        if enable_ganzi:
            self.ganzi_model = self._load_model(GANZI_MODEL_PATH, "ganzi")
        if enable_blue_ball:
            self.blue_ball_model = self._load_model(BLUE_BALL_MODEL_PATH, "blue_ball")
        if enable_orange_ball:
            self.orange_ball_model = self._load_model(ORANGE_BALL_MODEL_PATH, "orange_ball")

    def _load_model(self, path, name):
        try:
            model = YOLO(path)
            print(f"[Vision] {name} 模型加载成功: {path}")
            return model
        except Exception as e:
            print(f"[Vision] {name} 模型加载失败: {e}")
            return None

    def _inference(self, model, frame, conf=CONF_THRESHOLD):
        if model is None:
            return []
        results = model(frame, verbose=False, conf=conf, iou=0.45)
        if not results or results[0].boxes is None:
            return []
        detections = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf_val = float(box.conf[0])
            detections.append([x1, y1, x2, y2, conf_val])
        return detections

    def detect_coke(self, frame):
        if self.coke_model is None:
            return []
        now = time.time()
        if now - self._last_coke_time < INFERENCE_INTERVAL:
            return []
        self._last_coke_time = now
        return self._inference(self.coke_model, frame)

    def detect_football(self, frame):
        if self.football_model is None:
            return []
        now = time.time()
        if now - self._last_football_time < INFERENCE_INTERVAL:
            return []
        self._last_football_time = now
        return self._inference(self.football_model, frame)

    def detect_cube(self, frame):
        """检测立方体障碍物"""
        if self.cube_model is None:
            return []
        now = time.time()
        if now - self._last_cube_time < INFERENCE_INTERVAL:
            return []
        self._last_cube_time = now
        return self._inference(self.cube_model, frame)

    def detect_ganzi(self, frame):
        """检测限高杆"""
        if self.ganzi_model is None:
            return []
        now = time.time()
        if now - self._last_ganzi_time < INFERENCE_INTERVAL:
            return []
        self._last_ganzi_time = now
        return self._inference(self.ganzi_model, frame)

    def detect_blue_ball(self, frame):
        """检测蓝色小球"""
        if self.blue_ball_model is None:
            return []
        now = time.time()
        if now - self._last_blue_ball_time < INFERENCE_INTERVAL:
            return []
        self._last_blue_ball_time = now
        return self._inference(self.blue_ball_model, frame)

    def detect_blue(self, frame):
        """兼容旧接口：检测蓝色小球"""
        return self.detect_blue_ball(frame)

    def detect_orange_ball(self, frame):
        """检测橘色小球"""
        if self.orange_ball_model is None:
            return []
        now = time.time()
        if now - self._last_orange_ball_time < INFERENCE_INTERVAL:
            return []
        self._last_orange_ball_time = now
        return self._inference(self.orange_ball_model, frame)

    def detect_orange(self, frame):
        """兼容旧接口：检测橘色小球"""
        return self.detect_orange_ball(frame)

    def detect_all(self, frame):
        result = {}
        if self.enable_coke:
            result['coke'] = self.detect_coke(frame)
        if self.enable_football:
            result['football'] = self.detect_football(frame)
        if self.enable_cube:
            result['cube'] = self.detect_cube(frame)
        if self.enable_ganzi:
            result['ganzi'] = self.detect_ganzi(frame)
        if self.enable_blue_ball:
            result['blue_ball'] = self.detect_blue_ball(frame)
        if self.enable_orange_ball:
            result['orange_ball'] = self.detect_orange_ball(frame)
        return result

    def get_closest_target(self, frame, target_type='coke', area_threshold=0):
        func = getattr(self, f'detect_{target_type}', None)
        if func is None:
            return None
        dets = func(frame)
        if not dets:
            return None
        best = max(dets, key=lambda d: (d[2]-d[0])*(d[3]-d[1]))
        x1, y1, x2, y2, conf = best
        area = (x2 - x1) * (y2 - y1)
        if area < area_threshold:
            return None
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        return (cx, cy, area, conf)

    @staticmethod
    def draw_detections(frame, detections, color=(0,255,0), label=None):
        for det in detections:
            x1, y1, x2, y2, conf = det
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            text = f"{label} {conf:.2f}" if label else f"{conf:.2f}"
            cv2.putText(frame, text, (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return frame


_detector = None

def get_detector(enable_coke=True, enable_football=True, enable_cube=True,
                 enable_ganzi=True, enable_blue_ball=True,
                 enable_orange_ball=True):
    global _detector
    if _detector is None:
        _detector = VisionDetector(enable_coke, enable_football, enable_cube,
                                   enable_ganzi, enable_blue_ball,
                                   enable_orange_ball)
    return _detector

def detect_objects(frame, target='all', return_dict=False):
    det = get_detector()
    if target == 'all':
        results = det.detect_all(frame)
        if return_dict:
            return results
        else:
            all_dets = []
            for cls, dets in results.items():
                all_dets.extend(dets)
            return all_dets
    else:
        func = getattr(det, f'detect_{target}', None)
        if func:
            return func(frame)
        else:
            raise ValueError(f"Unknown target: {target}")
