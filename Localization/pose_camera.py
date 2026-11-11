"""
pose_camera.py — получение позы робота (x, y, theta) с потолочной камеры по AprilTag.

Подключение к Kalibrovka.py:
    1. В Kalibrovka.py замените блок get_pose на:

        from pose_camera import get_pose, start_camera, stop_camera
        # ANGLE_IN_DEGREES = False   # оставляем False — угол уже в радианах

    2. Перед калибровкой/управлением вызовите start_camera()
    3. В конце — stop_camera()

Единицы: пиксели изображения (условные). Камера сверху → перспектива
минимальна; центр тега даёт положение без сильных искажений.
Угол: atan2 по передней стороне тега (против часовой, как в OpenCV).
"""

import threading
import time
import math
import cv2
import numpy as np
from pupil_apriltags import Detector

# ===================== НАСТРОЙКИ =====================
CAMERA_INDEX = 0
CAP_BACKEND = cv2.CAP_AVFOUNDATION   # macOS; на Linux замените на cv2.CAP_V4L2 или 0
FRAME_W = 1920
FRAME_H = 1080

TAG_FAMILY = "tag36h11"
TARGET_ID = 0                       # ID тега на роботе (поменяйте при необходимости)

# Если угол из get_pose должен совпадать с направлением осей x,y
# (против часовой). При необходимости добавьте смещение в градусах:
ANGLE_OFFSET_DEG = 0.0


# ===================== ВНУТРЕННЕЕ СОСТОЯНИЕ =====================
_detector = Detector(
    families=TAG_FAMILY,
    nthreads=2,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
)

_cap = None
_lock = threading.Lock()
_latest = None          # (x, y, theta_rad) или None
_running = False
_thread = None


def _compute_pose(tag):
    """Центр тега + угол по передней стороне (corners[2]+[3])/2."""
    cx, cy = float(tag.center[0]), float(tag.center[1])
    corners = tag.corners  # [0]=BL, [1]=BR, [2]=TR, [3]=TL (по часовой)

    front_x = (corners[2][0] + corners[3][0]) * 0.5
    front_y = (corners[2][1] + corners[3][1]) * 0.5

    angle_rad = math.atan2(front_y - cy, front_x - cx)
    angle_rad += math.radians(ANGLE_OFFSET_DEG)
    # нормализуем в [-pi, pi]
    angle_rad = (angle_rad + math.pi) % (2 * math.pi) - math.pi

    return cx, cy, angle_rad


def _camera_loop():
    global _latest, _running
    while _running:
        ret, frame = _cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = _detector.detect(gray)

        pose = None
        for tag in detections:
            if tag.tag_id == TARGET_ID:
                pose = _compute_pose(tag)
                break

        with _lock:
            _latest = pose

        # небольшая пауза, чтобы не грузить CPU
        time.sleep(0.005)


def start_camera():
    """Открыть камеру и запустить фоновый поток детекции."""
    global _cap, _running, _thread, _latest

    if _running:
        return

    _cap = cv2.VideoCapture(CAMERA_INDEX, CAP_BACKEND)
    if not _cap.isOpened():
        # fallback без указания backend
        _cap = cv2.VideoCapture(CAMERA_INDEX)

    if not _cap.isOpened():
        raise RuntimeError(f"Не удалось открыть камеру {CAMERA_INDEX}")

    _cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    actual_w = int(_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[pose_camera] Камера: {actual_w}x{actual_h}, ищем тег ID={TARGET_ID}")

    _latest = None
    _running = True
    _thread = threading.Thread(target=_camera_loop, daemon=True)
    _thread.start()

    # ждём первый кадр
    t0 = time.time()
    while time.time() - t0 < 3.0:
        with _lock:
            if _latest is not None:
                break
        time.sleep(0.05)


def stop_camera():
    """Остановить поток и освободить камеру."""
    global _running, _cap, _thread
    _running = False
    if _thread is not None:
        _thread.join(timeout=2.0)
        _thread = None
    if _cap is not None:
        _cap.release()
        _cap = None
    print("[pose_camera] Камера остановлена")


def get_pose():
    """
    Возвращает (x, y, theta) в радианах или None, если тег не виден.
    x, y — в пикселях (условные единицы), theta — угол в радианах.
    """
    with _lock:
        return _latest


# ===================== ТЕСТ =====================
if __name__ == "__main__":
    print("Тест камеры. Нажмите Ctrl+C для выхода.")
    start_camera()
    try:
        while True:
            p = get_pose()
            if p is None:
                print("Тег не найден", end="\r")
            else:
                x, y, th = p
                print(f"X={x:7.1f}  Y={y:7.1f}  θ={math.degrees(th):6.1f}°   ", end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_camera()
