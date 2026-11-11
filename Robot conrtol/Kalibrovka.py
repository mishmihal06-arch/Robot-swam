#!/usr/bin/env python3
"""
robot_ctrl.py — точное управление трёхколёсным роботом по камере.

    robot.rotate(90)      # повернуться на 90° (знак как у theta из get_pose)
    robot.drive(300)      # проехать 300 единиц вперёд (вдоль оси X метки)
    robot.drive(200, direction_deg=90)   # боком (для омни-колёс)
    robot.go_to(x, y, theta)             # в произвольную точку и угол

Как это работает
  1. АВТОКАЛИБРОВКА. Геометрия колёс не нужна: робот по очереди крутит каждый
     мотор, камера измеряет, как он при этом едет/вращается, и из этого строится
     матрица B (3x3): [vx, vy, omega] в системе робота = B @ [m1, m2, m3].
     Обратная матрица даёт «какие моторы включить, чтобы поехать/повернуть так-то».
     Это работает для любой компоновки 3 колёс, любых направлений вращения,
     любого положения метки на роботе.
  2. РЕГУЛЯТОР. В каждом кадре: ошибка (x, y, угол) -> в систему робота ->
     P-регулятор -> скорости -> моторы (через обратную матрицу).
  3. ТОЧНАЯ ДОВОДКА. Вблизи цели робот двигается короткими импульсами с паузой
     на успокоение (чтобы не мешали инерция и задержка камеры).

Ограничения: единицы длины = единицы, которые возвращает get_pose().
Угол из get_pose() должен быть направлен в ту же сторону, что и оси x,y
(против часовой при обычных осях; для пиксельных координат — как atan2(dy, dx)).

Поза берётся из pose_camera.py (потолочная камера + AprilTag).
"""
import json
import math
import os
import socket
import sys
import time

import numpy as np

# ===================== 0. КАМЕРА (поза) =====================
from pose_camera import get_pose, start_camera, stop_camera

# ===================== 1. ПОДКЛЮЧЕНИЕ =====================
ROBOT_IP = "вставить"
ROBOT_PORT = 8888
CALIB_FILE = "robot_calibration.json"

# ===================== 2. ФУНКЦИЯ ПОЗЫ =====================
# get_pose импортирован из pose_camera: возвращает (x, y, theta_rad) или None
ANGLE_IN_DEGREES = False      # pose_camera отдаёт угол в радианах


# ===================== 3. НАСТРОЙКИ =====================
# Допуски (длина — в единицах get_pose, пиксели)
POS_TOL = 2.0                 # допустимая ошибка по положению
ANG_TOL_DEG = 1.0             # допустимая ошибка по углу
# Зона «точной доводки» импульсами
FINE_POS = 25.0
FINE_ANG_DEG = 10.0

# Регулятор
KP_POS = 2.5                  # 1/с: скорость = KP * ошибка (уменьшите, если проскакивает)
KP_ROT = 2.5
SPEED_FRAC = 0.7              # доля от максимальной скорости, которую разрешаем
PWM_MAX_RUN = 220             # максимум ШИМ в движении (0..255)
PWM_MIN_DEFAULT = 60          # минимум ШИМ, при котором робот трогается (уточняется find_min_pwm)

# Время
CMD_MS = 200                  # параметр T в пакете (если связь пропала — робот остановится сам)
LOOP_DT = 0.03                # период цикла управления
PULSE_MS = 60                 # длина импульса в режиме доводки
PULSE_SETTLE = 0.25           # пауза после импульса, с
SETTLE = 0.4                  # пауза после остановки перед измерением, с
LOST_TIMEOUT = 0.5            # сколько ждать появления метки, с
DEFAULT_TIMEOUT = 20.0        # таймаут одного движения, с

# Калибровка
CAL_PWM = 150                 # ШИМ при калибровке
CAL_DURATION = 0.8            # длительность каждого прогона, с
CAL_REPEATS = 2               # сколько раз повторять каждый мотор в каждую сторону


# ===================== 4. ВСПОМОГАТЕЛЬНОЕ =====================
def wrap(a):
    """Угол в диапазон [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def read_pose():
    p = get_pose()
    if p is None:
        return None
    x, y, th = p
    if ANGLE_IN_DEGREES:
        th = math.radians(th)
    return float(x), float(y), float(th)


def stable_pose(n=5, dt=0.03, timeout=2.0):
    """Медиана по нескольким кадрам — устойчивая оценка позы стоящего робота."""
    xs, ys, ss, cs = [], [], [], []
    t_end = time.time() + timeout
    while len(xs) < n and time.time() < t_end:
        p = read_pose()
        if p is not None:
            xs.append(p[0]); ys.append(p[1])
            ss.append(math.sin(p[2])); cs.append(math.cos(p[2]))
        time.sleep(dt)
    if not xs:
        return None
    return (float(np.median(xs)), float(np.median(ys)),
            math.atan2(float(np.median(ss)), float(np.median(cs))))


def body_delta(p0, p1):
    """Смещение p0 -> p1 в системе координат робота в позе p0: (dx, dy, dtheta)."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(p1[2] - p0[2])])


def mean_velocity(samples, skip=0.25, gap=0.15):
    """Средняя скорость (в системе робота) по записи [(t, pose), ...]."""
    if not samples:
        return None
    t0 = samples[0][0]
    s = [x for x in samples if x[0] - t0 >= skip]
    vs = []
    for k in range(len(s)):
        for j in range(k + 1, len(s)):
            dt = s[j][0] - s[k][0]
            if dt >= gap:
                vs.append(body_delta(s[k][1], s[j][1]) / dt)
                break
    return np.mean(vs, axis=0) if vs else None


# ===================== 5. РОБОТ =====================
class Robot:
    def __init__(self, ip=ROBOT_IP, port=ROBOT_PORT):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.B = None              # скорость робота [vx, vy, w] при ШИМ = cal_pwm на моторе
        self.Binv = None
        self.cal_pwm = CAL_PWM
        self.pwm_min = PWM_MIN_DEFAULT

    # ---------- связь ----------
    def send(self, m, T=CMD_MS):
        m = [int(round(max(-255, min(255, v)))) for v in m]
        pkt = f"M1:{m[0]};M2:{m[1]};M3:{m[2]};T:{int(T)}"
        self.sock.sendto(pkt.encode(), self.addr)
        self._drain()

    def stop(self):
        for _ in range(2):
            self.sock.sendto(b"STOP", self.addr)
        self._drain()

    def _drain(self):
        # прошивка отвечает "OK" на каждый пакет — вычищаем буфер
        try:
            while True:
                self.sock.recv(256)
        except (BlockingIOError, OSError):
            pass

    # ---------- калибровка ----------
    def _run_and_track(self, m, duration):
        samples = []
        t0 = time.time()
        while time.time() - t0 < duration:
            self.send(m, CMD_MS)
            p = read_pose()
            if p is not None:
                samples.append((time.time(), p))
            time.sleep(LOOP_DT)
        self.stop()
        return samples

    def calibrate(self, pwm=CAL_PWM, duration=CAL_DURATION, repeats=CAL_REPEATS):
        """Измеряет матрицу B. Робот проедет/повернётся в обе стороны — освободите место."""
        print("Калибровка: каждый мотор по очереди вперёд и назад...")
        cols = [[] for _ in range(3)]
        for i in range(3):
            for sign in (1, -1):
                for _ in range(repeats):
                    m = [0, 0, 0]
                    m[i] = sign * pwm
                    samples = self._run_and_track(m, duration)
                    time.sleep(SETTLE)
                    v = mean_velocity(samples)
                    if v is None:
                        raise RuntimeError("Метка не видна во время калибровки")
                    cols[i].append(v * sign)
        B = np.array([np.mean(c, axis=0) for c in cols]).T
        # проверки качества
        for i in range(3):
            c = np.array(cols[i])
            spread = np.linalg.norm(c.std(axis=0)) / (np.linalg.norm(c.mean(axis=0)) + 1e-9)
            if spread > 0.3:
                print(f"  ВНИМАНИЕ: мотор {i + 1}: разброс измерений {spread:.0%} "
                      f"(вперёд/назад отличаются) — попробуйте больший CAL_PWM")
        Bn = B.copy()
        Bn[2] *= np.linalg.norm(B[:2]) / (np.linalg.norm(B[2]) + 1e-12)
        cond = np.linalg.cond(Bn)
        print("B =\n", B)
        if cond > 30 or not np.isfinite(cond):
            raise RuntimeError(f"Матрица плохо обусловлена (cond={cond:.0f}): "
                               f"моторы не двигали робота независимо. Проверьте питание/CAL_PWM.")
        self.set_B(B, pwm)

    def set_B(self, B, pwm):
        self.B = np.array(B, dtype=float)
        self.Binv = np.linalg.inv(self.B)
        self.cal_pwm = pwm

    def find_min_pwm(self, start=40, step=5, hold=0.3):
        """Находит минимальный ШИМ, при котором робот реально трогается (стартовое трение)."""
        assert self.Binv is not None, "сначала calibrate()"
        found = []
        for v in (np.array([1.0, 0, 0]), np.array([0, 0, 1.0])):
            d = self.Binv @ v
            d = d / np.max(np.abs(d))
            p0 = stable_pose()
            hit = False
            for s in range(start, 256, step):
                self._run_and_track(list(s * d), hold)
                time.sleep(0.2)
                p1 = stable_pose()
                if p0 is None or p1 is None:
                    raise RuntimeError("Метка не видна")
                dd = body_delta(p0, p1)
                if math.hypot(dd[0], dd[1]) > 2 * POS_TOL or abs(dd[2]) > math.radians(2 * ANG_TOL_DEG):
                    found.append(s)
                    hit = True
                    break
                p0 = p1
            if not hit:
                raise RuntimeError("Робот не тронулся даже на максимальном ШИМ")
        self.pwm_min = int(math.ceil(1.1 * max(found)))
        print(f"Минимальный рабочий ШИМ: {self.pwm_min}")

    def save(self, path=CALIB_FILE):
        with open(path, "w") as f:
            json.dump({"B": self.B.tolist(), "cal_pwm": self.cal_pwm,
                       "pwm_min": self.pwm_min}, f, indent=2)

    def load(self, path=CALIB_FILE):
        with open(path) as f:
            d = json.load(f)
        self.set_B(d["B"], d["cal_pwm"])
        self.pwm_min = d["pwm_min"]

    # ---------- управление ----------
    def _cap(self, unit_vec):
        """Макс. величина скорости вдоль направления unit_vec при ШИМ = PWM_MAX_RUN."""
        col = np.abs(self.Binv @ unit_vec).max()
        return PWM_MAX_RUN / (self.cal_pwm * col + 1e-12)

    def _motors(self, v):
        """Желаемая скорость робота [vx, vy, w] -> ШИМ моторов (с учётом мин./макс. ШИМ)."""
        m = self.cal_pwm * (self.Binv @ v)
        mx = np.max(np.abs(m))
        if mx < 1e-9:
            return np.zeros(3)
        if mx > PWM_MAX_RUN:
            m *= PWM_MAX_RUN / mx
        elif mx < self.pwm_min:
            m *= self.pwm_min / mx       # направление сохраняется, скорость — минимально трогающая
        return m

    def go_to(self, tx, ty, tth, pos_tol=POS_TOL, ang_tol_deg=ANG_TOL_DEG,
              timeout=DEFAULT_TIMEOUT):
        """Приехать в точку (tx, ty) с углом tth (рад). tx=None — позицию не контролировать."""
        assert self.Binv is not None, "сначала calibrate() или load()"
        ang_tol = math.radians(ang_tol_deg)
        fine_ang = math.radians(FINE_ANG_DEG)
        w_cap = SPEED_FRAC * self._cap(np.array([0, 0, 1.0]))
        t_start = time.time()
        lost_since = None
        was_fine = False
        try:
            while time.time() - t_start < timeout:
                p = stable_pose(2, 0.02) if was_fine else read_pose()
                if p is None:
                    lost_since = lost_since or time.time()
                    self.stop()
                    if time.time() - lost_since > LOST_TIMEOUT:
                        raise RuntimeError("Метка потеряна")
                    time.sleep(LOOP_DT)
                    continue
                lost_since = None
                ex, ey = (0.0, 0.0) if tx is None else (tx - p[0], ty - p[1])
                eth = wrap(tth - p[2])
                dist = math.hypot(ex, ey)

                if dist <= pos_tol and abs(eth) <= ang_tol:
                    # цель достигнута — остановиться, подождать и перепроверить
                    self.stop()
                    time.sleep(SETTLE)
                    q = stable_pose()
                    if q is not None:
                        qx, qy = (0.0, 0.0) if tx is None else (tx - q[0], ty - q[1])
                        if math.hypot(qx, qy) <= pos_tol and abs(wrap(tth - q[2])) <= ang_tol:
                            return q
                    was_fine = True
                    continue

                c, s = math.cos(p[2]), math.sin(p[2])
                bx, by = c * ex + s * ey, -s * ex + c * ey
                v = np.array([KP_POS * bx, KP_POS * by, KP_ROT * eth])
                lin = math.hypot(v[0], v[1])
                if lin > 1e-9:
                    cap = SPEED_FRAC * self._cap(np.array([v[0] / lin, v[1] / lin, 0.0]))
                    if lin > cap:
                        v[:2] *= cap / lin
                v[2] = float(np.clip(v[2], -w_cap, w_cap))
                m = self._motors(v)

                fine = dist < FINE_POS and abs(eth) < fine_ang
                if fine:
                    self.send(m, PULSE_MS)
                    time.sleep(PULSE_MS / 1000.0)
                    self.stop()
                    time.sleep(PULSE_SETTLE)
                else:
                    self.send(m, CMD_MS)
                    time.sleep(LOOP_DT)
                was_fine = fine
            raise TimeoutError("Не удалось достичь цели за отведённое время")
        finally:
            self.stop()

    def rotate(self, angle_deg, **kw):
        """Повернуться на angle_deg относительно текущего угла."""
        p = stable_pose()
        if p is None:
            raise RuntimeError("Метка не видна")
        return self.go_to(None, None, p[2] + math.radians(angle_deg), **kw)

    def drive(self, distance, direction_deg=0.0, **kw):
        """Проехать distance (в единицах get_pose) в направлении direction_deg
        относительно оси X метки; угол робота при этом удерживается."""
        p = stable_pose()
        if p is None:
            raise RuntimeError("Метка не видна")
        a = p[2] + math.radians(direction_deg)
        return self.go_to(p[0] + distance * math.cos(a),
                          p[1] + distance * math.sin(a), p[2], **kw)


# ===================== 6. ЗАПУСК =====================
if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "demo"
    robot = Robot()
    start_camera()
    try:
        if cmd == "calibrate" or not os.path.exists(CALIB_FILE):
            robot.calibrate()
            robot.find_min_pwm()
            robot.save()
            print(f"Калибровка сохранена в {CALIB_FILE}")
        else:
            robot.load()
        if cmd == "demo":
            robot.rotate(90)
            robot.rotate(-90)
            robot.drive(200)
            robot.drive(-200)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        stop_camera()
