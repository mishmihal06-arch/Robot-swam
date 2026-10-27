#!/usr/bin/env python3
"""
robot_driver.py — драйвер трёхколёсного омни-робота (UDP). Без ручного управления.

Что делает:
  1. Находит робота по маякам (UDP :8889) и выбирает нужного по robot_id.
  2. Отправляет роботу пины моторов (команда PINS:).
  3. Переводит желаемое движение (vf, vl, w) в ШИМ моторов с учётом
     геометрии колёс и калибровки (gain / dz / inv) и шлёт "M:a,b,c,t".

Минимальный пример:

    from robot_driver import Robot

    with Robot.connect("robot_config.json") as robot:
        robot.hold(vf=0.5, vl=0.0, w=0.0, seconds=1.0)   # ехать вперёд 1 с
    # при выходе из with робот получает STOP

Подробности — в README.md.
"""

import json
import math
import os
import socket
import time

BEACON_PORT   = 8889    # порт, на котором робот шлёт маяки
SEND_INTERVAL = 0.05    # 20 Гц — рекомендуемый период отправки команд
TEST_SECONDS  = 2.0     # сколько крутить каждый мотор в тесте
TEST_PAUSE    = 0.5     # пауза между шагами теста

DEFAULT_CONFIG = {
    "robot_id": "",
    "wheel_angles_deg": [60, 180, 300],
    "drive_speed": 0.9,
    "turn_speed": 0.9,
    "command_ms": 250,
    "test_pwm": 200,
    "motors": {
        "1": {"pins": [18, 8, 17], "gain": 1.0, "inv": False, "dz": 0.7},
        "2": {"pins": [15, 7, 16], "gain": 1.0, "inv": False, "dz": 0.7},
        "3": {"pins": [10, 11, 9],  "gain": 1.0, "inv": True,  "dz": 0.7},
    },
}


class RobotError(Exception):
    """Базовая ошибка драйвера."""


class RobotNotFound(RobotError):
    """Робот не найден по маякам за отведённое время."""


# ==================== КОНФИГ ====================
def load_config(path="robot_config.json"):
    """
    Читает конфиг. Недостающие поля берутся из DEFAULT_CONFIG.
    Если файла нет — создаёт его со значениями по умолчанию.
    """
    if not os.path.exists(path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(DEFAULT_CONFIG, f, indent=2, ensure_ascii=False)
        print(f"[config] Создан {path} со значениями по умолчанию")
        return json.loads(json.dumps(DEFAULT_CONFIG))

    with open(path, encoding="utf-8") as f:
        user = json.load(f)

    if "alpha_deg" in user and "wheel_angles_deg" not in user:   # старое имя поля
        user["wheel_angles_deg"] = user["alpha_deg"]

    cfg = {**DEFAULT_CONFIG, **user}
    user_motors = user.get("motors", {})
    cfg["motors"] = {n: {**DEFAULT_CONFIG["motors"][n], **user_motors.get(n, {})}
                     for n in ("1", "2", "3")}
    missing = [k for k in DEFAULT_CONFIG if k not in user]
    if missing:
        print(f"[config] В {path} нет полей {missing}, использую значения по умолчанию")
    return cfg


# ==================== МАТЕМАТИКА ====================
def wheel_speeds(vf, vl, w, angles):
    """
    vf — скорость вперёд, vl — скорость влево, w — вращение против часовой
    (всё в долях от 1).

    angles — места колёс вокруг центра в радианах: 0 = спереди, дальше против
    часовой стрелки (90° = слева, 180° = сзади). Колесо толкает робота вдоль
    касательной к окружности, на которой оно стоит:

        u = vl * cos(angle) - vf * sin(angle) + w

    Результат нормируется так, чтобы max|u| <= 1.
    """
    u = [vl * math.cos(a) - vf * math.sin(a) + w for a in angles]
    m = max(1.0, max(abs(x) for x in u))
    return [x / m for x in u]


def calibrate(u, cfg):
    """
    Скорость колеса -> напряжение на мотор с учётом gain / dz / inv.
    Если слабому мотору нужно больше 100%, масштабируем ВСЕ колёса вместе,
    чтобы направление движения не искажалось.
    """
    motors = [cfg["motors"][str(i + 1)] for i in range(3)]
    v = [x / m["gain"] for x, m in zip(u, motors)]
    k = max(1.0, max(abs(x) for x in v))
    out = []
    for x, m in zip(v, motors):
        x /= k
        if abs(x) < 0.01:
            x = 0.0
        else:
            x = math.copysign(max(abs(x), m["dz"]), x)   # не ниже порога страгивания
        out.append(-x if m["inv"] else x)
    return out


def make_packet(cfg, vf, vl, w):
    """
    Чистая функция (без сети): (vf, vl, w) -> строка команды для робота.
    Возвращает "STOP", если все три компоненты нулевые, иначе "M:a,b,c,t".
    Удобно для тестов без робота.

    vf, vl, w — доли от 1. Вектор (vf, vl) ограничивается длиной drive_speed,
    |w| ограничивается turn_speed (потолки из конфига).
    """
    angles = [math.radians(a) for a in cfg["wheel_angles_deg"]]

    n = math.hypot(vf, vl)
    if n > cfg["drive_speed"]:
        vf, vl = vf / n * cfg["drive_speed"], vl / n * cfg["drive_speed"]
    w = max(-cfg["turn_speed"], min(cfg["turn_speed"], w))

    if vf == 0 and vl == 0 and w == 0:
        return "STOP"
    volts = calibrate(wheel_speeds(vf, vl, w, angles), cfg)
    pwm = [max(-255, min(255, round(x * 255))) for x in volts]   # целые -255..255
    return "M:" + ",".join(str(x) for x in pwm) + f",{cfg['command_ms']}"


# ==================== ПОИСК РОБОТА ====================
def find_robots(seconds=3.0):
    """Слушает маяки, возвращает {ID: (ip, port)}."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.bind(("", BEACON_PORT))
    except OSError as e:
        print(f"[discovery] Порт {BEACON_PORT} занят: {e}")
        sock.close()
        return {}
    sock.settimeout(0.5)

    found = {}
    t_end = time.time() + seconds
    while time.time() < t_end:
        try:
            data, _ = sock.recvfrom(512)
        except socket.timeout:
            continue
        msg = data.decode(errors="ignore").strip()
        if not msg.startswith("ROBOT:"):
            continue
        parts = msg.split(";")
        rid, ip, port = parts[0][6:], None, 8888
        for p in parts[1:]:
            if p.startswith("IP:"):
                ip = p[3:]
            elif p.startswith("PORT:"):
                port = int(p[5:])
        if ip:
            found[rid.upper()] = (ip, port)
    sock.close()
    return found


def pick_robot(cfg, timeout=30.0, interactive=True):
    """
    Возвращает (id, ip, port).
    - robot_id задан в конфиге -> ждёт именно его;
    - иначе берёт единственного найденного;
    - если роботов несколько: interactive=True — спрашивает номер в консоли,
      interactive=False — бросает RobotError (задайте robot_id в конфиге).
    Если за timeout секунд ничего не нашлось — RobotNotFound.
    """
    want = str(cfg.get("robot_id", "")).strip().upper()
    t_end = time.time() + timeout
    while time.time() < t_end:
        found = find_robots()
        if want:
            if want in found:
                return (want, *found[want])
        elif len(found) == 1:
            rid = next(iter(found))
            return (rid, *found[rid])
        elif len(found) > 1:
            ids = sorted(found)
            if not interactive:
                raise RobotError(f"Найдено несколько роботов {ids}, задайте robot_id в конфиге")
            print("[discovery] Найдено несколько роботов:")
            for i, rid in enumerate(ids, 1):
                print(f"  {i}) {rid}  {found[rid][0]}")
            try:
                rid = ids[int(input("Номер робота: ")) - 1]
                return (rid, *found[rid])
            except (ValueError, IndexError):
                continue
        print("[discovery] Робот не найден, жду маяк...")
    raise RobotNotFound(f"Робот {want or ''} не найден за {timeout:.0f} с")


# ==================== РОБОТ ====================
class Robot:
    """
    Соединение с одним роботом. Создавайте через Robot.connect().

    Главное правило: робот сам останавливается через command_ms (по умолчанию
    250 мс) после последней команды. Поэтому drive() нужно вызывать регулярно
    (рекомендуется каждые SEND_INTERVAL = 50 мс), пока робот должен ехать.
    """

    def __init__(self, cfg, rid, ip, port, verbose=True):
        self.cfg = cfg
        self.id = rid
        self.ip = ip
        self.port = port
        self.verbose = verbose
        self.last_packet = None          # последняя отправленная команда
        self.last_reply_time = time.time()
        self._last_printed = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    # ---------- создание / закрытие ----------
    @classmethod
    def connect(cls, config_path="robot_config.json", timeout=30.0,
                interactive=True, verbose=True, push_pins=True):
        """Находит робота, загружает конфиг, отправляет пины. Возвращает Robot."""
        cfg = load_config(config_path)
        rid, ip, port = pick_robot(cfg, timeout=timeout, interactive=interactive)
        if verbose:
            print(f"[main] Робот {rid} ({ip}:{port})")
        robot = cls(cfg, rid, ip, port, verbose=verbose)
        if push_pins and not robot.push_pins():
            robot.close()
            raise RobotError("Робот не принял пины моторов")
        return robot

    def close(self):
        """Останавливает робота (3 раза, UDP может потеряться) и закрывает сокет."""
        for _ in range(3):
            try:
                self.sock.sendto(b"STOP", (self.ip, self.port))
            except OSError:
                break
            time.sleep(0.03)
        self.sock.close()
        if self.verbose:
            print("[main] Робот остановлен.")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---------- команды ----------
    def _send(self, pkt):
        if self.verbose and pkt != self._last_printed:   # повторы не печатаем
            print(f"-> {pkt}")
            self._last_printed = pkt
        self.last_packet = pkt
        self.sock.sendto(pkt.encode(), (self.ip, self.port))

    def drive(self, vf, vl, w):
        """
        Отправляет ОДНУ команду движения. Возвращает отправленную строку.

        vf — вперёд (+) / назад (-)             ┐
        vl — влево (+) / вправо (-)             ├ доли от 1, обычно -1..1
        w  — поворот против часовой (+) / по (-)┘

        Вектор (vf, vl) ограничивается длиной drive_speed, w — turn_speed.
        Робот выполнит команду command_ms мс и остановится, если не придёт новая.
        """
        pkt = make_packet(self.cfg, vf, vl, w)
        self._send(pkt)
        self.poll()
        return pkt

    def stop(self):
        """Немедленная остановка."""
        self._send("STOP")
        self.poll()

    def hold(self, vf, vl, w, seconds):
        """Блокирующий вызов: повторяет drive() с частотой 20 Гц seconds секунд, затем stop()."""
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.drive(vf, vl, w)
            time.sleep(SEND_INTERVAL)
        self.stop()

    # ---------- ответы робота ----------
    def poll(self):
        """
        Читает накопившиеся ответы робота (не блокирует). Возвращает список строк.
        Ошибки ("ERR...") печатаются. Обновляет last_reply_time.
        """
        msgs = []
        try:
            while True:
                data, _ = self.sock.recvfrom(512)
                text = data.decode(errors="ignore")
                msgs.append(text)
                self.last_reply_time = time.time()
                if text.startswith("ERR") and self.verbose:
                    print(f"[robot] {text}")
        except (BlockingIOError, OSError):
            pass
        return msgs

    def seconds_since_reply(self):
        """Сколько секунд робот ничего не отвечал (для детекта потери связи; в ручной версии порог был 2 с)."""
        return time.time() - self.last_reply_time

    # ---------- настройка ----------
    def push_pins(self, tries=5):
        """Отправляет роботу пины моторов (они сохраняются в его флеше). True при успехе."""
        pins = [p for i in "123" for p in self.cfg["motors"][i]["pins"]]
        pkt = ("PINS:" + ",".join(map(str, pins))).encode()
        if self.verbose:
            print(f"-> {pkt.decode()}")
        for _ in range(tries):
            self.sock.sendto(pkt, (self.ip, self.port))
            t_end = time.time() + 1.0
            while time.time() < t_end:
                try:
                    data, _ = self.sock.recvfrom(512)
                except (BlockingIOError, OSError):
                    time.sleep(0.02)
                    continue
                self.last_reply_time = time.time()
                text = data.decode(errors="ignore")
                if text.startswith("OK"):
                    if self.verbose:
                        print("[config] Пины установлены")
                    return True
                if text.startswith("ERR"):
                    print(f"[config] Робот отклонил пины: {text}")
                    return False
        print("[config] Нет ответа от робота")
        return False

    def test_motors(self, should_abort=None):
        """
        Поочерёдно крутит моторы 1, 2, 3: каждый вперёд, потом назад, по 2 с.
        Команды идут напрямую на моторы, БЕЗ калибровки (gain / dz / inv):
        проверяется железо; положительный ШИМ = IN1 HIGH.
        should_abort — необязательная функция без аргументов; если вернёт True,
        тест прерывается. Ctrl+C тоже прерывает (и робот останавливается).
        """
        pwm = int(self.cfg["test_pwm"])
        print(f"[test] Тест моторов (ШИМ {pwm}): каждый вперёд и назад по {TEST_SECONDS:.0f} с.")

        def hold_raw(pkt, seconds):
            t_end = time.time() + seconds
            while time.time() < t_end:
                if should_abort and should_abort():
                    self._send("STOP")
                    print("[test] Прервано")
                    return False
                self._send(pkt)
                self.poll()
                time.sleep(SEND_INTERVAL)
            return True

        try:
            for n in range(3):
                for sign, name in ((1, "вперёд"), (-1, "назад")):
                    vals = [0, 0, 0]
                    vals[n] = sign * pwm
                    print(f"[test] Мотор {n + 1}: {name}")
                    pkt = f"M:{vals[0]},{vals[1]},{vals[2]},{self.cfg['command_ms']}"
                    if not hold_raw(pkt, TEST_SECONDS) or not hold_raw("STOP", TEST_PAUSE):
                        return
            print("[test] Тест завершён")
        finally:
            self._send("STOP")
