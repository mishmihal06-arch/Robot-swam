#!/usr/bin/env python3
"""
robot_control.py — управление трёхколёсным роботом с клавиатуры.

    python robot_control.py [config.json]

1. Ищет роботов по маякам (UDP :8889), выбирает нужного по robot_id.
2. Отправляет роботу пины моторов.
3. Считает кинематику + калибровку и шлёт "M:a,b,c,t"
   (a, b, c — целые от -255 до 255, t — время действия в мс).

Каждая новая команда печатается в терминал (повторы одной и той же команды
не печатаются, чтобы не засорять вывод).

    W / S / A / D (+ комбинации) — движение
    ← / →                        — поворот на месте
    T                            — тест моторов: каждый вперёд и назад по 2 с
    Пробел                       — стоп (во время теста — прервать тест)
    ESC                          — выход
"""

import json
import math
import os
import socket
import sys
import time

import keyboard

CONFIG_FILE   = sys.argv[1] if len(sys.argv) > 1 else "robot_config.json"
BEACON_PORT   = 8889
SEND_INTERVAL = 0.05   # 20 Гц
TEST_SECONDS  = 2.0    # сколько крутить каждый мотор в тесте
TEST_PAUSE    = 0.5    # пауза между шагами теста

DEFAULT_CONFIG = {
    "_help_wheel_angles_deg": (
        "Где стоит каждое колесо вокруг центра робота. Отсчёт от направления вперёд "
        "против часовой стрелки (налево): 0 = спереди, 90 = слева, 180 = сзади, 270 = справа. "
        "Колесо 1 спереди слева (60), колесо 2 сзади (180), колесо 3 спереди справа (300)."
    ),
    # id робота (печатается в Serial при старте и приходит в маяке).
    # Пусто = взять единственного найденного, а если их несколько — спросить.
    "robot_id": "",
    "wheel_angles_deg": [60, 180, 300],
    # Скорости с клавиатуры (доля от 1) и время действия каждой команды, мс.
    # Если связь пропадёт, робот остановится через command_ms
    # (прошивка принимает не больше 1000 мс).
    "drive_speed": 0.8,
    "turn_speed": 0.6,
    "command_ms": 250,
    # ШИМ (0..255) для теста моторов по клавише T
    "test_pwm": 200,
    # Моторы:
    #   pins — [IN1, IN2, EN]  (уходят на робота и сохраняются в его флеше)
    #   gain — макс. скорость относительно самого сильного мотора (слабый < 1)
    #   inv  — инверсия направления
    #   dz   — порог страгивания: минимальная доля ШИМ (0..1), с которой мотор уверенно
    #          крутится. Всё, что ниже (но не ноль), поднимается до этого значения.
    "motors": {
        "1": {"pins": [18, 8, 17],  "gain": 1.0, "inv": False, "dz": 0.0},
        "2": {"pins": [15, 7, 16],  "gain": 1.0, "inv": False, "dz": 0.0},
        "3": {"pins": [10, 11, 9],  "gain": 1.0, "inv": False, "dz": 0.0},
    },
}

HELP = """\
----------------------------------------------------------------
  W / S / A / D (+ комбинации) — движение
  ← / →                        — поворот на месте
  T                            — тест моторов (каждый вперёд/назад по 2 с)
  Пробел                       — стоп / прервать тест
  ESC                          — выход
----------------------------------------------------------------"""


# ==================== КОНФИГ ====================
def load_config():
    if not os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(DEFAULT_CONFIG, f, indent=2, ensure_ascii=False)
        print(f"[config] Создан {CONFIG_FILE} со значениями по умолчанию")
        return DEFAULT_CONFIG

    with open(CONFIG_FILE, encoding="utf-8") as f:
        user = json.load(f)

    # Старое название поля углов
    if "alpha_deg" in user and "wheel_angles_deg" not in user:
        user["wheel_angles_deg"] = user["alpha_deg"]

    # Недостающие поля берём из значений по умолчанию
    cfg = {**DEFAULT_CONFIG, **user}
    user_motors = user.get("motors", {})
    cfg["motors"] = {n: {**DEFAULT_CONFIG["motors"][n], **user_motors.get(n, {})}
                     for n in ("1", "2", "3")}
    missing = [k for k in DEFAULT_CONFIG if k not in user and not k.startswith("_")]
    if missing:
        print(f"[config] В {CONFIG_FILE} нет полей {missing}, использую значения по умолчанию")
    return cfg


# ==================== МАТЕМАТИКА ====================
def wheel_speeds(vf, vl, w, angles):
    """
    vf — скорость вперёд, vl — скорость влево, w — вращение против часовой
    (всё в долях от 1).

    angles — места колёс вокруг центра в радианах: 0 = спереди, дальше против
    часовой стрелки (90° = слева, 180° = сзади). Колесо толкает робота вдоль
    касательной к окружности, на которой оно стоит, а его скорость — проекция
    желаемого движения на это направление:

        u = vl * cos(angle) - vf * sin(angle) + w

    Первые два слагаемых отвечают за движение, третье — за поворот
    (при повороте на месте все колёса получают одно и то же w).
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


def compute_packet(cfg, angles):
    k = keyboard.is_pressed
    if k("space"):
        return "STOP"

    w  = (float(k("left")) - float(k("right"))) * cfg["turn_speed"]
    vf = float(k("w")) - float(k("s"))
    vl = float(k("a")) - float(k("d"))
    n = math.hypot(vf, vl)
    if n > 0:
        vf, vl = vf / n * cfg["drive_speed"], vl / n * cfg["drive_speed"]

    if vf == 0 and vl == 0 and w == 0:
        return "STOP"
    volts = calibrate(wheel_speeds(vf, vl, w, angles), cfg)
    pwm = [max(-255, min(255, round(x * 255))) for x in volts]   # целые -255..255
    return "M:" + ",".join(str(x) for x in pwm) + f",{cfg['command_ms']}"


# ==================== ПОИСК РОБОТА ====================
def find_robots(seconds=3.0):
    """Слушает маяки, возвращает {id: (ip, port)}."""
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


def pick_robot(cfg):
    """Возвращает (id, ip, port) или None, если нажали ESC."""
    want = cfg.get("robot_id", "").strip().upper()
    while not keyboard.is_pressed("esc"):
        found = find_robots()
        if want:
            if want in found:
                return (want, *found[want])
        elif len(found) == 1:
            rid = next(iter(found))
            return (rid, *found[rid])
        elif len(found) > 1:
            ids = sorted(found)
            print("[discovery] Найдено несколько роботов:")
            for i, rid in enumerate(ids, 1):
                print(f"  {i}) {rid}  {found[rid][0]}")
            try:
                rid = ids[int(input("Номер робота: ")) - 1]
                return (rid, *found[rid])
            except (ValueError, IndexError):
                continue
        print("[discovery] Робот не найден, жду маяк... (ESC — выход)")
    return None


# ==================== ОБЩЕНИЕ ====================
_last_printed = None


def send_command(sock, ip, port, pkt):
    """Отправляет команду и печатает её в терминал, если она изменилась."""
    global _last_printed
    if pkt != _last_printed:
        print(f"-> {pkt}")
        _last_printed = pkt
    sock.sendto(pkt.encode(), (ip, port))


def drain(sock):
    """Читает ответы робота. True, если что-то пришло. Ошибки робота печатает."""
    got = False
    try:
        while True:
            data, _ = sock.recvfrom(512)
            got = True
            text = data.decode(errors="ignore")
            if text.startswith("ERR"):
                print(f"[robot] {text}")
    except (BlockingIOError, OSError):
        pass
    return got


def push_pins(sock, ip, port, cfg, tries=5):
    pins = [p for i in "123" for p in cfg["motors"][i]["pins"]]
    pkt = ("PINS:" + ",".join(map(str, pins))).encode()
    print(f"-> {pkt.decode()}")
    for _ in range(tries):
        sock.sendto(pkt, (ip, port))
        t_end = time.time() + 1.0
        while time.time() < t_end:
            try:
                data, _ = sock.recvfrom(512)
            except (BlockingIOError, OSError):
                time.sleep(0.02)
                continue
            text = data.decode(errors="ignore")
            if text.startswith("OK"):
                print("[config] Пины установлены")
                return True
            if text.startswith("ERR"):
                print(f"[config] Робот отклонил пины: {text}")
                return False
    print("[config] Нет ответа от робота")
    return False


# ==================== ТЕСТ МОТОРОВ ====================
def hold(sock, ip, port, pkt, seconds):
    """Повторяет команду pkt в течение seconds. False, если тест прервали."""
    t_end = time.time() + seconds
    while time.time() < t_end:
        if keyboard.is_pressed("esc") or keyboard.is_pressed("space"):
            send_command(sock, ip, port, "STOP")
            print("[test] Прервано")
            return False
        send_command(sock, ip, port, pkt)
        drain(sock)
        time.sleep(SEND_INTERVAL)
    return True


def run_motor_test(sock, ip, port, cfg):
    """
    Поочерёдно крутит моторы 1, 2, 3: каждый вперёд, потом назад, по 2 секунды.
    Команды идут напрямую на моторы, без калибровки (gain / dz / inv),
    чтобы проверялось именно железо: положительный ШИМ = IN1 HIGH.
    """
    pwm = int(cfg["test_pwm"])
    print(f"[test] Тест моторов (ШИМ {pwm}): каждый вперёд и назад по {TEST_SECONDS:.0f} с.")
    for n in range(3):
        for sign, name in ((1, "вперёд"), (-1, "назад")):
            vals = [0, 0, 0]
            vals[n] = sign * pwm
            print(f"[test] Мотор {n + 1}: {name}")
            pkt = f"M:{vals[0]},{vals[1]},{vals[2]},{cfg['command_ms']}"
            if not hold(sock, ip, port, pkt, TEST_SECONDS):
                return
            if not hold(sock, ip, port, "STOP", TEST_PAUSE):
                return
    print("[test] Тест завершён")


# ==================== УПРАВЛЕНИЕ ====================
def control_loop(sock, ip, port, cfg):
    global _last_printed
    _last_printed = None
    angles = [math.radians(a) for a in cfg["wheel_angles_deg"]]
    print(HELP)
    last_ok = time.time()

    while True:
        if keyboard.is_pressed("esc"):
            return "exit"

        if keyboard.is_pressed("t"):
            run_motor_test(sock, ip, port, cfg)
            while keyboard.is_pressed("t"):   # ждём, пока отпустят T
                time.sleep(0.05)
            last_ok = time.time()
            continue

        send_command(sock, ip, port, compute_packet(cfg, angles))
        if drain(sock):
            last_ok = time.time()

        if time.time() - last_ok > 2.0:
            print("[robot] Связь потеряна")
            return "lost"
        time.sleep(SEND_INTERVAL)


# ==================== MAIN ====================
def main():
    cfg = load_config()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    ip = port = None

    try:
        while True:
            robot = pick_robot(cfg)
            if robot is None:
                break
            rid, ip, port = robot
            print(f"[main] Робот {rid} ({ip}:{port})")
            if not push_pins(sock, ip, port, cfg):
                time.sleep(2)
                continue
            if control_loop(sock, ip, port, cfg) == "exit":
                break
    except KeyboardInterrupt:
        pass
    finally:
        if ip:
            for _ in range(3):
                sock.sendto(b"STOP", (ip, port))
                time.sleep(0.03)
        sock.close()
        print("[main] Выход. Робот остановлен.")


if __name__ == "__main__":
    main()