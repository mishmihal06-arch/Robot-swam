#!/usr/bin/env python3
"""
robot_control.py — управление одним или несколькими трёхколёсными роботами.

    python robot_control.py                # выбрать робота(ов) интерактивно
    python robot_control.py --all          # подключиться ко всем найденным
    python robot_control.py A1B2C3 D4E5F6  # подключиться к указанным id
    python robot_control.py --list         # показать конфиги на диске и выйти

Конфиги берутся из robot_configs/<ID>.json (создаются robot_config_store.py).
Старый одиночный robot_config.json поддерживается только как fallback,
если для робота нет <ID>.json — но в этом случае будет предупреждение.

Команды дублируются всем выбранным роботам одновременно (N unicast-ов
из одного цикла). Каждая новая команда печатается один раз.

    W / S / A / D (+ комбинации) — движение
    ← / →                        — поворот на месте
    T                            — тест моторов (у каждого робота)
    Пробел                       — стоп / прервать тест
    ESC                          — выход
"""

import json
import math
import os
import socket
import sys
import time

import keyboard

from robot_config_store import (
    load_robot_config,
    list_robot_configs,
    find_robots,
)

BEACON_PORT   = 8889
SEND_INTERVAL = 0.05
TEST_SECONDS  = 2.0
TEST_PAUSE    = 0.5

# Fallback-конфиг (старый одиночный файл). Используется ТОЛЬКО если для
# конкретного робота нет robot_configs/<ID>.json — с явным предупреждением.
LEGACY_CONFIG_FILE = "robot_config.json"

FALLBACK_CONFIG = {
    "_help_wheel_angles_deg": (
        "Где стоит каждое колесо вокруг центра робота. 0 = спереди, 90 = слева, "
        "180 = сзади, 270 = справа."
    ),
    "robot_id": "",
    "wheel_angles_deg": [60, 180, 300],
    "drive_speed": 0.8,
    "turn_speed": 0.6,
    "command_ms": 250,
    "test_pwm": 200,
    "motors": {
        "1": {"pins": [18, 8, 17], "gain": 1.0, "inv": False, "dz": 0.0},
        "2": {"pins": [15, 7, 16], "gain": 1.0, "inv": False, "dz": 0.0},
        "3": {"pins": [10, 11, 9], "gain": 1.0, "inv": False, "dz": 0.0},
    },
}

HELP = """\
----------------------------------------------------------------
  W / S / A / D (+ комбинации) — движение
  ← / →                        — поворот на месте
  T                            — тест моторов
  Пробел                       — стоп / прервать тест
  ESC                          — выход
----------------------------------------------------------------"""


# ==================== КОНФИГИ ====================

def _legacy_cfg_or_none() -> dict:
    """Читает старый одиночный robot_config.json, если он есть."""
    if not os.path.exists(LEGACY_CONFIG_FILE):
        return None
    with open(LEGACY_CONFIG_FILE, encoding="utf-8") as f:
        user = json.load(f)
    if "alpha_deg" in user and "wheel_angles_deg" not in user:
        user["wheel_angles_deg"] = user["alpha_deg"]
    cfg = {**FALLBACK_CONFIG, **user}
    user_motors = user.get("motors", {})
    cfg["motors"] = {n: {**FALLBACK_CONFIG["motors"][n], **user_motors.get(n, {})}
                     for n in ("1", "2", "3")}
    return cfg


def resolve_configs(robot_ids: list) -> dict:
    """
    Для каждого id возвращает конфиг.
    Приоритет:
        1) robot_configs/<ID>.json
        2) legacy robot_config.json — только если <ID>.json нет (с предупреждением)
        3) FALLBACK_CONFIG — если и legacy нет (с предупреждением)
    Ничего не создаёт и не перезаписывает.
    """
    on_disk = set(list_robot_configs())
    legacy = None
    out = {}
    missing = []

    for rid in robot_ids:
        if rid in on_disk:
            out[rid] = load_robot_config(rid)
            continue
        missing.append(rid)
        if legacy is None:
            legacy = _legacy_cfg_or_none()
        if legacy is not None:
            print(f"[config] {rid}: нет robot_configs/{rid}.json, "
                  f"использую legacy {LEGACY_CONFIG_FILE} (только для этого запуска)")
            out[rid] = {**legacy, "robot_id": rid}
        else:
            print(f"[config] {rid}: нет robot_configs/{rid}.json и нет {LEGACY_CONFIG_FILE}, "
                  f"использую встроенные значения по умолчанию")
            out[rid] = {**FALLBACK_CONFIG, "robot_id": rid}

    if missing:
        print()
        print("!!! ВНИМАНИЕ: для следующих роботов нет персонального конфига:")
        for rid in missing:
            print(f"    {rid}")
        print("    Запустите:  python robot_config_store.py --scan")
        print("    (создаст недостающие конфиги; существующие не тронет)")
        print()
    return out


# ==================== МАТЕМАТИКА ====================

def wheel_speeds(vf, vl, w, angles):
    u = [vl * math.cos(a) - vf * math.sin(a) + w for a in angles]
    m = max(1.0, max(abs(x) for x in u))
    return [x / m for x in u]


def calibrate(u, cfg):
    """
    Скорость колеса (доля -1..1) -> напряжение на мотор с учётом gain / dz / inv.

    Порог страгивания dz — минимальная доля ШИМ (0..1), с которой мотор
    уверенно крутится. У ваших моторов это ~180/255 ≈ 0.706.

    Компенсация мёртвой зоны:
        out = sign(x) * (dz + (1 - dz) * |x|)
    При |x| = 0   -> 0      (мотор стоит)
    При |x| = 1   -> 1      (полная скорость)
    При малых |x| -> ~dz    (мотор уже крутится, а не «залипает»)
    Пропорции между колёсами сохраняются: если одному нужно 0.5, а другому 1.0,
    после компенсации будет ~0.85 и 1.0 — направление не инвертируется.

    Если слабому мотору (gain < 1) после деления на gain нужно больше 100%,
    масштабируем ВСЕ колёса вместе, чтобы направление движения не искажалось.
    """
    motors = [cfg["motors"][str(i + 1)] for i in range(3)]

    # 1) Учёт gain: слабый мотор получает больше «виртуальной» скорости
    v = [x / m["gain"] for x, m in zip(u, motors)]

    # 2) Если кто-то просит > 1 — нормируем все колёса одинаково
    k = max(1.0, max(abs(x) for x in v))
    v = [x / k for x in v]

    # 3) Компенсация мёртвой зоны + инверсия
    out = []
    for x, m in zip(v, motors):
        dz = float(m.get("dz", 0.0))
        dz = max(0.0, min(0.95, dz))          # защита от dz >= 1

        if abs(x) < 1e-6:
            y = 0.0                            # мотор стоит
        else:
            # sign(x) * (dz + (1 - dz) * |x|)
            y = math.copysign(dz + (1.0 - dz) * abs(x), x)

        if m["inv"]:
            y = -y
        out.append(y)
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
    pwm = [max(-255, min(255, round(x * 255))) for x in volts]
    return "M:" + ",".join(str(x) for x in pwm) + f",{cfg['command_ms']}"


# ==================== ВЫБОР РОБОТОВ ====================

def choose_targets(cfg_ids_from_argv=None, all_flag=False):
    """
    Возвращает {rid: (ip, port)} — выбранные роботы, или {} если ESC.

    Логика:
      * --all  -> все найденные
      * argv   -> только указанные (если не найдены в сети — предупреждаем)
      * иначе  -> интерактивно: показать найденных, спросить номера через запятую
                  ('a' = все; пусто = ждать ещё; ESC = выход)
    """
    while not keyboard.is_pressed("esc"):
        found = find_robots()
        if not found:
            print("[discovery] Роботов не видно, жду маяк... (ESC — выход)")
            continue

        ids = sorted(found)

        # ---- явный список из argv ----
        if cfg_ids_from_argv:
            chosen = {}
            for rid in cfg_ids_from_argv:
                rid = rid.upper()
                if rid in found:
                    chosen[rid] = found[rid]
                else:
                    print(f"[discovery] {rid} не найден в сети")
            if chosen:
                return chosen
            print("[discovery] Никто из указанных не найден, жду...")
            continue

        # ---- --all ----
        if all_flag:
            print(f"[discovery] Подключаюсь ко всем ({len(found)}):")
            for rid in ids:
                print(f"    {rid}  {found[rid][0]}:{found[rid][1]}")
            return found

        # ---- интерактивно ----
        print(f"[discovery] Найдено роботов: {len(found)}")
        for i, rid in enumerate(ids, 1):
            print(f"  {i}) {rid}  {found[rid][0]}:{found[rid][1]}")
        print("  Введите номера через запятую (например 1,3), 'a' — все, "
              "Enter — обновить список, ESC — выход")
        try:
            raw = input("Выбор: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return {}

        if raw == "":
            continue
        if raw == "a":
            return found
        chosen = {}
        try:
            for token in raw.split(","):
                idx = int(token) - 1
                rid = ids[idx]
                chosen[rid] = found[rid]
        except (ValueError, IndexError):
            print("[discovery] Некорректный выбор, попробуйте ещё раз")
            continue
        if chosen:
            return chosen

    return {}


# ==================== ОБЩЕНИЕ ====================

_last_printed = None


def send_to_all(sock, targets, pkt, print_once=True):
    """Отправляет pkt всем targets. Печатает команду один раз (если изменилась)."""
    global _last_printed
    if print_once and pkt != _last_printed:
        print(f"-> {pkt}  (x{len(targets)})")
        _last_printed = pkt
    data = pkt.encode()
    for ip, port in targets.values():
        sock.sendto(data, (ip, port))


def drain(sock):
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


def push_pins_one(sock, ip, port, cfg, tries=5):
    pins = [p for i in "123" for p in cfg["motors"][i]["pins"]]
    pkt = ("PINS:" + ",".join(map(str, pins))).encode()
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
                return True
            if text.startswith("ERR"):
                print(f"[config] {ip}:{port} отклонил пины: {text}")
                return False
    print(f"[config] {ip}:{port} не ответил на PINS")
    return False


def push_pins_all(sock, targets, configs):
    for rid, (ip, port) in targets.items():
        ok = push_pins_one(sock, ip, port, configs[rid])
        print(f"[config] {rid}: пины {'установлены' if ok else 'НЕ установлены'}")


# ==================== ТЕСТ МОТОРОВ ====================

def hold(sock, targets, pkt, seconds):
    t_end = time.time() + seconds
    while time.time() < t_end:
        if keyboard.is_pressed("esc") or keyboard.is_pressed("space"):
            send_to_all(sock, targets, "STOP")
            print("[test] Прервано")
            return False
        send_to_all(sock, targets, pkt)
        drain(sock)
        time.sleep(SEND_INTERVAL)
    return True


def run_motor_test(sock, targets, cfgs):
    """
    Тест моторов у всех выбранных роботов сразу. У каждого робота — свой
    test_pwm и command_ms из его конфига.
    """
    print(f"[test] Тест моторов у {len(targets)} робот(ов)")
    for n in range(3):
        for sign, name in ((1, "вперёд"), (-1, "назад")):
            print(f"[test] Мотор {n + 1}: {name}")
            # у каждого робота свой пакет (test_pwm / command_ms могут отличаться)
            for rid, (ip, port) in targets.items():
                cfg = cfgs[rid]
                vals = [0, 0, 0]
                vals[n] = sign * int(cfg["test_pwm"])
                pkt = f"M:{vals[0]},{vals[1]},{vals[2]},{cfg['command_ms']}"
                sock.sendto(pkt.encode(), (ip, port))
            t_end = time.time() + TEST_SECONDS
            while time.time() < t_end:
                if keyboard.is_pressed("esc") or keyboard.is_pressed("space"):
                    send_to_all(sock, targets, "STOP")
                    print("[test] Прервано")
                    return
                # повторяем каждому его собственный пакет
                for rid, (ip, port) in targets.items():
                    cfg = cfgs[rid]
                    vals = [0, 0, 0]
                    vals[n] = sign * int(cfg["test_pwm"])
                    sock.sendto(f"M:{vals[0]},{vals[1]},{vals[2]},{cfg['command_ms']}".encode(),
                                (ip, port))
                drain(sock)
                time.sleep(SEND_INTERVAL)
            if not hold(sock, targets, "STOP", TEST_PAUSE):
                return
    print("[test] Тест завершён")


# ==================== УПРАВЛЕНИЕ ====================

def control_loop(sock, targets, cfgs):
    """
    Один цикл — одна логическая команда. Пакеты считаются по конфигу каждого
    робота (у них могут быть разные drive_speed/turn_speed/command_ms),
    но клавиши читаются один раз, поэтому движение синхронное.
    """
    global _last_printed
    _last_printed = None
    angles = {rid: [math.radians(a) for a in cfgs[rid]["wheel_angles_deg"]]
              for rid in targets}
    print(HELP)
    print(f"[main] Активных роботов: {len(targets)}: {', '.join(sorted(targets))}")
    last_ok = time.time()

    while True:
        if keyboard.is_pressed("esc"):
            return "exit"

        if keyboard.is_pressed("t"):
            run_motor_test(sock, targets, cfgs)
            while keyboard.is_pressed("t"):
                time.sleep(0.05)
            last_ok = time.time()
            continue

        # Один и тот же набор клавиш -> пакет для каждого робота по его конфигу
        for rid, (ip, port) in targets.items():
            pkt = compute_packet(cfgs[rid], angles[rid])
            sock.sendto(pkt.encode(), (ip, port))

        # Печать: если у всех одинаковый пакет — печатаем один раз;
        # если разные (редко) — печатаем по каждому.
        pkts = {rid: compute_packet(cfgs[rid], angles[rid]) for rid in targets}
        uniq = set(pkts.values())
        if len(uniq) == 1:
            pkt = next(iter(uniq))
            if pkt != _last_printed:
                print(f"-> {pkt}  (x{len(targets)})")
                _last_printed = pkt
        else:
            for rid, pkt in pkts.items():
                print(f"-> [{rid}] {pkt}")
            _last_printed = None

        if drain(sock):
            last_ok = time.time()

        if time.time() - last_ok > 2.0:
            print("[robot] Связь потеряна у всех")
            return "lost"
        time.sleep(SEND_INTERVAL)


# ==================== MAIN ====================

def parse_argv(argv):
    all_flag = False
    ids = []
    for a in argv[1:]:
        if a == "--all":
            all_flag = True
        elif a == "--list":
            return {"list": True}
        elif a in ("-h", "--help"):
            return {"help": True}
        elif not a.startswith("-"):
            ids.append(a.upper())
    return {"all": all_flag, "ids": ids}


def main():
    opts = parse_argv(sys.argv)

    if opts.get("help"):
        print(__doc__)
        return
    if opts.get("list"):
        ids = list_robot_configs()
        print(f"Конфигов на диске: {len(ids)}")
        for rid in ids:
            print(f"  {rid}")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    targets = {}
    cfgs = {}

    try:
        while True:
            targets = choose_targets(opts.get("ids"), opts.get("all"))
            if not targets:
                break

            # --- проверка «есть ли конфиг у каждой платы» ---
            on_disk = set(list_robot_configs())
            online_ids = set(targets)
            without_cfg = sorted(online_ids - on_disk)
            if without_cfg:
                print()
                print("!!! Платы в сети БЕЗ персонального конфига:")
                for rid in without_cfg:
                    print(f"    {rid}")
                print("    Запустите:  python robot_config_store.py --scan")
                print()

            # --- загрузка конфигов (без создания/перезаписи) ---
            cfgs = resolve_configs(sorted(targets))

            print(f"[main] Роботы: {', '.join(sorted(targets))}")
            push_pins_all(sock, targets, cfgs)

            result = control_loop(sock, targets, cfgs)
            if result == "exit":
                break
            # "lost" -> снова выбор роботов
    except KeyboardInterrupt:
        pass
    finally:
        if targets:
            for _ in range(3):
                for ip, port in targets.values():
                    sock.sendto(b"STOP", (ip, port))
                time.sleep(0.03)
        sock.close()
        print("[main] Выход. Все роботы остановлены.")


if __name__ == "__main__":
    main()