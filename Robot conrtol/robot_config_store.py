#!/usr/bin/env python3
"""
robot_config_store.py — конфиги роботов по их <id> (хвост MAC) + автообнаружение.

Режимы запуска:
    python robot_config_store.py                 # сканировать сеть, отчёт, создать недостающие
    python robot_config_store.py <ROBOT_ID>      # загрузить/создать конфиг конкретного робота
    python robot_config_store.py --list          # только показать, что уже есть на диске
    python robot_config_store.py --scan 5        # сканировать 5 секунд

Гарантии:
    * Существующий <ID>.json НИКОГДА не перезаписывается — только читается.
    * Новый файл создаётся атомарно и только если его ещё нет.
"""

import json
import os
import re
import socket
import sys
import tempfile
import time

CONFIG_DIR = "robot_configs"
DEFAULT_FILE = os.path.join(CONFIG_DIR, "_default.json")
BEACON_PORT = 8889

DEFAULT_CONFIG = {
    "_help_wheel_angles_deg": (
        "Где стоит каждое колесо вокруг центра робота. Отсчёт от направления вперёд "
        "против часовой стрелки (налево): 0 = спереди, 90 = слева, 180 = сзади, 270 = справа. "
        "Колесо 1 спереди слева (60), колесо 2 сзади (180), колесо 3 спереди справа (300)."
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

_ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")


# ==================== утилиты ====================

def _normalize_id(robot_id: str) -> str:
    if not isinstance(robot_id, str):
        raise TypeError("robot_id должен быть строкой")
    rid = robot_id.strip().upper()
    if not _ID_RE.match(rid):
        raise ValueError(f"Некорректный robot_id: {robot_id!r}")
    return rid


def _atomic_write_json_new(path: str, data: dict) -> None:
    """Пишет JSON атомарно. Гарантирует, что существующий файл не будет перезаписан."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path) or ".", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.write("\n")
        # os.link завершится ошибкой, если path уже существует -> атомарный «create if not exists»
        os.link(tmp, path)
    finally:
        try:
            os.unlink(tmp)
        except OSError:
            pass


def _config_path(robot_id: str) -> str:
    return os.path.join(CONFIG_DIR, f"{_normalize_id(robot_id)}.json")


# ==================== шаблон ====================

def load_default_config() -> dict:
    """Возвращает шаблон, создавая _default.json при необходимости."""
    if not os.path.exists(DEFAULT_FILE):
        _atomic_write_json_new(DEFAULT_FILE, DEFAULT_CONFIG)
        print(f"[config] Создан шаблон {DEFAULT_FILE}")
        return json.loads(json.dumps(DEFAULT_CONFIG))

    with open(DEFAULT_FILE, encoding="utf-8") as f:
        user = json.load(f)

    cfg = {**DEFAULT_CONFIG, **user}
    user_motors = user.get("motors", {})
    cfg["motors"] = {
        n: {**DEFAULT_CONFIG["motors"][n], **user_motors.get(n, {})}
        for n in ("1", "2", "3")
    }
    return cfg


# ==================== загрузка/создание ====================

def load_robot_config(robot_id: str) -> dict:
    """
    Загружает конфиг робота. Если файла нет — создаёт его из шаблона.
    Существующий файл НЕ перезаписывает.
    """
    rid = _normalize_id(robot_id)
    path = _config_path(rid)

    if not os.path.exists(path):
        template = load_default_config()
        template["robot_id"] = rid
        try:
            _atomic_write_json_new(path, template)
            print(f"[config] Создан {path} (robot_id={rid}) со стандартными значениями")
            return template
        except FileExistsError:
            # кто-то успел создать между проверкой и записью — просто читаем
            pass

    with open(path, encoding="utf-8") as f:
        user = json.load(f)

    if "alpha_deg" in user and "wheel_angles_deg" not in user:
        user["wheel_angles_deg"] = user["alpha_deg"]

    base = load_default_config()
    cfg = {**base, **user}
    user_motors = user.get("motors", {})
    cfg["motors"] = {
        n: {**base["motors"][n], **user_motors.get(n, {})}
        for n in ("1", "2", "3")
    }
    if not cfg.get("robot_id"):
        cfg["robot_id"] = rid
    return cfg


def list_robot_configs() -> list:
    """Список id, для которых уже есть файлы (без _default.json)."""
    if not os.path.isdir(CONFIG_DIR):
        return []
    out = []
    for name in os.listdir(CONFIG_DIR):
        if name.startswith("_") or not name.endswith(".json"):
            continue
        rid = name[:-5]
        if _ID_RE.match(rid):
            out.append(rid)
    return sorted(out)


# ==================== обнаружение роботов ====================

def find_robots(seconds: float = 3.0) -> dict:
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
                try:
                    port = int(p[5:])
                except ValueError:
                    pass
        if ip and _ID_RE.match(rid.upper()):
            found[rid.upper()] = (ip, port)
    sock.close()
    return found


# ==================== отчёт + автосоздание ====================

def report_and_create(scan_seconds: float = 3.0) -> dict:
    """
    Сканирует сеть, печатает отчёт, создаёт недостающие конфиги.
    Возвращает {id: (ip, port)} — всех, кого увидели.
    """
    print(f"[scan] Слушаю маяки {scan_seconds:.1f} с на порту {BEACON_PORT}...")
    seen = find_robots(scan_seconds)

    on_disk = set(list_robot_configs())
    online = set(seen)

    new_ids      = sorted(online - on_disk)          # в сети есть, конфига нет
    known_ids    = sorted(online & on_disk)          # в сети есть, конфиг есть
    offline_ids  = sorted(on_disk - online)          # в конфиге есть, в сети нет

    print()
    print("================ ОТЧЁТ ================")
    print(f"  Роботов в сети:        {len(online)}")
    print(f"  Конфигов на диске:     {len(on_disk)}")
    print()
    print(f"  Вижу и есть конфиг:    {len(known_ids)}")
    for rid in known_ids:
        print(f"      {rid:8s}  {seen[rid][0]}:{seen[rid][1]}")
    print(f"  Вижу, но конфига нет:  {len(new_ids)}")
    for rid in new_ids:
        print(f"      {rid:8s}  {seen[rid][0]}:{seen[rid][1]}")
    print(f"  Есть конфиг, нет в сети: {len(offline_ids)}")
    for rid in offline_ids:
        print(f"      {rid}")
    print("=======================================")
    print()

    created = 0
    for rid in new_ids:
        path = _config_path(rid)
        if os.path.exists(path):
            # между list_robot_configs и сейчас файл появился — не трогаем
            print(f"[config] {path} уже существует — не трогаю")
            continue
        load_robot_config(rid)   # создаст новый и напечатает
        created += 1

    if created:
        print(f"[config] Итого создано новых конфигов: {created}")
    else:
        print("[config] Новых конфигов не потребовалось")

    # важное предупреждение: конфиги offline-роботов не трогаем
    if offline_ids:
        print(f"[config] {len(offline_ids)} конфиг(ов) без робота в сети — оставлены как есть")

    return seen


# ==================== main ====================

def _usage() -> None:
    print(__doc__)


def main(argv: list) -> int:
    if len(argv) == 2 and argv[1] in ("-h", "--help"):
        _usage()
        return 0

    if len(argv) == 2 and argv[1] == "--list":
        ids = list_robot_configs()
        print(f"Конфигов на диске: {len(ids)}")
        for rid in ids:
            print(f"  {rid}")
        return 0

    if len(argv) >= 2 and argv[1] == "--scan":
        try:
            secs = float(argv[2]) if len(argv) >= 3 else 3.0
        except ValueError:
            print("--scan ожидает число секунд")
            return 2
        report_and_create(secs)
        return 0

    if len(argv) == 2 and not argv[1].startswith("-"):
        # режим одного робота
        cfg = load_robot_config(argv[1])
        print(json.dumps(cfg, indent=2, ensure_ascii=False))
        return 0

    # без аргументов — сканируем и создаём недостающие
    seen = report_and_create(3.0)
    if not seen:
        print("[scan] Роботов в сети не видно. Включите робота и повторите.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv))
    except KeyboardInterrupt:
        print("\n[main] Прервано пользователем")
        sys.exit(130)