import socket
import keyboard
import time
import threading

# ==================== НАСТРОЙКИ ====================
ROBOT_IP = "192.168.1.42"
ROBOT_PORT = 8888
SEND_INTERVAL = 0.05          # 20 Гц

BASE_SPEED = 200
TURN_SPEED = 150
DURATION_MS = 500             # время действия команды в мс

running = True
last_packet = "M1:0;M2:0;M3:0;T:0"

lock = threading.Lock()

def build_packet(s1, s2, s3, duration=DURATION_MS):
    return f"M1:{s1};M2:{s2};M3:{s3};T:{duration}"

def sender():
    global last_packet
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while running:
        with lock:
            pkt = last_packet
        try:
            sock.sendto(pkt.encode(), (ROBOT_IP, ROBOT_PORT))
        except Exception:
            pass
        time.sleep(SEND_INTERVAL)

threading.Thread(target=sender, daemon=True).start()

print("--- Управление роботом ---")
print(f"Робот: {ROBOT_IP}:{ROBOT_PORT}")
print("W/S/A/D — движение, Пробел — стоп, ESC — выход")

try:
    while True:
        with lock:
            if keyboard.is_pressed('w'):
                last_packet = build_packet(BASE_SPEED, BASE_SPEED, BASE_SPEED)
            elif keyboard.is_pressed('s'):
                last_packet = build_packet(-BASE_SPEED, -BASE_SPEED, -BASE_SPEED)
            elif keyboard.is_pressed('a'):
                last_packet = build_packet(-TURN_SPEED, TURN_SPEED, TURN_SPEED)
            elif keyboard.is_pressed('d'):
                last_packet = build_packet(TURN_SPEED, -TURN_SPEED, TURN_SPEED)
            elif keyboard.is_pressed('space'):
                last_packet = "M1:0;M2:0;M3:0;T:0"
        if keyboard.is_pressed('esc'):
            running = False
            break
        time.sleep(0.02)
except KeyboardInterrupt:
    running = False

print("Выход...")