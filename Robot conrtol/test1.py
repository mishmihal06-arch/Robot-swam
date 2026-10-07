import socket
import keyboard
import time
import threading

ROBOT_IP = ""   # ← замените на IP из Serial-монитора
ROBOT_PORT = 8888
SEND_INTERVAL = 0.05         # 20 Гц — удерживает Watchdog (300 мс) живым

current_cmd = "STOP"
running = True

def sender():
    global current_cmd
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while running:
        try:
            sock.sendto(current_cmd.encode(), (ROBOT_IP, ROBOT_PORT))
        except Exception as e:
            pass
        time.sleep(SEND_INTERVAL)

threading.Thread(target=sender, daemon=True).start()

print("--- Управление роботом (UDP) ---")
print("W - Вперед | S - Назад | A - Влево | D - Вправо | Пробел - Стоп | ESC - выход")

try:
    while True:
        if keyboard.is_pressed('w'):     current_cmd = "FWD"
        elif keyboard.is_pressed('s'):   current_cmd = "BACK"
        elif keyboard.is_pressed('a'):   current_cmd = "LEFT"
        elif keyboard.is_pressed('d'):   current_cmd = "RIGHT"
        elif keyboard.is_pressed('space'): current_cmd = "STOP"
        elif keyboard.is_pressed('esc'):
            running = False
            break
        time.sleep(0.01)
except KeyboardInterrupt:
    running = False

print("Выход...")
