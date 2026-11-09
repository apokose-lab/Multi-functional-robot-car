import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import socket
import struct
import time
import pygame
import serial
import serial.tools.list_ports

# 1. Tự động tìm cổng COM USB của ESP32-S3 (nếu đang cắm cáp)
ser = None
for port in serial.tools.list_ports.comports():
    if port.vid == 0x303A or "USB JTAG" in port.description or "CH340" in port.description or "CP210" in port.description:
        try:
            ser = serial.Serial(port.device, 115200, timeout=0, write_timeout=0)
            ser.dtr = True
            ser.rts = True
            print(f"=> [USB] Đã kết nối với ESP32-S3 tại {port.device}!")
            break
        except Exception:
            pass

# 2. Đường truyền Wi-Fi UDP
UDP_IP = "192.168.4.1"
UDP_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

pygame.init()
pygame.joystick.init()
pygame.event.pump()

if pygame.joystick.get_count() == 0:
    print("[LOI] Chua tim thay tay cam!")
    exit()

js = pygame.joystick.Joystick(0)
js.init()
print(f"=== DA KET NOI TAY CAM: {js.get_name()} ===")

MAX_PWM = 200
DEADZONE = 0.12

def apply_deadzone(val):
    return 0.0 if abs(val) < DEADZONE else val

def constrain(val, min_val, max_val):
    return max(min_val, min(max_val, val))

try:
    while True:
        pygame.event.pump()

        # Đọc giá trị từ 2 cần gạt
        raw_vx = -apply_deadzone(js.get_axis(1))
        raw_vy = -apply_deadzone(js.get_axis(0))
        raw_omega = -apply_deadzone(js.get_axis(2))

        vx = int(raw_vx * MAX_PWM)
        vy = int(raw_vy * MAX_PWM)
        omega = int(raw_omega * MAX_PWM)

        if js.get_button(0):
            vx, vy, omega = 0, 0, 0
        buzzer = 1 if js.get_button(1) else 0

        # Tính toán PWM thực tế của từng bánh (giống hệt công thức trên ESP32-S3)
        fl = constrain(vx - vy - omega, -255, 255) # M1: Trái Trước (Front-Left)
        fr = constrain(vx + vy + omega, -255, 255) # M2: Phải Trước (Front-Right)
        rl = constrain(vx + vy - omega, -255, 255) # M3: Trái Sau   (Rear-Left)
        rr = constrain(vx - vy + omega, -255, 255) # M4: Phải Sau   (Rear-Right)

        # Gửi qua USB (nếu có cắm dây)
        if ser and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    ser.read(ser.in_waiting)
                ser.write(struct.pack('<BhhhB', 0xFE, vx, vy, omega, buzzer))
            except Exception:
                pass

        # Gửi qua Wi-Fi UDP
        try:
            sock.sendto(struct.pack('<hhhB', vx, vy, omega, buzzer), (UDP_IP, UDP_PORT))
        except Exception:
            pass

        # In chi tiết cả vận tốc tổng và PWM của từng động cơ trên cùng 1 dòng
        print(
            f"Input[Vx:{vx:4d} Vy:{vy:4d} W:{omega:4d}] ==> "
            f"PWM[ M1(FL):{fl:4d} | M2(FR):{fr:4d} | M3(RL):{rl:4d} | M4(RR):{rr:4d} ] Còi:{buzzer}  ",
            end="\r",
            flush=True
        )
        time.sleep(0.02)
except KeyboardInterrupt:
    if ser:
        ser.close()
    print("\nĐã dừng chương trình điều khiển.")