import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import socket
import struct
import time
import pygame
import serial
import serial.tools.list_ports

# 1. Tìm đúng cổng USB của ESP32-S3 (Bỏ qua các cổng COM Bluetooth ảo)
ser = None
print("Dang quet cac cong COM tren may tinh:")
for port in serial.tools.list_ports.comports():
    print(f" - {port.device}: {port.description} (VID: {port.vid})")
    # VID 0x303A (12346) là mã của Espressif ESP32-S3, hoặc lọc các chip nạp CH340/CP210x
    if port.vid == 0x303A or "USB JTAG" in port.description or "CH340" in port.description or "CP210" in port.description:
        try:
            # QUAN TRỌNG: Phải có write_timeout=0 để không bao giờ bị treo hàm write()
            ser = serial.Serial(port.device, 115200, timeout=0, write_timeout=0)
            ser.dtr = True
            ser.rts = True
            print(f"=> [USB] Đã kết nối chuẩn với ESP32-S3 tại cổng {port.device}!")
            break
        except Exception as e:
            print(f"=> Không mở được {port.device}: {e}")

# 2. Đường truyền Wi-Fi UDP (cấu hình non-blocking)
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

try:
    while True:
        pygame.event.pump()

        # Cần gạt Trái: Axis 1 (Tiến/Lùi), Axis 0 (Trượt ngang)
        raw_vx = -apply_deadzone(js.get_axis(1))
        raw_vy = -apply_deadzone(js.get_axis(0))

        # Cần gạt Phải: Axis 2 (Xoay Trái/Phải trên Xbox 360 / DS4Windows)
        raw_omega = -apply_deadzone(js.get_axis(2))

        vx = int(raw_vx * MAX_PWM)
        vy = int(raw_vy * MAX_PWM)
        omega = int(raw_omega * MAX_PWM)

        # Nút A / X (Button 0) = Phanh | Nút B / O (Button 1) = Còi Buzzer
        if js.get_button(0):
            vx, vy, omega = 0, 0, 0
        buzzer = 1 if js.get_button(1) else 0

        # Gửi qua cáp USB và xả bộ đệm đọc để ESP32-S3 không bị nghẽn
        if ser and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    ser.read(ser.in_waiting) # Xả log từ ESP32-S3 gửi lên
                usb_packet = struct.pack('<BhhhB', 0xFE, vx, vy, omega, buzzer)
                ser.write(usb_packet)
            except Exception:
                pass

        # Gửi qua Wi-Fi UDP
        try:
            udp_packet = struct.pack('<hhhB', vx, vy, omega, buzzer)
            sock.sendto(udp_packet, (UDP_IP, UDP_PORT))
        except Exception:
            pass

        print(f"Đang gửi -> Vx: {vx:4d} | Vy: {vy:4d} | Omega: {omega:4d} | Còi: {buzzer}   ", end="\r", flush=True)
        time.sleep(0.02)
except KeyboardInterrupt:
    if ser:
        ser.close()
    print("\nĐã dừng chương trình điều khiển.")