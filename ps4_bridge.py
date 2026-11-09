import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import socket
import struct
import time
import pygame

# Cấu hình mạng Wi-Fi UDP tới ESP32-S3
UDP_IP = "192.168.4.1"
UDP_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

pygame.init()
pygame.joystick.init()

# Khởi tạo cửa sổ Dashboard trực quan
WIDTH, HEIGHT = 1080, 720
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Mecanum Robot - Wireless Telemetry Dashboard")
font_title = pygame.font.SysFont("Consolas", 20, bold=True)
font_main  = pygame.font.SysFont("Consolas", 16, bold=True)
font_small = pygame.font.SysFont("Consolas", 14)

MAX_PWM = 200
DEADZONE = 0.12

def apply_deadzone(val):
    return 0.0 if abs(val) < DEADZONE else val

def get_direction_label(vx, vy, omega):
    if abs(vx) < 10 and abs(vy) < 10 and abs(omega) < 10:
        return "DUNG YEN (IDLE)"
    if abs(omega) >= max(abs(vx), abs(vy)):
        return "XOAY TRAI (CCW)" if omega > 0 else "XOAY PHAI (CW)"
    if abs(vx) > 20 and abs(vy) > 20:
        if vx > 0 and vy > 0: return "CHEO TRUOC - TRAI"
        if vx > 0 and vy < 0: return "CHEO TRUOC - PHAI"
        if vx < 0 and vy > 0: return "CHEO SAU - TRAI"
        return "CHEO SAU - PHAI"
    if abs(vx) >= abs(vy):
        return "TIEN THANG (FORWARD)" if vx > 0 else "LUI THANG (BACKWARD)"
    return "TRUOT NGANG TRAI" if vy > 0 else "TRUOT NGANG PHAI"

def draw_bar(surface, x, y, w, h, val, max_val, label):
    # Vẽ khung nền
    pygame.draw.rect(surface, (45, 50, 60), (x, y, w, h), border_radius=6)
    center_x = x + w // 2
    pygame.draw.line(surface, (120, 120, 130), (center_x, y), (center_x, y + h), 2)

    # Vẽ thanh giá trị PWM (-255 đến +255)
    ratio = max(-1.0, min(1.0, val / float(max_val)))
    bar_w = int((w // 2) * abs(ratio))
    if val > 0:
        pygame.draw.rect(surface, (46, 204, 113), (center_x, y + 4, bar_w, h - 8), border_radius=4)
        dir_str = "TIEN"
    elif val < 0:
        pygame.draw.rect(surface, (230, 126, 34), (center_x - bar_w, y + 4, bar_w, h - 8), border_radius=4)
        dir_str = "LUI "
    else:
        dir_str = "DUNG"

    txt = font_main.render(f"{label}: {val:4d} PWM [{dir_str}]", True, (240, 240, 240))
    surface.blit(txt, (x + 12, y + h // 2 - 8))

# Biến lưu dữ liệu phản hồi từ xe gửi về
rx_vx, rx_vy, rx_omega = 0, 0, 0
rx_pwm = [0, 0, 0, 0]
rx_rpm = [0, 0, 0, 0]
rx_motion_alive = 0
rx_uptime = 0
last_rx_time = 0.0

TELE_FORMAT = "<BBhhhhhhhhhhhBBI" # Khớp với struct WifiTelemetryPacket (29 bytes payload + 1 byte checksum)
TELE_SIZE = 30

running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Tự động nhận diện tay cầm
    js = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
    if js:
        raw_vx = -apply_deadzone(js.get_axis(1))
        raw_vy = -apply_deadzone(js.get_axis(0))
        raw_omega = -apply_deadzone(js.get_axis(2))

        vx = int(raw_vx * MAX_PWM)
        vy = int(raw_vy * MAX_PWM)
        omega = int(raw_omega * MAX_PWM)

        if js.get_button(0):
            vx, vy, omega = 0, 0, 0
        buzzer = 1 if js.get_button(1) else 0
    else:
        vx, vy, omega, buzzer = 0, 0, 0, 0

    # 1. Gửi gói điều khiển xuống xe qua Wi-Fi UDP
    try:
        tx_packet = struct.pack('<hhhB', vx, vy, omega, buzzer)
        sock.sendto(tx_packet, (UDP_IP, UDP_PORT))
    except Exception:
        pass

    # 2. Nhận gói Telemetry từ xe gửi trả về qua Wi-Fi UDP
    while True:
        try:
            data, addr = sock.recvfrom(256)
            if len(data) == TELE_SIZE and data[0] == 0xCC and data[1] == 0x77:
                unpacked = struct.unpack(TELE_FORMAT + "B", data)
                rx_vx, rx_vy, rx_omega = unpacked[2], unpacked[3], unpacked[4]
                rx_pwm = list(unpacked[5:9])
                rx_rpm = list(unpacked[9:13])
                rx_motion_alive = unpacked[14]
                rx_uptime = unpacked[15]
                last_rx_time = time.time()
        except BlockingIOError:
            break
        except Exception:
            break

    # Kiểm tra trạng thái kết nối phản hồi từ xe
    is_online = (time.time() - last_rx_time) < 0.5
    direction_text = get_direction_label(rx_vx if is_online else vx, rx_vy if is_online else vy, rx_omega if is_online else omega)

    # 3. Vẽ giao diện Dashboard
    screen.fill((24, 28, 36))

    # Thanh tiêu đề & Trạng thái kết nối
    title_surf = font_title.render("MECANUM ROBOT TELEMETRY DASHBOARD", True, (88, 166, 255))
    screen.blit(title_surf, (20, 15))

    status_color = (46, 204, 113) if is_online else (231, 76, 60)
    status_str = f"WIFI TELEMETRY: ONLINE (Uptime: {rx_uptime // 1000}s)" if is_online else "WIFI TELEMETRY: OFFLINE (Hay ket noi Wi-Fi Mecanum_Robot)"
    screen.blit(font_small.render(status_str, True, status_color), (20, 45))

    uart_color = (46, 204, 113) if (is_online and rx_motion_alive) else (241, 196, 15)
    uart_str = "UART (ESP32 <-> Motion 2350): OK" if (is_online and rx_motion_alive) else "UART (ESP32 <-> Motion 2350): WAITING..."
    screen.blit(font_small.render(uart_str, True, uart_color), (430, 45))

    # Khung hiển thị Vận tốc & Hướng di chuyển
    pygame.draw.rect(screen, (35, 40, 50), (20, 75, 680, 85), border_radius=8)
    screen.blit(font_main.render(f"HUONG DI CHUYEN: {direction_text}", True, (255, 215, 0)), (35, 88))
    vel_str = f"Van toc -> Vx (Tien/Lui): {vx:4d}  |  Vy (Ngang): {vy:4d}  |  Omega (Xoay): {omega:4d}"
    screen.blit(font_main.render(vel_str, True, (220, 220, 220)), (35, 122))

    # Khung hiển thị PWM thực tế của 4 bánh nhận về từ xe
    wheel_names = ["M1 (Trai Truoc - FL)", "M2 (Phai Truoc - FR)", "M3 (Trai Sau   - RL)", "M4 (Phai Sau   - RR)"]
    for i in range(4):
        y_pos = 180 + i * 62
        display_pwm = rx_pwm[i] if is_online else 0
        draw_bar(screen, 20, y_pos, 680, 48, display_pwm, 255, wheel_names[i])

    pygame.display.flip()

    # In tóm tắt trên Terminal
    print(f"[{'ONLINE' if is_online else 'OFFLINE'}] {direction_text:20s} | PWM: FL={rx_pwm[0]:4d} FR={rx_pwm[1]:4d} RL={rx_pwm[2]:4d} RR={rx_pwm[3]:4d}   ", end="\r", flush=True)
    clock.tick(50)

pygame.quit()