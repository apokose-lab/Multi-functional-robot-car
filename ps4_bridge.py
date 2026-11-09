import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import socket
import struct
import time
import pygame
import serial
import serial.tools.list_ports

# 1. Auto-detect ESP32-S3 USB Serial port
ser = None
for port in serial.tools.list_ports.comports():
    if port.vid == 0x303A or "USB JTAG" in port.description or "CH340" in port.description or "CP210" in port.description:
        try:
            ser = serial.Serial(port.device, 115200, timeout=0, write_timeout=0)
            ser.dtr = True
            ser.rts = True
            print(f"[USB] Connected to ESP32-S3 on {port.device}")
            break
        except Exception:
            pass

# 2. Non-blocking Wi-Fi UDP socket
UDP_IP = "192.168.4.1"
UDP_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

pygame.init()
pygame.joystick.init()

screen = pygame.display.set_mode((1280, 760), pygame.RESIZABLE)
pygame.display.set_caption("Mecanum Robot - PID & Ultrasonic Telemetry Dashboard")

font_h1    = pygame.font.SysFont("Segoe UI", 26, bold=True)
font_h2    = pygame.font.SysFont("Segoe UI", 20, bold=True)
font_body  = pygame.font.SysFont("Consolas", 18, bold=True)
font_small = pygame.font.SysFont("Consolas", 15, bold=True)

MAX_PWM = 220
DEADZONE = 0.12
pid_enable = 0
prev_y_btn = False

rx_pid_mode = 0
rx_vx, rx_vy, rx_omega = 0, 0, 0
rx_target = [0, 0, 0, 0]
rx_pwm    = [0, 0, 0, 0]
rx_rpm    = [0, 0, 0, 0]
rx_us     = [200, 200, 200]
rx_motion_alive = 0
rx_uptime = 0
last_rx_time = 0.0
conn_source = "OFFLINE"

TELE_FMT = "<BBB15h3HBIB"
TELE_SIZE = struct.calcsize(TELE_FMT) # 45 bytes
serial_buf = bytearray()

def apply_deadzone(val):
    return 0.0 if abs(val) < DEADZONE else val

def get_direction_label(vx, vy, omega, test_w):
    if test_w > 0:
        return f"CALIBRATING MOTOR M{test_w} (+150 PWM)"
    if abs(vx) < 10 and abs(vy) < 10 and abs(omega) < 10:
        return "IDLE (STOPPED)"
    if abs(omega) >= max(abs(vx), abs(vy)):
        return "ROTATE LEFT (CCW)" if omega > 0 else "ROTATE RIGHT (CW)"
    if abs(vx) > 25 and abs(vy) > 25:
        if vx > 0 and vy > 0: return "DIAG FRONT-LEFT"
        if vx > 0 and vy < 0: return "DIAG FRONT-RIGHT"
        if vx < 0 and vy > 0: return "DIAG REAR-LEFT"
        return "DIAG REAR-RIGHT"
    if abs(vx) >= abs(vy):
        return "MOVE FORWARD" if vx > 0 else "MOVE BACKWARD"
    return "STRAFE LEFT" if vy > 0 else "STRAFE RIGHT"

def draw_badge(surf, x, y, w, h, text, bg_color):
    pygame.draw.rect(surf, bg_color, (x, y, w, h), border_radius=8)
    txt = font_small.render(text, True, (255, 255, 255))
    surf.blit(txt, (x + (w - txt.get_width()) // 2, y + (h - txt.get_height()) // 2))

def draw_dual_bar(surf, x, y, w, h, title, target_v, rpm_v, pwm_v, is_pid, highlight):
    border_col = (241, 196, 15) if highlight else (55, 65, 82)
    pygame.draw.rect(surf, (33, 39, 50), (x, y, w, h), border_radius=10)
    pygame.draw.rect(surf, border_col, (x, y, w, h), 3 if highlight else 2, border_radius=10)

    unit_str = "RPM" if is_pid else "PWM"
    header_str = f"{title}   |   Target: {target_v:4d} {unit_str}   |   Encoder: {rpm_v:4d} RPM   |   Out: {pwm_v:4d} PWM"
    surf.blit(font_body.render(header_str, True, (240, 246, 252)), (x + 16, y + 10))

    bar_x = x + 16
    bar_w = w - 32
    mid_x = bar_x + bar_w // 2

    # Bar 1: Encoder RPM
    y_rpm = y + 42
    pygame.draw.rect(surf, (22, 26, 34), (bar_x, y_rpm, bar_w, 22), border_radius=5)
    rpm_ratio = max(-1.0, min(1.0, rpm_v / 150.0))
    rw = int((bar_w // 2) * abs(rpm_ratio))
    if rpm_v >= 0:
        pygame.draw.rect(surf, (52, 152, 219), (mid_x, y_rpm + 2, rw, 18), border_radius=4)
    else:
        pygame.draw.rect(surf, (155, 89, 182), (mid_x - rw, y_rpm + 2, rw, 18), border_radius=4)
    pygame.draw.line(surf, (140, 150, 165), (mid_x, y_rpm), (mid_x, y_rpm + 22), 2)
    surf.blit(font_small.render("ENCODER (RPM)", True, (170, 185, 205)), (bar_x + 8, y_rpm + 2))

    # Bar 2: Motor PWM
    y_pwm = y + 70
    pygame.draw.rect(surf, (22, 26, 34), (bar_x, y_pwm, bar_w, 22), border_radius=5)
    pwm_ratio = max(-1.0, min(1.0, pwm_v / 255.0))
    pw = int((bar_w // 2) * abs(pwm_ratio))
    if pwm_v >= 0:
        pygame.draw.rect(surf, (46, 204, 113), (mid_x, y_pwm + 2, pw, 18), border_radius=4)
    else:
        pygame.draw.rect(surf, (230, 126, 34), (mid_x - pw, y_pwm + 2, pw, 18), border_radius=4)
    pygame.draw.line(surf, (140, 150, 165), (mid_x, y_pwm), (mid_x, y_pwm + 22), 2)
    surf.blit(font_small.render("MOTOR (PWM)", True, (170, 185, 205)), (bar_x + 8, y_pwm + 2))

def us_color(cm):
    if cm < 20: return (231, 76, 60)
    if cm < 45: return (241, 196, 15)
    return (46, 204, 113)

def parse_telemetry_bytes(data, source_name):
    global rx_pid_mode, rx_vx, rx_vy, rx_omega, rx_target, rx_pwm, rx_rpm, rx_us
    global rx_motion_alive, rx_uptime, last_rx_time, conn_source
    unpacked = struct.unpack(TELE_FMT, data)
    rx_pid_mode = unpacked[2]
    rx_vx, rx_vy, rx_omega = unpacked[3], unpacked[4], unpacked[5]
    rx_target = list(unpacked[6:10])
    rx_pwm    = list(unpacked[10:14])
    rx_rpm    = list(unpacked[14:18])
    rx_us     = list(unpacked[18:21])
    rx_motion_alive = unpacked[21]
    rx_uptime = unpacked[22]
    last_rx_time = time.time()
    conn_source = source_name

running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
            pid_enable = 1 - pid_enable

    # Check keyboard keys 1, 2, 3, 4 for single-wheel calibration
    keys = pygame.key.get_pressed()
    test_wheel = 0
    if keys[pygame.K_1]: test_wheel = 1
    elif keys[pygame.K_2]: test_wheel = 2
    elif keys[pygame.K_3]: test_wheel = 3
    elif keys[pygame.K_4]: test_wheel = 4

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

        y_btn = bool(js.get_button(3)) if js.get_numbuttons() > 3 else False
        if y_btn and not prev_y_btn:
            pid_enable = 1 - pid_enable
        prev_y_btn = y_btn
    else:
        vx, vy, omega, buzzer = 0, 0, 0, 0

    # 1. Send Control Packet (9 bytes)
    if ser and ser.is_open:
        try:
            ser.write(struct.pack('<BhhhBBB', 0xFE, vx, vy, omega, buzzer, pid_enable, test_wheel))
        except Exception:
            pass
    try:
        sock.sendto(struct.pack('<hhhBBB', vx, vy, omega, buzzer, pid_enable, test_wheel), (UDP_IP, UDP_PORT))
    except Exception:
        pass

    # 2A. Receive Telemetry via Wi-Fi UDP
    while True:
        try:
            data, _ = sock.recvfrom(256)
            if len(data) == TELE_SIZE and data[0] == 0xCC and data[1] == 0x77:
                parse_telemetry_bytes(data, "WI-FI UDP")
        except Exception:
            break

    # 2B. Receive Telemetry via USB Serial
    if ser and ser.is_open:
        try:
            if ser.in_waiting > 0:
                serial_buf.extend(ser.read(ser.in_waiting))
                while len(serial_buf) >= TELE_SIZE:
                    if serial_buf[0] == 0xCC and serial_buf[1] == 0x77:
                        pkt_bytes = bytes(serial_buf[:TELE_SIZE])
                        del serial_buf[:TELE_SIZE]
                        parse_telemetry_bytes(pkt_bytes, f"USB ({ser.port})")
                    else:
                        del serial_buf[0]
        except Exception:
            pass

    is_online = (time.time() - last_rx_time) < 0.7

    # 3. Render Dashboard UI
    W, H = screen.get_size()
    screen.fill((18, 22, 29))

    pygame.draw.rect(screen, (26, 32, 44), (0, 0, W, 68))
    screen.blit(font_h1.render("MECANUM ROBOT TELEMETRY & PID DASHBOARD", True, (88, 166, 255)), (24, 18))

    badge_y = 18
    conn_txt = f"LINK: {conn_source} ({rx_uptime // 1000}s)" if is_online else "LINK: OFFLINE"
    draw_badge(screen, W - 650, badge_y, 230, 34, conn_txt, (39, 174, 96) if is_online else (192, 57, 43))

    uart_ok = is_online and (rx_motion_alive == 1)
    draw_badge(screen, W - 405, badge_y, 195, 34, "UART 2350: OK" if uart_ok else "UART 2350: WAIT", (41, 128, 185) if uart_ok else (211, 84, 0))

    mode_txt = "MODE: PID (RPM)" if pid_enable else "MODE: DIRECT PWM"
    draw_badge(screen, W - 195, badge_y, 175, 34, mode_txt, (142, 68, 173) if pid_enable else (44, 62, 80))

    left_x = 24
    left_w = int(W * 0.58) - 36
    right_x = left_x + left_w + 24
    right_w = W - right_x - 24

    # --- LEFT COLUMN ---
    pygame.draw.rect(screen, (33, 39, 50), (left_x, 86, left_w, 115), border_radius=10)
    dir_label = get_direction_label(vx, vy, omega, test_wheel)
    screen.blit(font_h2.render(f"MOTION STATE: {dir_label}", True, (255, 215, 0)), (left_x + 18, 96))
    screen.blit(font_body.render(f"Gamepad -> Vx (Fwd): {vx:4d}  |  Vy (Strafe): {vy:4d}  |  Omega (Yaw): {omega:4d}", True, (220, 230, 242)), (left_x + 18, 132))
    screen.blit(font_small.render("Hold Keys [1][2][3][4]: Test M1..M4 (+150 PWM) | [Triangle/Key P]: Toggle PID | [Circle]: Buzzer", True, (140, 185, 220)), (left_x + 18, 166))

    wheel_titles = [
        "M1 (Front-Left  - FL)",
        "M2 (Front-Right - FR)",
        "M3 (Rear-Left   - RL)",
        "M4 (Rear-Right  - RR)"
    ]
    card_h = 104
    for i in range(4):
        cy = 218 + i * (card_h + 14)
        t_val = rx_target[i] if is_online else 0
        r_val = rx_rpm[i]    if is_online else 0
        p_val = rx_pwm[i]    if is_online else 0
        draw_dual_bar(screen, left_x, cy, left_w, card_h, wheel_titles[i], t_val, r_val, p_val, bool(pid_enable), test_wheel == (i + 1))

    # --- RIGHT COLUMN ---
    radar_h = H - 86 - 24
    pygame.draw.rect(screen, (33, 39, 50), (right_x, 86, right_w, radar_h), border_radius=10)
    pygame.draw.rect(screen, (55, 65, 82), (right_x, 86, right_w, radar_h), 2, border_radius=10)
    screen.blit(font_h2.render("ULTRASONIC PROXIMITY RADAR (3x SR-04)", True, (88, 166, 255)), (right_x + 20, 102))

    cx = right_x + right_w // 2
    cy = 86 + radar_h // 2 + 10
    car_w, car_h = 120, 170

    f_col = us_color(rx_us[0])
    pygame.draw.polygon(screen, f_col, [(cx - 25, cy - car_h//2 - 8), (cx + 25, cy - car_h//2 - 8), (cx + 70, cy - car_h//2 - 95), (cx - 70, cy - car_h//2 - 95)], 3)
    draw_badge(screen, cx - 85, cy - car_h//2 - 140, 170, 36, f"FRONT: {rx_us[0]:3d} cm", f_col)

    l_col = us_color(rx_us[1])
    pygame.draw.polygon(screen, l_col, [(cx - car_w//2 - 8, cy - 25), (cx - car_w//2 - 8, cy + 25), (cx - car_w//2 - 85, cy + 65), (cx - car_w//2 - 85, cy - 65)], 3)
    draw_badge(screen, cx - car_w//2 - 190, cy - 18, 150, 36, f"LEFT: {rx_us[1]:3d} cm", l_col)

    r_col = us_color(rx_us[2])
    pygame.draw.polygon(screen, r_col, [(cx + car_w//2 + 8, cy - 25), (cx + car_w//2 + 8, cy + 25), (cx + car_w//2 + 85, cy + 65), (cx + car_w//2 + 85, cy - 65)], 3)
    draw_badge(screen, cx + car_w//2 + 40, cy - 18, 150, 36, f"RIGHT: {rx_us[2]:3d} cm", r_col)

    pygame.draw.rect(screen, (60, 72, 92), (cx - car_w//2, cy - car_h//2, car_w, car_h), border_radius=14)
    pygame.draw.rect(screen, (140, 160, 190), (cx - car_w//2, cy - car_h//2, car_w, car_h), 3, border_radius=14)
    screen.blit(font_small.render("FRONT", True, (255, 215, 0)), (cx - 24, cy - car_h//2 + 12))

    wheel_coords = [
        (cx - car_w//2 - 22, cy - car_h//2 + 15),
        (cx + car_w//2 + 4,  cy - car_h//2 + 15),
        (cx - car_w//2 - 22, cy + car_h//2 - 60),
        (cx + car_w//2 + 4,  cy + car_h//2 - 60)
    ]
    for i, (wx, wy) in enumerate(wheel_coords):
        w_col = (100, 110, 125)
        if rx_pwm[i] > 10: w_col = (46, 204, 113)
        elif rx_pwm[i] < -10: w_col = (230, 126, 34)
        pygame.draw.rect(screen, w_col, (wx, wy, 18, 45), border_radius=4)

    legend_y = 86 + radar_h - 42
    screen.blit(font_small.render("Proximity Alert:   [Red < 20cm]     [Yellow 20-45cm]     [Green > 45cm]", True, (180, 195, 215)), (right_x + 24, legend_y))

    pygame.display.flip()
    clock.tick(35)

if ser:
    ser.close()
pygame.quit()