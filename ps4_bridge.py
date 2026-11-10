import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import socket
import struct
import time
import pygame

UDP_IP = "192.168.4.1"
UDP_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

pygame.init()
pygame.joystick.init()

screen = pygame.display.set_mode((1280, 780), pygame.RESIZABLE)
pygame.display.set_caption("Mecanum Robot - PID, Encoder & Ultrasonic Diagnostic Dashboard")

font_h1    = pygame.font.SysFont("Segoe UI", 25, bold=True)
font_h2    = pygame.font.SysFont("Segoe UI", 19, bold=True)
font_body  = pygame.font.SysFont("Consolas", 17, bold=True)
font_small = pygame.font.SysFont("Consolas", 14, bold=True)

MAX_PWM = 220
DEADZONE = 0.12
pid_enable = 0
prev_y_btn = False

rx_pid_mode = 0
rx_vx, rx_vy, rx_omega = 0, 0, 0
rx_target = [0, 0, 0, 0]
rx_pwm    = [0, 0, 0, 0]
rx_rpm    = [0, 0, 0, 0]
rx_enc_h  = [0, 0, 0, 0]
rx_gpios  = 0
rx_us     = [0, 0, 0]
rx_us_ok  = 0
rx_motion_alive = 0
rx_uptime = 0
last_rx_time = 0.0

# 52-byte WifiTelemetryPacket format
TELE_FMT = "<3B15h4B4H2BIB"
TELE_SIZE = struct.calcsize(TELE_FMT)

SCAN_PIN_LABELS = ["GP4", "GP5", "GP6", "GP7", "GP18", "GP19", "GP20", "GP21", "GP26", "GP27", "GP28", "GP29"]
gpio_hold_time = [0.0] * 12

def apply_deadzone(val):
    return 0.0 if abs(val) < DEADZONE else val

def get_direction_label(vx, vy, omega, test_w):
    if test_w > 0:
        dpad_names = ["", "D-PAD UP -> M1 (FL)", "D-PAD RIGHT -> M2 (FR)", "D-PAD DOWN -> M3 (RL)", "D-PAD LEFT -> M4 (RR)"]
        return f"TESTING {dpad_names[test_w]} (+150 PWM)"
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

def get_enc_status_badge(pwm_v, health_bits):
    if health_bits == 3:
        return ("ENC: A+B OK", (39, 174, 96))
    if health_bits == 1:
        return ("WARN: WIRE B LOOSE!", (211, 84, 0))
    if health_bits == 2:
        return ("WARN: WIRE A LOOSE!", (211, 84, 0))
    if abs(pwm_v) > 30:
        return ("ERR: NO ENC SIGNAL!", (192, 57, 43))
    return ("ENC: STANDBY", (70, 80, 95))

def draw_dual_bar(surf, x, y, w, h, title, target_v, rpm_v, pwm_v, health_bits, is_pid, highlight):
    border_col = (241, 196, 15) if highlight else (55, 65, 82)
    pygame.draw.rect(surf, (33, 39, 50), (x, y, w, h), border_radius=10)
    pygame.draw.rect(surf, border_col, (x, y, w, h), 3 if highlight else 2, border_radius=10)

    unit_str = "RPM" if is_pid else "PWM"
    header_str = f"{title} | Target:{target_v:4d} {unit_str} | Enc:{rpm_v:4d} RPM | Out:{pwm_v:4d} PWM"
    surf.blit(font_body.render(header_str, True, (240, 246, 252)), (x + 14, y + 10))

    # Encoder Wire Health Badge on top-right of each motor card
    eb_text, eb_col = get_enc_status_badge(pwm_v, health_bits)
    draw_badge(surf, x + w - 185, y + 7, 172, 26, eb_text, eb_col)

    bar_x = x + 14
    bar_w = w - 28
    mid_x = bar_x + bar_w // 2

    # Bar 1: Encoder RPM
    y_rpm = y + 40
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
    y_pwm = y + 68
    pygame.draw.rect(surf, (22, 26, 34), (bar_x, y_pwm, bar_w, 22), border_radius=5)
    pwm_ratio = max(-1.0, min(1.0, pwm_v / 255.0))
    pw = int((bar_w // 2) * abs(pwm_ratio))
    if pwm_v >= 0:
        pygame.draw.rect(surf, (46, 204, 113), (mid_x, y_pwm + 2, pw, 18), border_radius=4)
    else:
        pygame.draw.rect(surf, (230, 126, 34), (mid_x - pw, y_pwm + 2, pw, 18), border_radius=4)
    pygame.draw.line(surf, (140, 150, 165), (mid_x, y_pwm), (mid_x, y_pwm + 22), 2)
    surf.blit(font_small.render("MOTOR (PWM)", True, (170, 185, 205)), (bar_x + 8, y_pwm + 2))

def us_style(cm, is_connected):
    if not is_connected:
        return (120, 130, 145), "DISCONNECTED"
    if cm < 20:
        return (231, 76, 60), f"{cm:3d} cm [DANGER]"
    if cm < 45:
        return (241, 196, 15), f"{cm:3d} cm [WARN]"
    if cm >= 400:
        return (46, 204, 113), ">400 cm [CLEAR]"
    return (46, 204, 113), f"{cm:3d} cm [OK]"

running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
            pid_enable = 1 - pid_enable

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

        # Toggle PID with Triangle / Y (Button 3)
        y_btn = bool(js.get_button(3)) if js.get_numbuttons() > 3 else False
        if y_btn and not prev_y_btn:
            pid_enable = 1 - pid_enable
        prev_y_btn = y_btn

        # Read D-Pad (Cross 4-direction buttons) for Single-Wheel Test
        # 1. Standard Hat 0 (Xbox 360 / DS4Windows)
        if js.get_numhats() > 0:
            hx, hy = js.get_hat(0)
            if hy == 1:    test_wheel = 1 # D-Pad UP    -> M1 (Front-Left)
            elif hx == 1:  test_wheel = 2 # D-Pad RIGHT -> M2 (Front-Right)
            elif hy == -1: test_wheel = 3 # D-Pad DOWN  -> M3 (Rear-Left)
            elif hx == -1: test_wheel = 4 # D-Pad LEFT  -> M4 (Rear-Right)

        # 2. Fallback if SDL maps D-Pad to Buttons 11, 12, 13, 14
        if test_wheel == 0 and js.get_numbuttons() >= 15:
            if js.get_button(11):   test_wheel = 1 # UP
            elif js.get_button(14): test_wheel = 2 # RIGHT
            elif js.get_button(12): test_wheel = 3 # DOWN
            elif js.get_button(13): test_wheel = 4 # LEFT
    else:
        vx, vy, omega, buzzer = 0, 0, 0, 0

    # 1. Send Control Packet via Wi-Fi UDP
    try:
        sock.sendto(struct.pack('<hhhBBB', vx, vy, omega, buzzer, pid_enable, test_wheel), (UDP_IP, UDP_PORT))
    except Exception:
        pass

    # 2. Receive Telemetry via Wi-Fi UDP
    now_t = time.time()
    while True:
        try:
            data, _ = sock.recvfrom(256)
            if len(data) == TELE_SIZE and data[0] == 0xCC and data[1] == 0x77:
                unpacked = struct.unpack(TELE_FMT, data)
                rx_pid_mode = unpacked[2]
                rx_vx, rx_vy, rx_omega = unpacked[3], unpacked[4], unpacked[5]
                rx_target = list(unpacked[6:10])
                rx_pwm    = list(unpacked[10:14])
                rx_rpm    = list(unpacked[14:18])
                rx_enc_h  = list(unpacked[18:22])
                rx_gpios  = unpacked[22]
                rx_us     = list(unpacked[23:26])
                rx_us_ok  = unpacked[26]
                rx_motion_alive = unpacked[27]
                rx_uptime = unpacked[28]
                last_rx_time = now_t

                for k in range(12):
                    if rx_gpios & (1 << k):
                        gpio_hold_time[k] = now_t
        except Exception:
            break

    is_online = (now_t - last_rx_time) < 0.7

    # 3. Render Dashboard UI
    W, H = screen.get_size()
    screen.fill((18, 22, 29))

    pygame.draw.rect(screen, (26, 32, 44), (0, 0, W, 64))
    screen.blit(font_h1.render("MECANUM ROBOT TELEMETRY & DIAGNOSTIC DASHBOARD", True, (88, 166, 255)), (24, 16))

    badge_y = 15
    conn_txt = f"WIFI: ONLINE ({rx_uptime // 1000}s)" if is_online else "WIFI: OFFLINE"
    draw_badge(screen, W - 620, badge_y, 210, 34, conn_txt, (39, 174, 96) if is_online else (192, 57, 43))

    uart_ok = is_online and (rx_motion_alive == 1)
    draw_badge(screen, W - 395, badge_y, 185, 34, "UART 2350: OK" if uart_ok else "UART 2350: WAIT", (41, 128, 185) if uart_ok else (211, 84, 0))

    mode_txt = "MODE: PID (RPM)" if pid_enable else "MODE: DIRECT PWM"
    draw_badge(screen, W - 195, badge_y, 175, 34, mode_txt, (142, 68, 173) if pid_enable else (44, 62, 80))

    left_x = 24
    left_w = int(W * 0.58) - 36
    right_x = left_x + left_w + 24
    right_w = W - right_x - 24

    # --- LEFT COLUMN: KINEMATICS & 4 WHEEL CARDS ---
    pygame.draw.rect(screen, (33, 39, 50), (left_x, 78, left_w, 108), border_radius=10)
    dir_label = get_direction_label(vx, vy, omega, test_wheel)
    screen.blit(font_h2.render(f"MOTION STATE: {dir_label}", True, (255, 215, 0)), (left_x + 16, 86))
    screen.blit(font_body.render(f"Gamepad -> Vx (Fwd): {vx:4d}  |  Vy (Strafe): {vy:4d}  |  Omega (Yaw): {omega:4d}", True, (220, 230, 242)), (left_x + 16, 118))
    screen.blit(font_small.render("D-Pad [+]: [UP]=M1(FL)  [RIGHT]=M2(FR)  [DOWN]=M3(RL)  [LEFT]=M4(RR)  |  [Triangle]: PID", True, (140, 195, 235)), (left_x + 16, 152))

    wheel_titles = [
        "M1 (FL - DPad UP)   ",
        "M2 (FR - DPad RIGHT)",
        "M3 (RL - DPad DOWN) ",
        "M4 (RR - DPad LEFT) "
    ]
    card_h = 98
    for i in range(4):
        cy = 198 + i * (card_h + 12)
        t_val = rx_target[i] if is_online else 0
        r_val = rx_rpm[i]    if is_online else 0
        p_val = rx_pwm[i]    if is_online else 0
        h_val = rx_enc_h[i]  if is_online else 0
        draw_dual_bar(screen, left_x, cy, left_w, card_h, wheel_titles[i], t_val, r_val, p_val, h_val, bool(pid_enable), test_wheel == (i + 1))

    # --- BOTTOM BAR: LIVE MOTION 2350 PRO GPIO PULSE SCANNER ---
    scan_y = 642
    pygame.draw.rect(screen, (33, 39, 50), (left_x, scan_y, W - 48, 114), border_radius=10)
    pygame.draw.rect(screen, (55, 65, 82), (left_x, scan_y, W - 48, 114), 2, border_radius=10)
    screen.blit(font_h2.render("LIVE MOTION 2350 PRO GPIO PULSE SCANNER (Spins wheel -> Lights up GREEN if pin receives signal)", True, (88, 166, 255)), (left_x + 16, scan_y + 12))

    box_w = (W - 80) // 12
    for k in range(12):
        bx = left_x + 16 + k * box_w
        by = scan_y + 50
        active = is_online and ((now_t - gpio_hold_time[k]) < 0.35)
        col = (39, 174, 96) if active else (45, 52, 66)
        draw_badge(screen, bx, by, box_w - 8, 46, SCAN_PIN_LABELS[k], col)

    # --- RIGHT COLUMN: ULTRASONIC RADAR & CONNECTION STATUS ---
    radar_h = scan_y - 78 - 14
    pygame.draw.rect(screen, (33, 39, 50), (right_x, 78, right_w, radar_h), border_radius=10)
    pygame.draw.rect(screen, (55, 65, 82), (right_x, 78, right_w, radar_h), 2, border_radius=10)
    screen.blit(font_h2.render("ULTRASONIC RADAR & SENSOR HEALTH (3x SR-04)", True, (88, 166, 255)), (right_x + 18, 92))

    # Sensor Connection Summary List
    us_names = ["SR-04 #1 (FRONT - Trig:4 Echo:5)", "SR-04 #2 (LEFT  - Trig:6 Echo:8)", "SR-04 #3 (RIGHT - Trig:10 Echo:11)"]
    for idx in range(3):
        conn_ok = is_online and bool(rx_us_ok & (1 << idx))
        scol, stxt = us_style(rx_us[idx], conn_ok)
        sy = 128 + idx * 32
        screen.blit(font_small.render(us_names[idx], True, (210, 220, 235)), (right_x + 20, sy + 5))
        draw_badge(screen, right_x + right_w - 190, sy, 170, 26, stxt, scol)

    cx = right_x + right_w // 2
    cy = 78 + radar_h // 2 + 65
    car_w, car_h = 110, 150

    # 1. Front Cone
    f_ok = is_online and bool(rx_us_ok & 1)
    f_col, f_txt = us_style(rx_us[0], f_ok)
    pygame.draw.polygon(screen, f_col, [(cx - 22, cy - car_h//2 - 6), (cx + 22, cy - car_h//2 - 6), (cx + 60, cy - car_h//2 - 75), (cx - 60, cy - car_h//2 - 75)], 3)
    draw_badge(screen, cx - 85, cy - car_h//2 - 115, 170, 32, f"FRONT: {f_txt}", f_col)

    # 2. Left Cone
    l_ok = is_online and bool(rx_us_ok & 2)
    l_col, l_txt = us_style(rx_us[1], l_ok)
    pygame.draw.polygon(screen, l_col, [(cx - car_w//2 - 6, cy - 22), (cx - car_w//2 - 6, cy + 22), (cx - car_w//2 - 70, cy + 55), (cx - car_w//2 - 70, cy - 55)], 3)
    draw_badge(screen, cx - car_w//2 - 195, cy - 16, 165, 32, f"L: {l_txt}", l_col)

    # 3. Right Cone
    r_ok = is_online and bool(rx_us_ok & 4)
    r_col, r_txt = us_style(rx_us[2], r_ok)
    pygame.draw.polygon(screen, r_col, [(cx + car_w//2 + 6, cy - 22), (cx + car_w//2 + 6, cy + 22), (cx + car_w//2 + 70, cy + 55), (cx + car_w//2 + 70, cy - 55)], 3)
    draw_badge(screen, cx + car_w//2 + 30, cy - 16, 165, 32, f"R: {r_txt}", r_col)

    # Chassis & Wheels
    pygame.draw.rect(screen, (60, 72, 92), (cx - car_w//2, cy - car_h//2, car_w, car_h), border_radius=12)
    pygame.draw.rect(screen, (140, 160, 190), (cx - car_w//2, cy - car_h//2, car_w, car_h), 3, border_radius=12)
    screen.blit(font_small.render("FRONT", True, (255, 215, 0)), (cx - 24, cy - car_h//2 + 10))

    wheel_coords = [
        (cx - car_w//2 - 20, cy - car_h//2 + 12),
        (cx + car_w//2 + 4,  cy - car_h//2 + 12),
        (cx - car_w//2 - 20, cy + car_h//2 - 52),
        (cx + car_w//2 + 4,  cy + car_h//2 - 52)
    ]
    for i, (wx, wy) in enumerate(wheel_coords):
        w_col = (100, 110, 125)
        if rx_pwm[i] > 10: w_col = (46, 204, 113)
        elif rx_pwm[i] < -10: w_col = (230, 126, 34)
        pygame.draw.rect(screen, w_col, (wx, wy, 16, 40), border_radius=4)

    pygame.display.flip()
    clock.tick(35)

pygame.quit()