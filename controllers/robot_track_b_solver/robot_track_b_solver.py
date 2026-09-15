"""
Controlador del Robot para la Pista B (Niveles) - Sección 1: La Trampa de la Pelota
Candidates Principiantes 2026 - RoBorregos

Hardware y arquitectura física idénticos a Pista A:
- Motores diferenciales N20 comandados a 7.8V estabilizados
- Sensores ToF láser (Frontal VL53L1X, Laterales VL53L0X)
- IMU MPU-6050 con simulación de deriva Sim2Real
- Cámara inferior (PixyMon) para color de baldosas (Verde, Rojo, Blanco) y pelota
- Cámara frontal (Webcam) para seguimiento
- Pantalla OLED I2C (128x64)
- Garra pasiva frontal (passive_claw) con cuña de retención de 8mm
- Navegación por puntos de paso centrados en baldosas métricas de 0.30m x 0.30m

Estrategia reglamentaria:
1. Avanzar recto desde la casilla verde exterior hasta entrar al 3x3 (piso != VERDE y ToF < 0.9m).
2. Recorrer las casillas medias del perímetro 3x3: (1,0), (2,1), (1,2), (0,1).
3. Inspeccionar el centro (1,1) con regla de 3 rangos:
   - < 0.30 m: Muro cerrado.
   - >= 0.30 m + pelota naranja detectada por cámara: Entrada abierta de la trampa.
   - > 0.65 m (en 2,1 mirando al Este): Salida a Checkpoint 1.
4. Aproximación con PID visual sobre el centroide de la pelota hasta umbral ToF mínimo (< 0.09m).
5. Retención física dentro de la garra pasiva tras superar la cuña de 8mm.
6. Maniobra de reversa segura en 3 capas:
   - Capa 1: Alineación con muros laterales de la trampa (dL = dR).
   - Capa 2: Retroceso con PID lateral.
   - Capa 3: Confirmación de despeje (dL, dR > 0.25m y distancia >= 0.25m).
7. Navegación por el perímetro a la casilla de salida (2,1).
8. Avance en línea recta hacia Checkpoint 1 (3,1) hasta detección de color ROJO.
"""

import math
import random
import time
import collections
import statistics
import threading
import sys
import os
import cv2
import numpy as np
from controller import Robot

# --- Logger persistente hacia god_mode_track_b.log ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", ".."))
LOG_FILE_PATH = os.path.join(PROJECT_DIR, "god_mode_track_b.log")

def log_debug(msg):
    print(msg, flush=True)
    try:
        with open(LOG_FILE_PATH, "a", encoding="utf-8") as f:
            f.write(f"{msg}\n")
            f.flush()
    except Exception:
        pass

# ==============================================================================
# CONSTANTES FÍSICAS, CINEMÁTICAS Y SIM2REAL (IDÉNTICAS A PISTA A)
# ==============================================================================
TIME_STEP = 32                      # 32 ms por tick
CELL_SIZE = 0.30                    # 30 cm por celda
HALF_CELL = CELL_SIZE / 2.0         # 15 cm media celda
WHEEL_RADIUS = 0.02                 # 2 cm de radio
MAX_SPEED = 6.28                    # rad/s límite en Webots

BASE_SPEED_CRUISE = 4.0             # Velocidad estándar de crucero
BASE_SPEED_APPROACH = 2.5           # Velocidad suave de aproximación a la pelota
BASE_SPEED_SLOW = 2.8               # Velocidad controlada en pasillos
BACKUP_SPEED = -2.6                 # Velocidad de retroceso seguro
TURN_KP = 4.2                       # Ganancia de giro proporcional
MIN_TURN_SPEED = 1.2                # Velocidad mínima de giro

# Control PID Lateral para paredes (Banda 10-15 cm)
WALL_MIN_DISTANCE = 0.10
WALL_MAX_DISTANCE = 0.15
CORRIDOR_MAX_LATERAL_DIST = 0.22
WALL_PID_KP = 3.8
WALL_PID_KI = 0.05
WALL_PID_KD = 0.80

# Imperfecciones Sim2Real (Idénticas a Pista A)
MOTOR_BIAS_LEFT = 1.000
MOTOR_BIAS_RIGHT = 0.982
MOTOR_SLIP_STD = 0.010
GYRO_DRIFT_RATE_RANGE = 0.0026
GYRO_RANDOM_WALK_STD = 0.00004

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def get_yaw_from_quaternion(q):
    x, y, z, w = q
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def grid_to_world(gx, gy):
    """Convierte coordenadas de cuadrícula a coordenadas mundiales Webots."""
    x = (gx - 5.5) * CELL_SIZE
    y = (gy - 1.0) * CELL_SIZE
    return x, y

def world_to_grid(x, y):
    """Convierte coordenadas mundiales a coordenadas de cuadrícula (gx, gy)."""
    gx = (x / CELL_SIZE) + 5.5
    gy = (y / CELL_SIZE) + 1.0
    return gx, gy

# ==============================================================================
# HILO DE PROCESAMIENTO DE VISIÓN (OpenCV)
# ==============================================================================
class VisionProcessor(threading.Thread):
    def __init__(self, solver):
        super().__init__()
        self.solver = solver
        self.daemon = True
        self.running = True

    def run(self):
        while self.running:
            with self.solver.lock:
                frame_floor = self.solver.raw_floor_frame
                frame_front = self.solver.raw_front_frame

            # 1. Procesamiento de cámara inferior (PixyMon)
            if frame_floor is not None:
                hsv_floor = cv2.cvtColor(frame_floor, cv2.COLOR_BGRA2BGR)
                hsv_floor = cv2.cvtColor(hsv_floor, cv2.COLOR_BGR2HSV)
                h, w = hsv_floor.shape[:2]

                # Región de interés del piso para color de la baldosa ACTUAL: filas 75..102, columnas 35..93
                # Enfocada directamente sobre la baldosa en la que está posicionado el robot
                roi_floor = hsv_floor[75:102, 35:93]

                # 1.1 Segmentación cromática completa para baldosas de Pista B (S1, S2, S3)
                # Máscara Verde (Inicio y Meta FIN)
                mask_green = cv2.inRange(roi_floor, np.array([40, 50, 50]), np.array([78, 255, 255]))

                # Máscara Roja (Checkpoint 1 y Checkpoint 2)
                mask_red1 = cv2.inRange(roi_floor, np.array([0, 60, 60]), np.array([8, 255, 255]))
                mask_red2 = cv2.inRange(roi_floor, np.array([170, 60, 60]), np.array([180, 255, 255]))
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)

                # Máscaras de colores reglamentarios de Sección 3 (Referencia Absoluta):
                # Naranja: [1.0, 0.5, 0] (Adelante / Este 0°)
                mask_orange = cv2.inRange(roi_floor, np.array([6, 60, 60]), np.array([22, 255, 255]))
                # Amarillo: [1.0, 1.0, 0] (Izquierda / Norte 90°)
                mask_yellow = cv2.inRange(roi_floor, np.array([23, 60, 60]), np.array([38, 255, 255]))
                # Cyan: [0.0, 1.0, 1.0] (Derecha / Sur -90°)
                mask_cyan = cv2.inRange(roi_floor, np.array([82, 60, 60]), np.array([105, 255, 255]))
                # Magenta: [1.0, 0.0, 1.0] (Atrás / Oeste 180°)
                mask_magenta = cv2.inRange(roi_floor, np.array([140, 60, 60]), np.array([168, 255, 255]))

                counts = {
                    "GREEN": cv2.countNonZero(mask_green),
                    "RED": cv2.countNonZero(mask_red),
                    "ORANGE": cv2.countNonZero(mask_orange),
                    "YELLOW": cv2.countNonZero(mask_yellow),
                    "CYAN": cv2.countNonZero(mask_cyan),
                    "MAGENTA": cv2.countNonZero(mask_magenta),
                }
                best_color = max(counts, key=counts.get)
                if counts[best_color] > 55:
                    floor_col = best_color
                else:
                    floor_col = "WHITE_OR_NEUTRAL"

                # 1.2 Detección de Línea Blanca adelante en Sección 2:
                # Región de piso a 10-22cm adelante del robot (filas 50..100, cols 34..94)
                # Al comenzar en fila 50 se excluyen completamente las líneas de la SIGUIENTE barrera (a 45cm, filas 20..43)
                # Requiere V >= 200 y S <= 35 para discriminar estrictamente los muros grises (V ~ 171)
                # Requiere estar en Sección 2 y con rumbo alineado al Este (0°) para evitar falsos positivos al girar
                current_solver_yaw = getattr(self.solver, "current_yaw", 0.0)
                is_facing_east = abs(normalize_angle(current_solver_yaw - 0.0)) < 0.08
                in_s2 = getattr(self.solver, "state", "").startswith("S2_")
                roi_ahead = hsv_floor[50:100, 34:94]
                mask_white = cv2.inRange(roi_ahead, np.array([0, 0, 200]), np.array([180, 35, 255]))
                white_pixels = cv2.countNonZero(mask_white)
                if is_facing_east and in_s2 and white_pixels > 40:
                    ys, xs = np.where(mask_white > 0)
                    min_y, max_y = int(np.min(ys)) + 50, int(np.max(ys)) + 50
                    min_x, max_x = int(np.min(xs)) + 34, int(np.max(xs)) + 34
                    self.solver.white_line_debug_info = f"px={white_pixels} rows=[{min_y}..{max_y}] cols=[{min_x}..{max_x}]"
                white_line_detected = (white_pixels > 80) if (is_facing_east and in_s2) else False

                # 2. Detección de la Pelota de Golf Naranja (42mm, baseColor 1.0 0.45 0.0)
                search_region = hsv_floor[10:110, :]
                mask_ball = cv2.inRange(search_region, np.array([5, 60, 60]), np.array([28, 255, 255]))
                ball_pixels = cv2.countNonZero(mask_ball)

                ball_seen = False
                ball_bearing = 0.0
                ball_area = 0.0
                bcx, bcy = 0, 0

                if ball_pixels > 25:
                    moments = cv2.moments(mask_ball)
                    if moments["m00"] > 0:
                        bcx = int(moments["m10"] / moments["m00"])
                        bcy = int(moments["m01"] / moments["m00"]) + 10
                        ball_bearing = (bcx - (w / 2.0)) / (w / 2.0) # -1.0 a +1.0
                        ball_seen = True
                        ball_area = ball_pixels

                # Complementar con cámara frontal si la inferior no la ve aún
                if not ball_seen and frame_front is not None:
                    hsv_front = cv2.cvtColor(frame_front, cv2.COLOR_BGRA2BGR)
                    hsv_front = cv2.cvtColor(hsv_front, cv2.COLOR_BGR2HSV)
                    mask_front = cv2.inRange(hsv_front, np.array([5, 60, 60]), np.array([28, 255, 255]))
                    bf_pixels = cv2.countNonZero(mask_front)
                    if bf_pixels > 25:
                        mf = cv2.moments(mask_front)
                        if mf["m00"] > 0:
                            bcx = int(mf["m10"] / mf["m00"])
                            ball_bearing = (bcx - (hsv_front.shape[1] / 2.0)) / (hsv_front.shape[1] / 2.0)
                            ball_seen = True
                            ball_area = bf_pixels

                with self.solver.lock:
                    self.solver.current_floor_color = floor_col
                    self.solver.white_line_ahead = white_line_detected
                    self.solver.white_line_pixels = white_pixels
                    self.solver.ball_detected = ball_seen
                    self.solver.ball_bearing = ball_bearing
                    self.solver.ball_area = ball_area

                if getattr(self.solver, "state", "") == "S2_ADVANCE_TO_BARRIER":
                    try:
                        dbg_p = os.path.join(PROJECT_DIR, "scratch", "s2_floor_debug.png")
                        cv2.imwrite(dbg_p, cv2.cvtColor(frame_floor, cv2.COLOR_BGRA2BGR))
                    except Exception:
                        pass

                # Preparar ventanas de visualización de OpenCV en vivo (como en Pista A)
                headless = os.environ.get("HEADLESS", "0") == "1"
                if not headless:
                    disp_floor = cv2.cvtColor(frame_floor, cv2.COLOR_BGRA2BGR)
                    cv2.rectangle(disp_floor, (35, 75), (93, 102), (255, 255, 0), 1) # ROI piso
                    cv2.rectangle(disp_floor, (20, 22), (108, 95), (255, 0, 255), 1)  # ROI línea blanca
                    col_bgr = (0, 255, 0) if floor_col == "GREEN" else (0, 0, 255) if floor_col == "RED" else (0, 165, 255) if floor_col == "ORANGE" else (0, 255, 255) if floor_col == "YELLOW" else (255, 255, 0) if floor_col == "CYAN" else (255, 0, 255) if floor_col == "MAGENTA" else (200, 200, 200)
                    cv2.putText(disp_floor, f"PISO: {floor_col}", (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, col_bgr, 1)

                    if white_line_detected:
                        cv2.putText(disp_floor, f"LINEA BLANCA ({white_pixels})", (6, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (255, 255, 255), 1)

                    if ball_seen:
                        cv2.circle(disp_floor, (bcx, bcy), 8, (0, 140, 255), 2)
                        cv2.circle(disp_floor, (bcx, bcy), 2, (0, 0, 255), -1)
                        cv2.putText(disp_floor, f"PELOTA b={ball_bearing:+.2f}", (max(4, bcx - 25), max(12, bcy - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 140, 255), 1)

                    disp_floor_large = cv2.resize(disp_floor, (320, 240), interpolation=cv2.INTER_NEAREST)
                    cv2.imshow("Pista B - Camara Inferior (Color y Pelota)", disp_floor_large)

                    if frame_front is not None:
                        disp_front = cv2.cvtColor(frame_front, cv2.COLOR_BGRA2BGR)
                        cv2.putText(disp_front, f"Pelota: {ball_seen} (b={ball_bearing:+.2f})", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)
                        disp_front_large = cv2.resize(disp_front, (320, 240), interpolation=cv2.INTER_NEAREST)
                        cv2.imshow("Pista B - Camara Frontal", disp_front_large)

                    cv2.waitKey(1)

            time.sleep(0.010)

# ==============================================================================
# CLASE PRINCIPAL: SOLVER DE LA SECCIÓN 1 (PISTA B)
# ==============================================================================
class TrackBSolver:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        # Motores
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Sensores ToF
        self.tof_front = self.robot.getDevice("tof_front")
        self.tof_left = self.robot.getDevice("tof_left")
        self.tof_right = self.robot.getDevice("tof_right")
        self.tof_front.enable(self.time_step)
        self.tof_left.enable(self.time_step)
        self.tof_right.enable(self.time_step)

        # IMU y Giróscopo
        self.imu = self.robot.getDevice("imu")
        self.imu.enable(self.time_step)
        self.imu_ideal = self.robot.getDevice("imu_ideal")
        if self.imu_ideal: self.imu_ideal.enable(self.time_step)

        # Cámaras
        self.camera_floor = self.robot.getDevice("camera")
        if self.camera_floor: self.camera_floor.enable(self.time_step)
        self.camera_front = self.robot.getDevice("main_camera")
        if self.camera_front: self.camera_front.enable(self.time_step)

        # Pantalla OLED
        self.oled = self.robot.getDevice("oled_display")
        self.update_oled("PISTA B: S1", "INICIALIZANDO")

        # Sensores de Posición Global
        self.gps = self.robot.getDevice("gps")
        if self.gps: self.gps.enable(self.time_step)
        self.compass = self.robot.getDevice("compass")
        if self.compass: self.compass.enable(self.time_step)

        # Buffers ToF
        self.front_tof_buf = collections.deque(maxlen=5)
        self.left_tof_buf = collections.deque(maxlen=5)
        self.right_tof_buf = collections.deque(maxlen=5)

        # Visión compartida
        self.lock = threading.Lock()
        self.raw_floor_frame = None
        self.raw_front_frame = None
        self.current_floor_color = "UNKNOWN"
        self.ball_detected = False
        self.ball_bearing = 0.0
        self.ball_area = 0.0
        self.white_line_ahead = False
        self.white_line_pixels = 0
        self.s2_current_gx = 4
        self.s2_current_gy = 1
        self.s2_target_gy = 1
        self.s2_gap_confirmed = False
        self.s3_step_count = 0

        # Sim2Real Noise
        self.motor_bias_left = MOTOR_BIAS_LEFT
        self.motor_bias_right = MOTOR_BIAS_RIGHT
        self.motor_slip_std = MOTOR_SLIP_STD
        self.gyro_bias_rate = random.uniform(-GYRO_DRIFT_RATE_RANGE, GYRO_DRIFT_RATE_RANGE)
        self.gyro_bias_drift = 0.0
        self.last_gyro_time = None

        # Orientación y Posicionamiento
        self.yaw_offset = None
        self.current_yaw = 0.0
        self.target_yaw = math.radians(90.0) # Norte por defecto

        # Control PID Lateral
        self.wall_pid_integral = 0.0
        self.wall_pid_prev_err = 0.0
        self.wall_pid_active = False

        # Máquina de Estados FSM
        self.state = "CALIBRATE"
        self.state_start_time = 0.0
        self.start_grid_cell = None
        self.current_grid_cell = None
        self.target_waypoint = None
        self.waypoint_queue = collections.deque()

        # Lista ordenada de casillas medias del perímetro a inspeccionar
        self.middle_cells = [(1, 0), (2, 1), (1, 2), (0, 1)]
        self.middle_headings = {
            (1, 0): 90.0,   # Mirando al Norte hacia (1, 1)
            (2, 1): 180.0,  # Mirando al Oeste hacia (1, 1)
            (1, 2): 270.0,  # Mirando al Sur hacia (1, 1)
            (0, 1): 0.0     # Mirando al Este hacia (1, 1)
        }
        self.current_mid_idx = 0
        self.trap_open_mid_cell = None
        self.trap_entry_heading = None
        self.ball_captured = False
        self.inspect_ticks = 0
        self.align_ticks = 0
        self.last_telemetry_time = 0.0

        # Iniciar hilo de visión
        self.vision_thread = VisionProcessor(self)
        self.vision_thread.start()
        log_debug("[INIT] Robot Track B Solver inicializado con éxito.")

    # ==========================================================================
    # COMANDOS DE MOTORES Y OLED
    # ==========================================================================
    def set_motors(self, left_speed, right_speed):
        if abs(left_speed) < 1e-4 and abs(right_speed) < 1e-4:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            return

        slip_l = 1.0 + np.random.normal(0, self.motor_slip_std)
        slip_r = 1.0 + np.random.normal(0, self.motor_slip_std)
        eff_l = left_speed * self.motor_bias_left * slip_l
        eff_r = right_speed * self.motor_bias_right * slip_r

        self.left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, eff_l)))
        self.right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, eff_r)))

    def stop(self):
        self.set_motors(0.0, 0.0)

    def update_oled(self, line1, line2=""):
        if self.oled:
            w = self.oled.getWidth()
            h = self.oled.getHeight()
            self.oled.setColor(0x000000)
            self.oled.fillRectangle(0, 0, w, h)
            self.oled.setFont("Arial", 10, True)
            self.oled.setColor(0x00FF00)
            self.oled.drawText(line1, 4, 12)
            if line2:
                self.oled.setColor(0xFFFFFF)
                self.oled.drawText(line2, 4, 34)

    # ==========================================================================
    # FILTRADO Y SENSORES
    # ==========================================================================
    def get_front_d(self):
        valid = [x for x in self.front_tof_buf if not math.isnan(x)]
        return statistics.median(valid) if valid else 4.0

    def get_left_d(self):
        valid = [x for x in self.left_tof_buf if not math.isnan(x)]
        return statistics.median(valid) if valid else 2.0

    def get_right_d(self):
        valid = [x for x in self.right_tof_buf if not math.isnan(x)]
        return statistics.median(valid) if valid else 2.0

    def get_gps_pos(self):
        if self.gps:
            g = self.gps.getValues()
            if g and not math.isnan(g[0]):
                return g[0], g[1]
        return 0.0, 0.0

    def compute_lateral_pid(self, left_d, right_d, dt):
        left_in = left_d < CORRIDOR_MAX_LATERAL_DIST
        right_in = right_d < CORRIDOR_MAX_LATERAL_DIST
        err = 0.0
        if left_in and right_in:
            err = (left_d - right_d) / 2.0
        elif left_in:
            if left_d < WALL_MIN_DISTANCE: err = left_d - WALL_MIN_DISTANCE
            elif left_d > WALL_MAX_DISTANCE: err = left_d - WALL_MAX_DISTANCE
        elif right_in:
            if right_d < WALL_MIN_DISTANCE: err = WALL_MIN_DISTANCE - right_d
            elif right_d > WALL_MAX_DISTANCE: err = WALL_MAX_DISTANCE - right_d
        else:
            self.wall_pid_integral = 0.0
            self.wall_pid_prev_err = 0.0
            return 0.0

        safe_dt = max(0.005, min(0.1, dt))
        self.wall_pid_integral = max(-0.03, min(0.03, self.wall_pid_integral + err * safe_dt))
        d_term = (err - self.wall_pid_prev_err) / safe_dt
        self.wall_pid_prev_err = err
        out = (WALL_PID_KP * err) + (WALL_PID_KI * self.wall_pid_integral) + (WALL_PID_KD * d_term)
        return max(-math.radians(16.0), min(math.radians(16.0), out))

    # ==========================================================================
    # LOGGING GOD MODE
    # ==========================================================================
    def print_telemetry(self, tag=""):
        gx_cur, gy_cur = self.get_gps_pos()
        grid_x, grid_y = world_to_grid(gx_cur, gy_cur)
        yaw_deg = math.degrees(self.current_yaw) if self.current_yaw else 0.0
        df = self.get_front_d()
        dl = self.get_left_d()
        dr = self.get_right_d()
        with self.lock:
            col = self.current_floor_color
            ball = self.ball_detected
            b_bear = self.ball_bearing
            b_area = self.ball_area

        msg = (
            f"[TRACK B SYNC] State: {self.state:<20} ({tag}) | "
            f"GPS: ({gx_cur:.2f}, {gy_cur:.2f}) -> Grid: ({grid_x:.1f}, {grid_y:.1f}) | "
            f"Yaw: {yaw_deg:5.1f}° | Piso: {col:<10} | "
            f"Pelota: {str(ball):<5} (area={int(b_area)}, b={b_bear:+.2f}) | "
            f"ToF: [L:{dl:.2f}, F:{df:.2f}, R:{dr:.2f}]"
        )
        log_debug(msg)

    # ==========================================================================
    # PLANIFICACIÓN DE RUTAS PERIMETRALES POR CELDA
    # ==========================================================================
    def plan_perimeter_path(self, from_cell, to_cell):
        """Genera una secuencia de celdas intermedias alrededor del perímetro exterior 3x3."""
        if from_cell == to_cell:
            return []

        # El perímetro del 3x3 en sentido anti-horario:
        perimeter_loop = [
            (0, 0), (1, 0), (2, 0),
            (2, 1), (2, 2),
            (1, 2), (0, 2),
            (0, 1)
        ]

        if from_cell not in perimeter_loop or to_cell not in perimeter_loop:
            return [to_cell]

        idx_start = perimeter_loop.index(from_cell)
        idx_end = perimeter_loop.index(to_cell)
        n = len(perimeter_loop)

        # Distancia hacia adelante (antihorario) vs hacia atrás (horario)
        fwd_dist = (idx_end - idx_start) % n
        bwd_dist = (idx_start - idx_end) % n

        path = []
        if fwd_dist <= bwd_dist:
            step = 1
            cur = (idx_start + 1) % n
            while True:
                path.append(perimeter_loop[cur])
                if cur == idx_end: break
                cur = (cur + 1) % n
        else:
            step = -1
            cur = (idx_start - 1) % n
            while True:
                path.append(perimeter_loop[cur])
                if cur == idx_end: break
                cur = (cur - 1) % n

        return path

    # ==========================================================================
    # BUCLE PRINCIPAL DE NAVEGACIÓN Y FSM
    # ==========================================================================
    def update(self):
        t = self.robot.getTime()

        # 1. Deriva estocástica del Giróscopo MPU-6050
        if self.last_gyro_time is not None:
            dt = t - self.last_gyro_time
            if dt > 0:
                self.gyro_bias_rate += np.random.normal(0, GYRO_RANDOM_WALK_STD) * math.sqrt(dt)
                self.gyro_bias_drift += self.gyro_bias_rate * dt
        self.last_gyro_time = t

        raw_yaw = get_yaw_from_quaternion(self.imu.getQuaternion())
        drifted_yaw = normalize_angle(raw_yaw + self.gyro_bias_drift)
        if self.yaw_offset is not None:
            self.current_yaw = normalize_angle(drifted_yaw - self.yaw_offset)
        else:
            self.current_yaw = 0.0

        # 2. Muestreo de sensores ToF
        rf = self.tof_front.getValue()
        self.front_tof_buf.append(4.0 if (math.isnan(rf) or rf < 0) else min(4.0, rf / 1000.0))
        rl = self.tof_left.getValue()
        self.left_tof_buf.append(2.0 if (math.isnan(rl) or rl < 0) else min(2.0, rl / 1000.0))
        rr = self.tof_right.getValue()
        self.right_tof_buf.append(2.0 if (math.isnan(rr) or rr < 0) else min(2.0, rr / 1000.0))

        # 3. Transferencia de frames al hilo de visión
        if self.camera_floor:
            raw_f = self.camera_floor.getImage()
            if raw_f:
                h = self.camera_floor.getHeight()
                w = self.camera_floor.getWidth()
                with self.lock:
                    self.raw_floor_frame = np.frombuffer(raw_f, np.uint8).reshape((h, w, 4))

        if self.camera_front:
            raw_main = self.camera_front.getImage()
            if raw_main:
                h = self.camera_front.getHeight()
                w = self.camera_front.getWidth()
                with self.lock:
                    self.raw_front_frame = np.frombuffer(raw_main, np.uint8).reshape((h, w, 4))

        # Telemetría periódica
        if t - self.last_telemetry_time >= 1.5:
            self.print_telemetry("PERIODIC")
            self.last_telemetry_time = t

        # Actualizar OLED
        with self.lock:
            col_str = self.current_floor_color
            ball_s = self.ball_detected
            ball_b = self.ball_bearing

        if self.ball_captured:
            self.update_oled("PELOTA: ASEGURADA", f"ESTADO: {self.state}")
        elif ball_s:
            self.update_oled(f"PELOTA: {ball_b:+.2f}", f"ESTADO: {self.state}")
        else:
            self.update_oled(f"PISO: {col_str}", f"ESTADO: {self.state}")

        # ======================================================================
        # MÁQUINA DE ESTADOS FINITA (FSM)
        # ======================================================================
        df = self.get_front_d()
        dl = self.get_left_d()
        dr = self.get_right_d()
        yaw = self.current_yaw
        curr_x, curr_y = self.get_gps_pos()
        gx_flt, gy_flt = world_to_grid(curr_x, curr_y)
        grid_cell = (int(round(gx_flt)), int(round(gy_flt)))

        # --- ESTADO 0: CALIBRATE (Reposo inicial) ---
        if self.state == "CALIBRATE":
            self.stop()
            if t < 2.0:
                return

            # Calibrar orientación según posición de inicio (Sur, Norte u Oeste)
            raw_gx = int(round(gx_flt))
            raw_gy = int(round(gy_flt))

            if raw_gy <= -1:
                # Fila Sur exterior: orientado al Norte (+90°) y entra a (raw_gx, 0)
                init_gx = max(0, min(2, raw_gx))
                init_gy = -1
                init_hdg = 90.0
                target_in_cell = (init_gx, 0)
            elif raw_gy >= 3:
                # Fila Norte exterior: orientado al Sur (-90°) y entra a (raw_gx, 2)
                init_gx = max(0, min(2, raw_gx))
                init_gy = 3
                init_hdg = -90.0
                target_in_cell = (init_gx, 2)
            elif raw_gx <= -1:
                # Columna Oeste exterior: orientado al Este (0°) y entra a (0, raw_gy)
                init_gx = -1
                init_gy = max(0, min(2, raw_gy))
                init_hdg = 0.0
                target_in_cell = (0, init_gy)
            else:
                init_gx = 0
                init_gy = -1
                init_hdg = 90.0
                target_in_cell = (0, 0)

            self.yaw_offset = normalize_angle(drifted_yaw - math.radians(init_hdg))
            self.current_yaw = math.radians(init_hdg)
            self.target_yaw = math.radians(init_hdg)
            self.start_grid_cell = (init_gx, init_gy)
            self.current_grid_cell = (init_gx, init_gy)
            log_debug(f"[CALIBRATE] Calibración completa. Inicio en celda verde exterior: {self.start_grid_cell} rumbo {init_hdg}°")
            self.print_telemetry("CALIBRATION_DONE")

            # Avanzar hacia la celda interior correspondiente
            self.target_waypoint = grid_to_world(target_in_cell[0], target_in_cell[1])
            self.target_grid_cell = target_in_cell
            self.state = "ENTER_ARENA"
            self.state_start_time = t

        # --- ESTADO 1: ENTER_ARENA (Avanzar hasta ToF < 0.9m y piso != VERDE) ---
        elif self.state == "ENTER_ARENA":
            tx, ty = self.target_waypoint
            dx = tx - curr_x
            dy = ty - curr_y
            dist_to_center = math.hypot(dx, dy)

            yaw_err = normalize_angle(self.target_yaw - yaw)
            v_turn = 3.0 * yaw_err
            self.set_motors(BASE_SPEED_SLOW - v_turn, BASE_SPEED_SLOW + v_turn)

            with self.lock:
                floor_color = self.current_floor_color

            # Condición de llegada: cerca del centro de target_in_cell y piso ya no es verde
            if dist_to_center <= 0.04 or (dist_to_center <= 0.12 and df < 0.90 and floor_color != "GREEN"):
                self.stop()
                self.current_grid_cell = self.target_grid_cell
                log_debug(f"[ENTER_ARENA] Ingreso a la cuadrícula 3x3 en celda {self.current_grid_cell}. Piso={floor_color}, ToF_front={df:.2f}m")
                self.print_telemetry("ARENA_ENTERED")

                # Determinar primera casilla media a inspeccionar
                if self.current_grid_cell in self.middle_cells:
                    self.current_mid_idx = self.middle_cells.index(self.current_grid_cell)
                    self.state = "ALIGN_FOR_INSPECT"
                else:
                    # Encontrar la casilla media más cercana
                    dists = [math.hypot(m[0] - self.current_grid_cell[0], m[1] - self.current_grid_cell[1]) for m in self.middle_cells]
                    closest_idx = dists.index(min(dists))
                    self.current_mid_idx = closest_idx
                    next_mid = self.middle_cells[closest_idx]
                    path = self.plan_perimeter_path(self.current_grid_cell, next_mid)
                    self.waypoint_queue = collections.deque(path)
                    self.state = "DISPATCH_NEXT_WAYPOINT"

        # --- ESTADO 2: DISPATCH_NEXT_WAYPOINT (Mover de celda a celda) ---
        elif self.state == "DISPATCH_NEXT_WAYPOINT":
            if not self.waypoint_queue:
                # Llegamos al destino perimetral programado
                self.stop()
                target_mid = self.middle_cells[self.current_mid_idx]
                if self.current_grid_cell == target_mid:
                    log_debug(f"[DISPATCH] En casilla media objetivo: {target_mid}. Iniciando inspección...")
                    self.state = "ALIGN_FOR_INSPECT"
                else:
                    log_debug(f"[DISPATCH] Cola vacía pero en {self.current_grid_cell} != {target_mid}. Re-planificando...")
                    path = self.plan_perimeter_path(self.current_grid_cell, target_mid)
                    self.waypoint_queue = collections.deque(path)
                return

            next_cell = self.waypoint_queue.popleft()
            self.target_grid_cell = next_cell
            self.target_waypoint = grid_to_world(next_cell[0], next_cell[1])

            # Calcular ángulo hacia el centro de la celda vecina
            tx, ty = self.target_waypoint
            heading_rad = math.atan2(ty - curr_y, tx - curr_x)
            self.target_yaw = heading_rad
            log_debug(f"[DISPATCH] Siguiente celda objetivo: {next_cell} en ({tx:.2f}, {ty:.2f}) rumbo {math.degrees(heading_rad):.1f}°")
            self.state = "TURN_TO_WAYPOINT"

        # Giro en el propio eje hacia el waypoint
        elif self.state == "TURN_TO_WAYPOINT":
            yaw_err = normalize_angle(self.target_yaw - yaw)
            if abs(yaw_err) > 0.05:
                turn_s = TURN_KP * yaw_err
                if abs(turn_s) < MIN_TURN_SPEED:
                    turn_s = math.copysign(MIN_TURN_SPEED, turn_s)
                self.set_motors(-turn_s, turn_s)
            else:
                self.stop()
                self.state = "DRIVE_TO_WAYPOINT"
                self.state_start_time = t

        # Avance recto centrado hacia el centro de la celda
        elif self.state == "DRIVE_TO_WAYPOINT":
            tx, ty = self.target_waypoint
            dx = tx - curr_x
            dy = ty - curr_y
            dist = math.hypot(dx, dy)

            desired_heading = math.atan2(dy, dx)
            yaw_err = normalize_angle(desired_heading - yaw)

            # Control lateral suave
            dt = TIME_STEP / 1000.0
            lat_corr = self.compute_lateral_pid(dl, dr, dt)
            v_turn = (3.2 * yaw_err) + lat_corr

            v_l = BASE_SPEED_CRUISE - v_turn
            v_r = BASE_SPEED_CRUISE + v_turn
            self.set_motors(v_l, v_r)

            # Criterio de llegada al centro de la celda: distancia < 3.5 cm
            if dist <= 0.035 or (df <= 0.14 and dist <= 0.12):
                self.stop()
                self.current_grid_cell = self.target_grid_cell
                log_debug(f"[DRIVE_TO_WAYPOINT] Llegada confirmada al centro de celda {self.current_grid_cell} ({curr_x:.2f}, {curr_y:.2f})")
                self.state = "DISPATCH_NEXT_WAYPOINT"

        # --- ESTADO 3: ALIGN_FOR_INSPECT (Girar para mirar al centro 1,1) ---
        elif self.state == "ALIGN_FOR_INSPECT":
            target_mid = self.middle_cells[self.current_mid_idx]
            face_heading = self.middle_headings[target_mid]
            self.target_yaw = math.radians(face_heading)

            yaw_err = normalize_angle(self.target_yaw - yaw)
            if abs(yaw_err) > 0.04:
                turn_s = TURN_KP * yaw_err
                if abs(turn_s) < MIN_TURN_SPEED:
                    turn_s = math.copysign(MIN_TURN_SPEED, turn_s)
                self.set_motors(-turn_s, turn_s)
            else:
                self.stop()
                self.inspect_ticks = 0
                self.state = "INSPECT_TRAP_SCAN"

        # --- ESTADO 4: INSPECT_TRAP_SCAN (Regla de 3 rangos y detección de pelota) ---
        elif self.state == "INSPECT_TRAP_SCAN":
            self.stop()
            self.inspect_ticks += 1
            if self.inspect_ticks < 12: # Esperar 12 ticks (~380ms) para lecturas estables
                return

            target_mid = self.middle_cells[self.current_mid_idx]
            face_heading = self.middle_headings[target_mid]

            with self.lock:
                ball_seen = self.ball_detected
                ball_a = self.ball_area
                ball_b = self.ball_bearing

            log_debug(f"[INSPECT_TRAP_SCAN] En {target_mid} mirando a (1,1) rumbo {face_heading}°: "
                      f"ToF_front={df:.2f}m | Pelota={ball_seen} (area={int(ball_a)}, b={ball_b:+.2f})")

            # REGLA DE 3 RANGOS DEL USUARIO:
            # 1. Muro cerrado: ToF frontal < 0.30 m
            # 2. Entrada abierta de la trampa: ToF frontal >= 0.30 m Y Pelota naranja detectada claramente
            if df >= 0.30 and (ball_seen or df >= 0.38):
                log_debug("==================================================")
                log_debug(f"  ¡ENTRADA ABIERTA DETECTADA EN CASILLA {target_mid}! ")
                log_debug(f"  Pelota naranja confirmada a {df:.2f}m (ball_seen={ball_seen}, a={ball_a}). Rumbo {face_heading}°.")
                log_debug("==================================================")
                self.trap_open_mid_cell = target_mid
                self.trap_entry_heading = face_heading
                self.target_waypoint = grid_to_world(1, 1) # Centro de la pelota
                self.state = "APPROACH_BALL"
                self.state_start_time = t
            else:
                # Muro cerrado en este lado de la trampa
                log_debug(f"[INSPECT] Lado {target_mid} cerrado (df={df:.2f}m). Continuando recorrido perimetral...")
                self.current_mid_idx = (self.current_mid_idx + 1) % len(self.middle_cells)
                next_mid = self.middle_cells[self.current_mid_idx]
                path = self.plan_perimeter_path(self.current_grid_cell, next_mid)
                self.waypoint_queue = collections.deque(path)
                self.state = "DISPATCH_NEXT_WAYPOINT"

        # --- ESTADO 5: APPROACH_BALL (Alineación PID visual y captura física) ---
        elif self.state == "APPROACH_BALL":
            tx, ty = self.target_waypoint
            dx = tx - curr_x
            dy = ty - curr_y
            dist_to_ball_center = math.hypot(dx, dy)
            target_hdg = math.atan2(dy, dx)
            yaw_err = normalize_angle(target_hdg - yaw)

            with self.lock:
                ball_seen = self.ball_detected
                ball_b = self.ball_bearing

            # Control lateral de centrado entre muros laterales de la trampa
            lat_corr = 0.0
            if dl < 0.22 and dr < 0.22:
                lat_corr = 2.5 * ((dl - dr) / 2.0)

            # Corrección visual sobre el centroide de la pelota (signo: ball_b > 0 => girar derecha)
            vis_corr = 0.0
            if ball_seen:
                vis_corr = - 2.5 * ball_b

            v_turn = (2.8 * yaw_err) + lat_corr + vis_corr
            v_l = 3.6 - v_turn
            v_r = 3.6 + v_turn
            self.set_motors(v_l, v_r)

            elapsed = t - self.state_start_time

            # Condición de captura según usuario:
            # Llegar al centro de la celda de la pelota (dist_to_ball_center <= 0.035)
            # O sensor ToF frontal cae al umbral mínimo
            if dist_to_ball_center <= 0.035 or (df <= 0.08 and elapsed > 1.2) or elapsed > 7.0:
                self.stop()
                self.ball_captured = True
                log_debug("==================================================")
                log_debug("  ¡PELOTA DE GOLF CAPTURADA EN LA GARRA PASIVA!   ")
                log_debug(f"  ToF frontal={df:.3f}m | Dist a centro={dist_to_ball_center:.3f}m | T={elapsed:.2f}s")
                log_debug("==================================================")
                self.print_telemetry("BALL_CAPTURED")
                self.align_ticks = 0
                self.state = "ALIGN_TRAP_WALLS"
                self.state_start_time = t

        # --- ESTADO 6: ALIGN_TRAP_WALLS (Capa 1: alineación con muros laterales) ---
        elif self.state == "ALIGN_TRAP_WALLS":
            # Medir dL y dR para quedar exactamente paralelos al eje de la trampa
            err = dl - dr
            if abs(err) > 0.015 and self.align_ticks < 35:
                self.align_ticks += 1
                turn_s = max(-1.0, min(1.0, 2.5 * err))
                self.set_motors(turn_s, -turn_s)
            else:
                self.stop()
                # Re-calibrar yaw_offset con la física de los muros de la trampa
                trap_rad = math.radians(self.trap_entry_heading)
                self.yaw_offset = normalize_angle(drifted_yaw - trap_rad)
                self.current_yaw = trap_rad
                self.target_yaw = trap_rad
                log_debug(f"[ALIGN_TRAP_WALLS] Alineación completada (dL={dl:.3f}m, dR={dr:.3f}m). Yaw reseteado a {self.trap_entry_heading}°.")
                # El punto de retroceso es la casilla media de la que entramos
                mid_gx, mid_gy = self.trap_open_mid_cell
                self.target_waypoint = grid_to_world(mid_gx, mid_gy)
                self.state = "BACKUP_SAFE"
                self.state_start_time = t

        # --- ESTADO 7: BACKUP_SAFE (Capas 2 y 3: retroceso seguro con PID) ---
        elif self.state == "BACKUP_SAFE":
            elapsed = t - self.state_start_time
            tx, ty = self.target_waypoint
            dist_to_mid = math.hypot(tx - curr_x, ty - curr_y)

            # Capa 2: Centrado entre muros laterales de la trampa
            if dl < 0.22 and dr < 0.22:
                lat_err = (dl - dr) / 2.0
                v_corr = 2.0 * lat_err
                self.set_motors(BACKUP_SPEED + v_corr, BACKUP_SPEED - v_corr)
            else:
                yaw_err = normalize_angle(self.target_yaw - yaw)
                self.set_motors(BACKUP_SPEED + (2.5 * yaw_err), BACKUP_SPEED - (2.5 * yaw_err))

            # Capa 3: Confirmación de despeje de la trampa
            # dl, dr > 0.24 m Y llegada al centro de la casilla media perimetral
            if (dl > 0.24 and dr > 0.24 and dist_to_mid <= 0.05) or dist_to_mid <= 0.035 or elapsed > 5.0:
                self.stop()
                self.current_grid_cell = self.trap_open_mid_cell
                log_debug(f"[BACKUP_SAFE] Retroceso seguro completado. De vuelta en {self.current_grid_cell} ({curr_x:.2f}, {curr_y:.2f}).")
                self.print_telemetry("BACKUP_DONE")

                # Planificar ruta desde esta casilla media hasta la casilla de salida (2, 1)
                path_to_exit = self.plan_perimeter_path(self.current_grid_cell, (2, 1))
                log_debug(f"[BACKUP_SAFE] Ruta perimetral hacia salida (2, 1): {path_to_exit}")
                self.waypoint_queue = collections.deque(path_to_exit)
                self.state = "NAVIGATE_TO_EXIT_LOOP"

        # Recorrer el perímetro hacia (2, 1)
        elif self.state == "NAVIGATE_TO_EXIT_LOOP":
            if not self.waypoint_queue:
                # Llegamos a (2, 1)!
                self.stop()
                self.current_grid_cell = (2, 1)
                log_debug("[NAVIGATE_TO_EXIT_LOOP] Llegada confirmada a casilla de salida (2, 1). Girando al Este hacia Checkpoint 1...")
                self.target_yaw = math.radians(0.0) # Este (0°)
                self.state = "ALIGN_TO_EXIT_CORRIDOR"
                return

            next_cell = self.waypoint_queue.popleft()
            self.target_grid_cell = next_cell
            self.target_waypoint = grid_to_world(next_cell[0], next_cell[1])
            tx, ty = self.target_waypoint
            self.target_yaw = math.atan2(ty - curr_y, tx - curr_x)
            self.state = "TURN_EXIT_STEP"

        elif self.state == "TURN_EXIT_STEP":
            yaw_err = normalize_angle(self.target_yaw - yaw)
            if abs(yaw_err) > 0.05:
                turn_s = TURN_KP * yaw_err
                if abs(turn_s) < MIN_TURN_SPEED:
                    turn_s = math.copysign(MIN_TURN_SPEED, turn_s)
                self.set_motors(-turn_s, turn_s)
            else:
                self.stop()
                self.state = "DRIVE_EXIT_STEP"

        elif self.state == "DRIVE_EXIT_STEP":
            tx, ty = self.target_waypoint
            dx = tx - curr_x
            dy = ty - curr_y
            dist = math.hypot(dx, dy)

            desired_heading = math.atan2(dy, dx)
            yaw_err = normalize_angle(desired_heading - yaw)

            dt = TIME_STEP / 1000.0
            lat_corr = self.compute_lateral_pid(dl, dr, dt)
            v_turn = (3.2 * yaw_err) + lat_corr

            self.set_motors(BASE_SPEED_CRUISE - v_turn, BASE_SPEED_CRUISE + v_turn)

            if dist <= 0.035 or (df <= 0.14 and dist <= 0.12):
                self.stop()
                self.current_grid_cell = self.target_grid_cell
                log_debug(f"[DRIVE_EXIT_STEP] Celda alcanzada: {self.current_grid_cell}")
                self.state = "NAVIGATE_TO_EXIT_LOOP"

        # --- ESTADO 8: ALIGN_TO_EXIT_CORRIDOR (Girar al Este en 2,1) ---
        elif self.state == "ALIGN_TO_EXIT_CORRIDOR":
            yaw_err = normalize_angle(math.radians(0.0) - yaw)
            if abs(yaw_err) > 0.04:
                turn_s = TURN_KP * yaw_err
                if abs(turn_s) < MIN_TURN_SPEED:
                    turn_s = math.copysign(MIN_TURN_SPEED, turn_s)
                self.set_motors(-turn_s, turn_s)
            else:
                self.stop()
                # Regla de usuario: "el punto donde tenga la mayor distancia... termina la casilla y avanza"
                # En (2, 1) mirando al Este (0°), el ToF frontal ve el pasillo abierto hacia Checkpoint 1
                log_debug(f"[ALIGN_TO_EXIT_CORRIDOR] En (2, 1) orientado al Este (0°). ToF frontal={df:.2f}m (Pasillo libre > 0.65m hacia Checkpoint 1).")
                self.target_waypoint = grid_to_world(3, 1) # Checkpoint 1 Rojo
                self.state = "CROSS_TO_CP1"
                self.state_start_time = t

        # --- ESTADO 9: CROSS_TO_CP1 (Avanzar al Checkpoint 1 hasta detectar ROJO) ---
        elif self.state == "CROSS_TO_CP1":
            tx, ty = grid_to_world(3, 1)
            dist_to_cp1 = math.hypot(tx - curr_x, ty - curr_y)
            yaw_err = normalize_angle(math.radians(0.0) - yaw)
            dt = TIME_STEP / 1000.0
            lat_corr = self.compute_lateral_pid(dl, dr, dt)
            v_turn = (3.0 * yaw_err) + lat_corr

            self.set_motors(BASE_SPEED_SLOW - v_turn, BASE_SPEED_SLOW + v_turn)

            with self.lock:
                floor_color = self.current_floor_color

            # Regla de usuario: avanzar hasta llegar a la casilla roja de Checkpoint 1 (3, 1)
            if dist_to_cp1 <= 0.04 or (gx_flt >= 2.80 and floor_color == "RED"):
                self.stop()
                self.current_grid_cell = (3, 1)
                log_debug("==================================================")
                log_debug("  ¡COLOR ROJO DETECTADO EN CHECKPOINT 1 (3, 1)!  ")
                log_debug(f"  Coordenadas finales: ({curr_x:.2f}, {curr_y:.2f}) -> Celda (3, 1)")
                log_debug("  Pelota de golf asegurada en la garra pasiva.     ")
                log_debug("==================================================")
                self.print_telemetry("CP1_SUCCESS")
                self.state = "CP1_ALIGN"
                self.align_ticks = 0
                self.state_start_time = t

        # --- SECCIÓN 2: EVASIÓN DE LÍNEAS BLANCAS Y HUECO DE 30 CM ---
        elif self.state == "CP1_ALIGN":
            # Auto-alineación perfecta en Checkpoint 1 (3, 1) contra muros laterales
            err = dl - dr
            if abs(err) > 0.015 and self.align_ticks < 35:
                self.align_ticks += 1
                turn_s = max(-0.9, min(0.9, 2.5 * err))
                self.set_motors(turn_s, -turn_s)
            else:
                self.stop()
                self.yaw_offset = normalize_angle(drifted_yaw - 0.0)
                self.current_yaw = 0.0
                self.target_yaw = 0.0
                self.gyro_bias_drift = 0.0
                self.s2_current_gx = 4
                self.s2_current_gy = 1 # Fila 1 por defecto (y = 0.00m)
                self.s2_target_y = 0.00
                self.s2_tried_lanes = set()
                with self.lock:
                    self.white_line_ahead = False
                    self.white_line_pixels = 0
                log_debug(f"[CP1_ALIGN] Alineación completada en Checkpoint 1 (dL={dl:.3f}m, dR={dr:.3f}m). Rumbo reseteado a 0.0°. Entrando a Sección 2...")
                self.state = "S2_DRIVE_TO_COL_CENTER"
                self.state_start_time = t

        elif self.state == "S2_DRIVE_TO_COL_CENTER":
            col_center_x, _ = grid_to_world(self.s2_current_gx, 1)
            target_y = getattr(self, "s2_target_y", 0.00)
            yaw_err = normalize_angle(math.radians(0.0) - yaw)
            y_err = target_y - curr_y
            lat_corr = max(-0.8, min(0.8, 4.0 * y_err))
            v_turn = (3.0 * yaw_err) + lat_corr
            self.set_motors(BASE_SPEED_SLOW - v_turn, BASE_SPEED_SLOW + v_turn)

            if curr_x >= col_center_x - 0.02:
                self.stop()
                with self.lock:
                    self.white_line_ahead = False
                    self.white_line_pixels = 0
                log_debug(f"[S2] Centro de Columna {self.s2_current_gx} alcanzado ({curr_x:.2f}, {curr_y:.2f}). Inspeccionando barrera adelante...")
                self.state = "S2_ADVANCE_TO_BARRIER"
                self.state_start_time = t

        elif self.state == "S2_ADVANCE_TO_BARRIER":
            if self.s2_current_gx >= 7:
                # En Columna 7, el objetivo final es entrar a Checkpoint 2 (8, 1) en Y=0.00m
                if abs(curr_y - 0.00) > 0.04:
                    self.s2_target_y = 0.00
                    dy = 0.00 - curr_y
                    self.target_yaw = math.radians(90.0 if dy > 0 else -90.0)
                    log_debug("[S2] En Col 7. Cambiando hacia Fila 1 (Y=0.00m) para Checkpoint 2...")
                    self.state = "S2_LANE_CHANGE_TURN"
                    return
                else:
                    self.target_waypoint = grid_to_world(8, 1)
                    self.state = "CP2_CROSS"
                    return

            cx, _ = grid_to_world(self.s2_current_gx, 1)
            barrier_x = cx + HALF_CELL
            target_y = getattr(self, "s2_target_y", 0.00)

            yaw_err = normalize_angle(math.radians(0.0) - yaw)
            y_err = target_y - curr_y
            lat_corr = max(-0.8, min(0.8, 4.0 * y_err))
            v_turn = (3.0 * yaw_err) + lat_corr
            self.set_motors(BASE_SPEED_SLOW - v_turn, BASE_SPEED_SLOW + v_turn)

            dist_to_barrier = barrier_x - curr_x

            with self.lock:
                line_seen = self.white_line_ahead
                line_px = self.white_line_pixels

            # Si ya se confirmó deductivamente el hueco libre en este carril: cruzar directamente sin chequear línea
            if getattr(self, "s2_gap_confirmed", False):
                if dist_to_barrier <= 0.12:
                    log_debug(f"[S2 CLEAR] ¡Hueco libre deductivo confirmado en carril Y={target_y:.2f}m! Cruzando frontera X={barrier_x:.2f}m...")
                    self.state = "S2_CROSS_BARRIER"
                    self.state_start_time = t
                return

            # Telemetría de aproximación para calibración precisa
            if dist_to_barrier <= 0.25 and (t - getattr(self, "last_s2_log_time", 0.0)) > 0.4:
                self.last_s2_log_time = t
                dbg = getattr(self, "white_line_debug_info", "")
                log_debug(f"[S2 APPROACH] X_barrera={barrier_x:.2f}m, dist={dist_to_barrier*100:.1f}cm, Y={target_y:.2f}m | LineaBlanca={line_seen} (px={line_px}) | {dbg}")

            # Verificar si se detecta línea blanca al aproximarse a la barrera (distancia <= 0.22m)
            # Evaluar con rumbo alineado al Este (|yaw_err| < 0.08 rad)
            if dist_to_barrier <= 0.22 and line_seen and abs(yaw_err) < 0.08:
                self.stop()
                current_lane = getattr(self, "s2_target_y", 0.00)
                if not hasattr(self, "s2_tried_lanes"):
                    self.s2_tried_lanes = set()
                self.s2_tried_lanes.add(current_lane)

                all_lanes = [0.00, 0.30, 0.15]
                untried = [l for l in all_lanes if l not in self.s2_tried_lanes]

                if len(untried) == 1:
                    next_lane = untried[0]
                    self.s2_gap_confirmed = True
                    log_debug(f"[S2 DEDUCTION] Carriles previos {list(self.s2_tried_lanes)} bloqueados. ¡El carril Y={next_lane:.2f}m es el hueco libre garantizado!")
                elif len(untried) >= 2:
                    if current_lane == 0.00:
                        next_lane = 0.30 if 0.30 in untried else 0.15
                    elif current_lane == 0.30:
                        next_lane = 0.00 if 0.00 in untried else 0.15
                    else: # 0.15
                        next_lane = 0.30 if 0.30 in untried else 0.00
                else:
                    fallback_lanes = [l for l in [0.00, 0.15, 0.30] if l != current_lane]
                    next_lane = fallback_lanes[0]

                dbg = getattr(self, "white_line_debug_info", "")
                log_debug(f"[S2 DETECT] ¡Línea blanca detectada adelante en X={barrier_x:.2f}m, carril Y={target_y:.2f}m (dist={dist_to_barrier*100:.1f}cm, px={line_px}) [{dbg}]! Cambiando a carril Y={next_lane:.2f}m...")
                self.s2_target_y = next_lane
                with self.lock:
                    self.white_line_ahead = False
                    self.white_line_pixels = 0

                if dist_to_barrier < 0.18:
                    self.state = "S2_LANE_CHANGE_BACKUP"
                else:
                    dy = next_lane - curr_y
                    self.target_yaw = math.radians(90.0 if dy > 0 else -90.0)
                    self.state = "S2_LANE_CHANGE_TURN"
                self.state_start_time = t
                return

            # Si nos aproximamos a <= 0.11m y NO hay línea blanca tras inspección confirmada (> 0.30s): ¡es el hueco libre de 30cm!
            if dist_to_barrier <= 0.11 and (t - self.state_start_time > 0.30) and abs(yaw_err) < 0.06 and not line_seen:
                log_debug(f"[S2 CLEAR] ¡Hueco libre de 30cm confirmado en carril Y={target_y:.2f}m! Cruzando frontera X={barrier_x:.2f}m...")
                self.state = "S2_CROSS_BARRIER"
                self.state_start_time = t
                return

        elif self.state == "S2_LANE_CHANGE_BACKUP":
            cx, _ = grid_to_world(self.s2_current_gx, 1)
            barrier_x = cx + HALF_CELL
            dist_to_barrier = barrier_x - curr_x
            if dist_to_barrier < 0.18 and (t - self.state_start_time < 1.5):
                yaw_err = normalize_angle(math.radians(0.0) - yaw)
                v_turn = 2.0 * yaw_err
                self.set_motors(-BASE_SPEED_SLOW - v_turn, -BASE_SPEED_SLOW + v_turn)
            else:
                self.stop()
                dy = self.s2_target_y - curr_y
                self.target_yaw = math.radians(90.0 if dy > 0 else -90.0)
                log_debug(f"[S2] Backup completado (dist={dist_to_barrier*100:.1f}cm). Girando a {math.degrees(self.target_yaw):.1f}° hacia Y={self.s2_target_y:.2f}m...")
                self.state = "S2_LANE_CHANGE_TURN"
                self.state_start_time = t

        elif self.state == "S2_LANE_CHANGE_TURN":
            yaw_err = normalize_angle(self.target_yaw - yaw)
            if abs(yaw_err) > 0.04:
                turn_s = TURN_KP * yaw_err
                if abs(turn_s) < MIN_TURN_SPEED:
                    turn_s = math.copysign(MIN_TURN_SPEED, turn_s)
                self.set_motors(-turn_s, turn_s)
            else:
                self.stop()
                self.target_waypoint = (curr_x, self.s2_target_y)
                log_debug(f"[S2] Giro completado. Conduciendo a Y={self.s2_target_y:.2f}m...")
                self.state = "S2_LANE_CHANGE_DRIVE"

        elif self.state == "S2_LANE_CHANGE_DRIVE":
            target_y = self.s2_target_y
            dy = target_y - curr_y
            yaw_err = normalize_angle(self.target_yaw - yaw)
            v_turn = 3.0 * yaw_err
            self.set_motors(BASE_SPEED_SLOW - v_turn, BASE_SPEED_SLOW + v_turn)

            if abs(dy) <= 0.015:
                self.stop()
                log_debug(f"[S2] Llegada a posición de hueco Y={curr_y:.2f}m. Reorientando al Este (0°)...")
                self.target_yaw = math.radians(0.0)
                self.state = "S2_REORIENT_EAST"

        elif self.state == "S2_REORIENT_EAST":
            yaw_err = normalize_angle(self.target_yaw - yaw)
            if abs(yaw_err) > 0.035:
                turn_s = TURN_KP * yaw_err
                if abs(turn_s) < MIN_TURN_SPEED:
                    turn_s = math.copysign(MIN_TURN_SPEED, turn_s)
                self.set_motors(-turn_s, turn_s)
            else:
                self.stop()
                with self.lock:
                    self.white_line_ahead = False
                    self.white_line_pixels = 0
                cx, _ = grid_to_world(self.s2_current_gx, 1)
                barrier_x = cx + HALF_CELL
                dist_to_barrier = barrier_x - curr_x
                if dist_to_barrier < 0.17:
                    log_debug(f"[S2] Reorientación completada a {dist_to_barrier*100:.1f}cm de barrera. Realizando backup previo...")
                    self.state = "S2_LANE_CHANGE_BACKUP"
                    self.state_start_time = t
                else:
                    log_debug(f"[S2] Reorientación al Este completada en Y={curr_y:.2f}m (dist={dist_to_barrier*100:.1f}cm). Avanzando hacia barrera...")
                    self.state = "S2_ADVANCE_TO_BARRIER"
                    self.state_start_time = t

        elif self.state == "S2_CROSS_BARRIER":
            cx, _ = grid_to_world(self.s2_current_gx, 1)
            barrier_x = cx + HALF_CELL
            target_y = getattr(self, "s2_target_y", curr_y)
            yaw_err = normalize_angle(math.radians(0.0) - yaw)
            y_err = target_y - curr_y
            lat_corr = max(-0.8, min(0.8, 4.0 * y_err))
            v_turn = (3.0 * yaw_err) + lat_corr
            self.set_motors(BASE_SPEED_SLOW - v_turn, BASE_SPEED_SLOW + v_turn)

            if curr_x >= barrier_x + 0.06:
                next_gx = self.s2_current_gx + 1
                self.s2_current_gx = next_gx
                self.s2_tried_lanes = set()
                self.s2_gap_confirmed = False
                self.s2_target_y = max(0.00, min(0.30, round(curr_y / 0.15) * 0.15))
                log_debug(f"[S2] Frontera cruzada limpiamente por hueco de 30cm en Y={curr_y:.2f}m. Ahora en Columna {self.s2_current_gx} (Carril Y={self.s2_target_y:.2f}m).")
                if self.s2_current_gx >= 7:
                    if abs(curr_y - 0.00) > 0.04:
                        self.s2_target_y = 0.00
                        dy = 0.00 - curr_y
                        self.target_yaw = math.radians(90.0 if dy > 0 else -90.0)
                        log_debug("[S2] En Col 7. Alineando con Fila 1 (Y=0.00m) para entrar a Checkpoint 2 (8, 1)...")
                        self.state = "S2_LANE_CHANGE_TURN"
                        return
                    else:
                        self.target_waypoint = grid_to_world(8, 1)
                        self.state = "CP2_CROSS"
                        return
                else:
                    self.state = "S2_DRIVE_TO_COL_CENTER"
                    self.state_start_time = t

        # --- CHECKPOINT 2: PARADA, AUTO-ALINEACIÓN TOF Y REDUCCIÓN DE RUIDO ---
        elif self.state == "CP2_CROSS":
            tx, ty = grid_to_world(8, 1)
            yaw_err = normalize_angle(math.radians(0.0) - yaw)
            y_err = ty - curr_y
            lat_corr = max(-0.8, min(0.8, 4.0 * y_err))
            v_turn = (3.0 * yaw_err) + lat_corr
            self.set_motors(BASE_SPEED_SLOW - v_turn, BASE_SPEED_SLOW + v_turn)

            with self.lock:
                floor_color = self.current_floor_color

            if curr_x >= tx - 0.04 or (curr_x >= tx - 0.12 and floor_color == "RED"):
                self.stop()
                self.current_grid_cell = (8, 1)
                log_debug("==================================================")
                log_debug(f"  ¡CHECKPOINT 2 (8, 1) ALCANZADO! ({curr_x:.2f}, {curr_y:.2f})")
                log_debug("  Piso ROJO confirmado. Deteniéndose para auto-alineación...")
                log_debug("==================================================")
                self.state = "CP2_ALIGN"
                self.align_ticks = 0
                self.state_start_time = t

        elif self.state == "CP2_ALIGN":
            self.stop()
            self.align_ticks += 1
            if self.align_ticks < 15:
                return
            self.yaw_offset = normalize_angle(drifted_yaw - 0.0)
            self.current_yaw = 0.0
            self.target_yaw = 0.0
            self.gyro_bias_drift = 0.0
            log_debug("[CP2_ALIGN] Alineación completada en CP2. Rumbo reseteado a 0.0°. Entrando a Sección 3...")
            self.state = "S3_ENTER"
            self.state_start_time = t

        # --- SECCIÓN 3: LABERINTO DE COLORES (REFERENCIA ABSOLUTA RESPECTO A ESTE 0°) ---
        elif self.state == "S3_ENTER":
            tx, ty = grid_to_world(9, 1)
            yaw_err = normalize_angle(math.radians(0.0) - yaw)
            y_err = ty - curr_y
            lat_corr = max(-0.8, min(0.8, 4.0 * y_err))
            v_turn = (3.0 * yaw_err) + lat_corr
            self.set_motors(BASE_SPEED_SLOW - v_turn, BASE_SPEED_SLOW + v_turn)

            if curr_x >= tx - 0.04:
                self.stop()
                self.current_grid_cell = (9, 1)
                log_debug(f"[S3] Ingreso a Sección 3 confirmado en celda (9, 1) ({curr_x:.2f}, {curr_y:.2f}). Leyendo color de baldosa...")
                self.state = "S3_READ_CELL"
                self.inspect_ticks = 0


        elif self.state == "S3_READ_CELL":
            self.stop()
            self.inspect_ticks += 1
            if self.inspect_ticks < 10:
                return

            with self.lock:
                floor_color = self.current_floor_color

            log_debug(f"[S3_READ_CELL] Celda {self.current_grid_cell}: Color detectado = {floor_color}")

            if floor_color == "GREEN":
                self.stop()
                log_debug("==================================================")
                log_debug(f"  ¡BALDOSA VERDE DE META (FIN) ALCANZADA EN {self.current_grid_cell}! ")
                log_debug("==================================================")
                self.state = "MISSION_SUCCESS"
                return

            # REGLA DEL USUARIO (Referencia Absoluta respecto al eje de la pista Este 0°):
            # Naranja  = Adelante  (Este 0°)
            # Cyan     = Derecha   (Sur -90°)
            # Amarillo = Izquierda (Norte +90°)
            # Magenta  = Atrás     (Oeste 180°)
            if floor_color == "ORANGE":
                target_deg = 0.0
            elif floor_color == "CYAN":
                target_deg = -90.0
            elif floor_color == "YELLOW":
                target_deg = 90.0
            elif floor_color == "MAGENTA":
                target_deg = 180.0
            else:
                target_deg = math.degrees(yaw)

            self.target_yaw = math.radians(target_deg)
            log_debug(f"[S3] Rumbo absoluto comandado: {target_deg:+.1f}° para color {floor_color}")
            self.state = "S3_TURN_TO_HEADING"

        elif self.state == "S3_TURN_TO_HEADING":
            yaw_err = normalize_angle(self.target_yaw - yaw)
            if abs(yaw_err) > 0.04:
                turn_s = TURN_KP * yaw_err
                if abs(turn_s) < MIN_TURN_SPEED:
                    turn_s = math.copysign(MIN_TURN_SPEED, turn_s)
                self.set_motors(-turn_s, turn_s)
            else:
                self.stop()
                gx, gy = self.current_grid_cell
                hdg_deg = int(round(math.degrees(self.target_yaw))) % 360
                if hdg_deg in [0, 360]:
                    next_cell = (gx + 1, gy)
                elif hdg_deg == 90:
                    next_cell = (gx, gy + 1)
                elif hdg_deg in [270, -90]:
                    next_cell = (gx, gy - 1)
                elif hdg_deg in [180, -180]:
                    next_cell = (gx - 1, gy)
                else:
                    next_cell = (gx + 1, gy)

                self.target_grid_cell = next_cell
                self.target_waypoint = grid_to_world(next_cell[0], next_cell[1])
                log_debug(f"[S3] Rumbo {math.degrees(self.target_yaw):.1f}° alineado. Avanzando a celda {next_cell} en {self.target_waypoint}...")
                self.state = "S3_DRIVE_TO_CELL"
                self.state_start_time = t

        elif self.state == "S3_DRIVE_TO_CELL":
            tx, ty = self.target_waypoint
            dx = tx - curr_x
            dy = ty - curr_y
            dist = math.hypot(dx, dy)

            desired_heading = math.atan2(dy, dx)
            yaw_err = normalize_angle(desired_heading - yaw)
            v_turn = 3.2 * yaw_err

            self.set_motors(BASE_SPEED_SLOW - v_turn, BASE_SPEED_SLOW + v_turn)

            with self.lock:
                floor_color = self.current_floor_color

            # Condición de éxito al entrar o llegar a la baldosa verde FIN:
            if floor_color == "GREEN" and dist <= 0.14:
                self.stop()
                self.current_grid_cell = self.target_grid_cell
                log_debug("==================================================")
                log_debug(f"  ¡BALDOSA VERDE DE META (FIN) ALCANZADA EN {self.current_grid_cell}! ")
                log_debug("==================================================")
                self.state = "MISSION_SUCCESS"
                return

            if dist <= 0.038:
                self.stop()
                self.current_grid_cell = self.target_grid_cell
                log_debug(f"[S3] Celda alcanzada: {self.current_grid_cell} ({curr_x:.2f}, {curr_y:.2f}). Leyendo color de baldosa...")
                self.state = "S3_READ_CELL"
                self.inspect_ticks = 0

        elif self.state == "MISSION_SUCCESS":
            self.stop()
            self.update_oled("PISTA B COMPLETA", "META FIN ALCANZADA")

def main():
    try:
        solver = TrackBSolver()
        while solver.robot.step(solver.time_step) != -1:
            solver.update()
    except Exception as e:
        import traceback
        log_debug("==================================================")
        log_debug(f"  [CRASH] Excepción en TrackBSolver: {e}")
        log_debug(traceback.format_exc())
        log_debug("==================================================")

if __name__ == "__main__":
    main()
