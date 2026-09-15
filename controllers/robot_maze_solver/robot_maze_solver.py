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

# Ruta al archivo de log para telemetría en vivo (God Mode)
LOG_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "god_mode_live.log")

class LoggerTee(object):
    """
    Duplicador de salida: todo lo que imprimimos sale en la consola de Webots
    y además se guarda en vivo en god_mode_live.log para no perdernos nada.
    """
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.filename = filename
        
    def write(self, message):
        self.terminal.write(message)
        self.terminal.flush()
        try:
            with open(self.filename, "a", encoding="utf-8") as f:
                f.write(message)
                f.flush()
        except Exception:
            pass
            
    def flush(self):
        self.terminal.flush()

sys.stdout = LoggerTee(LOG_FILE_PATH)
sys.stderr = sys.stdout

def log_debug(msg):
    print(msg, flush=True)

# ==============================================================================
# VARIABLES AJUSTABLES Y PARÁMETROS DE CONFIGURACIÓN (SIM2REAL & PID)
# ==============================================================================
# ¡Aquí mero tienes todas las perillas para afinar el comportamiento del carrito!

# 1. Parámetros Físicos y Geometría del Robot
TIME_STEP = 32                      # Paso de simulación de Webots en milisegundos (32 ms por tick)
CELL_SIZE = 0.30                    # Tamaño de cada celda del laberinto (30 cm)
WHEEL_RADIUS = 0.02                 # Radio de las ruedas (2 cm)
MAX_SPEED = 6.28                    # Velocidad máxima permitida por los motores en Webots (rad/s)

# 2. Modelo Eléctrico de Motores DC y Dead-Reckoning (Alimentación Estabilizada a 7.8V con Step-Down)
# Datos de laboratorio del motor físico real:
# - Tensión de prueba nominal: 6.0 V -> Velocidad sin carga medida: 297.0 RPM
# - Tensión estabilizada con regulador Buck (Step-Down): 7.8 V (constante durante toda la carrera)
MOTOR_RATED_VOLTAGE = 6.0           # Tensión de referencia nominal (6.0V)
MOTOR_RATED_RPM = 297.0             # 297 RPM a 6.0V
MOTOR_STABILIZED_VOLTAGE = 7.8      # Tensión fija de salida del convertidor Step-Down (7.8V)

# Conversión física lineal de RPM según la tensión de alimentación:
# RPM(7.8V) = 297.0 * (7.8V / 6.0V) = 297.0 * 1.30 = 386.1 RPM
VOLTAGE_RATIO = MOTOR_STABILIZED_VOLTAGE / MOTOR_RATED_VOLTAGE      # Factor de escala por voltaje: 1.30
MOTOR_MAX_RPM_7_8V = MOTOR_RATED_RPM * VOLTAGE_RATIO               # 386.1 RPM a 7.8V
MOTOR_MAX_OMEGA_7_8V = MOTOR_MAX_RPM_7_8V * (2.0 * math.pi / 60.0) # ~40.4323 rad/s
MOTOR_MAX_LINEAR_SPEED = MOTOR_MAX_OMEGA_7_8V * WHEEL_RADIUS        # ~0.8086 m/s (80.86 cm/s)

# Modulación de Crucero para navegación estable en casillas de 30 cm:
# (En el mundo real se aplica modulación PWM; en simulación se comanda velocidad angular equivalente)
BASE_SPEED_FLAT = 4.2               # Velocidad de crucero en suelo plano (rad/s)
BASE_SPEED_RAMP = 4.5               # Empuje extra para vencer la fricción en rampas (rad/s)
PWM_DUTY_CYCLE_FLAT = BASE_SPEED_FLAT / MOTOR_MAX_OMEGA_7_8V        # ~10.39% PWM equivalente a 7.8V

# Odometría por Dead-Reckoning temporal a velocidad estabilizada:
DEAD_RECKONING_LINEAR_SPEED = BASE_SPEED_FLAT * WHEEL_RADIUS        # 0.084 m/s (8.4 cm/s)
DEAD_RECKONING_CELL_TIME = CELL_SIZE / DEAD_RECKONING_LINEAR_SPEED  # ~3.571 segundos por celda (30 cm)

TURN_KP = 4.0                       # Ganancia del giro sobre el propio eje (para apuntar rápido al rumbo)
MIN_TURN_SPEED = 1.0                # Velocidad mínima al girar para vencer la fricción estática del piso
BACKUP_SPEED = -2.8                 # Velocidad al echarse para atrás cuando nos pegamos a un muro frontal

# 3. Repulsión de Emergencia de Muro Frontal (Exclusiva para la pared de enfrente)
# Ojo: si el sensor frontal ve una pared a menos de 10 cm, clava los frenos de una y recula
FRONT_WALL_STOP_THRESHOLD = 0.10    # 10 cm: frenada en seco si la pared frontal se nos viene encima
FRONT_WALL_BACKUP_THRESHOLD = 0.10  # 10 cm: umbral para activar la maniobra de retroceso (BACKUP)
FRONT_WALL_CLEAR_THRESHOLD = 0.15   # 15 cm: retrocede hasta aquí (nos deja exactamente en el centro de la celda)
NEIGHBOR_WALL_THRESHOLD = 0.28      # 28 cm: distancia ToF para saber si hay pasaje libre o muro en el DFS

# 4. Controlador PID para Paredes Laterales (Banda 10-15 cm y Centrado Equidistante)
# Mantiene al robot en el carril central (entre 10 y 15 cm de la pared más cercana)
# y centra suavemente con dos paredes, re-alineando el rumbo para eliminar la deriva del giróscopo.
WALL_MIN_DISTANCE = 0.10            # 10 cm: límite inferior (empujar para alejarse si estamos muy pegados)
WALL_MAX_DISTANCE = 0.15            # 15 cm: límite superior (atraer suavemente hacia el carril si nos abrimos)
CORRIDOR_MAX_LATERAL_DIST = 0.22    # 22 cm: alcance máximo para considerar que hay pared lateral
WALL_PID_KP = 3.8                   # Ganancia Proporcional: reacción suave y firme
WALL_PID_KI = 0.05                  # Ganancia Integral: elimina la asimetría de motores sin sobreoscilar
WALL_PID_KD = 0.80                  # Ganancia Derivativa: amortigua para avance recto
WALL_PID_MAX_CORRECTION_DEG = 16.0  # Ángulo máximo de autoridad de timoneo (16 grados)

# 5. Modelado de Imperfecciones Físicas Sim2Real (GY-521 & Motores)
# Esto es para que la simulación se sienta como el mundo real y no como un videojuego perfecto
MOTOR_BIAS_LEFT = 1.000             # Motor izquierdo al 100% de potencia
MOTOR_BIAS_RIGHT = 0.982            # Motor derecho 1.8% más flojo (típica asimetría de fabricación de motores DC)
MOTOR_SLIP_STD = 0.010              # Micro-patinaje aleatorio de las ruedas (1% de ruido gaussiano en tracción)
GYRO_DRIFT_RATE_RANGE = 0.0026      # Sesgo inicial del giróscopo MPU-6050 (+-0.15°/s de deriva constante al encender)
GYRO_RANDOM_WALK_STD = 0.00004      # Paseo aleatorio térmico (el sesgo va cambiando poco a poco con el tiempo)

# ==============================================================================
# FUNCIONES MATEMÁTICAS AUXILIARES
# ==============================================================================

def normalize_angle(angle):
    """Mantiene cualquier ángulo en radianes dentro del rango bonito [-pi, +pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

def get_yaw_from_quaternion(q):
    """Convierte el cuaternión que nos da Webots [x, y, z, w] al ángulo de Yaw (giro en Z)."""
    x, y, z, w = q
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def get_compass_heading(yaw_rad):
    """Redondea el ángulo al rumbo cardinal más cercano (0°, 90°, 180° o 270°)."""
    deg = math.degrees(yaw_rad)
    return int(round(deg / 90.0) * 90) % 360

# ==============================================================================
# HILO DE PROCESAMIENTO DE VISIÓN (OpenCV)
# ==============================================================================

class VisionProcessor(threading.Thread):
    """
    Corre la visión artificial en un hilo separado para que el procesamiento
    de imágenes no le robe ciclos ni freezee el control de los motores en el hilo principal.
    """
    def __init__(self, solver):
        super().__init__()
        self.solver = solver
        self.daemon = True
        self.running = True
        self.last_color_printed = None
        self.last_aruco_printed = None
        
    def run(self):
        headless = os.environ.get("HEADLESS", "0") == "1"
        while self.running:
            # 1. Agarramos los frames más recientes de ambas cámaras
            with self.solver.lock:
                upper_frame = self.solver.upper_frame_data
                lower_frame = self.solver.lower_frame_data
                
            # --- CÁMARA SUPERIOR: Detección de marcadores ArUco en las paredes ---
            if upper_frame is not None:
                gray = cv2.cvtColor(upper_frame, cv2.COLOR_BGRA2GRAY)
                aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                parameters = cv2.aruco.DetectorParameters()
                detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
                corners, ids, _ = detector.detectMarkers(gray)
                
                disp_upper = cv2.cvtColor(upper_frame, cv2.COLOR_BGRA2BGR)
                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(disp_upper, corners, ids)
                    marker_id = int(np.ravel(ids)[0])
                    if self.last_aruco_printed != marker_id:
                        print(f"[VISION] ¡ARUCO DETECTADO! ID: {marker_id}")
                        self.last_aruco_printed = marker_id
                        
                    # Filtramos detecciones esporádicas exigiendo que aparezca al menos 3 veces
                    if marker_id == self.solver.last_aruco_id:
                        self.solver.aruco_consecutive += 1
                    else:
                        self.solver.last_aruco_id = marker_id
                        self.solver.aruco_consecutive = 1
                        
                    if self.solver.aruco_consecutive >= 3:
                        with self.solver.lock:
                            self.solver.shared_aruco_text = f"ARUCO ID: {marker_id}"
                            self.solver.aruco_clear_time = time.time() + 3.0
                else:
                    self.solver.aruco_consecutive = 0
                    self.last_aruco_printed = None
                    
                if not headless:
                    cv2.imshow("Upper Camera (ArUco)", disp_upper)
                
            # --- CÁMARA INFERIOR: Identificación del color del piso en HSV ---
            if lower_frame is not None:
                hsv = cv2.cvtColor(lower_frame, cv2.COLOR_BGRA2BGR)
                hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)
                
                # Región de interés (ROI) en el centro de la imagen
                h, w = hsv.shape[:2]
                cx, cy = w // 2, h // 2
                roi_size = 40
                roi = hsv[cy-roi_size:cy+roi_size, cx-roi_size:cx+roi_size]
                
                # Máscaras de color para el laberinto
                mask_red1 = cv2.inRange(roi, np.array([0, 120, 120]), np.array([10, 255, 255]))
                mask_red2 = cv2.inRange(roi, np.array([160, 120, 120]), np.array([180, 255, 255]))
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)
                
                mask_yellow = cv2.inRange(roi, np.array([20, 120, 120]), np.array([40, 255, 255]))
                mask_orange = cv2.inRange(roi, np.array([11, 120, 120]), np.array([19, 255, 255]))
                mask_cyan = cv2.inRange(roi, np.array([85, 120, 120]), np.array([105, 255, 255]))
                mask_magenta = cv2.inRange(roi, np.array([130, 120, 120]), np.array([160, 255, 255]))
                
                color_name = None
                is_red = False
                threshold = 2000
                
                # Checamos si alguno de los colores supera el umbral de píxeles
                if cv2.countNonZero(mask_red) > threshold:
                    color_name = "RED"
                    is_red = True
                elif cv2.countNonZero(mask_yellow) > threshold:
                    color_name = "YELLOW"
                elif cv2.countNonZero(mask_orange) > threshold:
                    color_name = "ORANGE"
                elif cv2.countNonZero(mask_cyan) > threshold:
                    color_name = "CYAN"
                elif cv2.countNonZero(mask_magenta) > threshold:
                    color_name = "MAGENTA"
                        
                if color_name:
                    if self.last_color_printed != color_name:
                        print(f"[VISION] Color de piso detectado: {color_name}")
                        self.last_color_printed = color_name
                        
                    if color_name != "RED":
                        with self.solver.lock:
                            self.solver.shared_color_text = f"COLOR: {color_name}"
                            
                with self.solver.lock:
                    # ¡Si pisamos ROJO alcanzamos la meta del laberinto!
                    if is_red and self.solver.state in ["SCAN", "DECIDE", "MOVE"]:
                        self.solver.found_red_goal = True
                        
                if not headless:
                    cv2.imshow("Lower Camera (Floor)", lower_frame)
                
            if not headless:
                cv2.waitKey(1)
            time.sleep(0.005 if headless else 0.03)

# ==============================================================================
# CLASE PRINCIPAL: CONTROLADOR Y MÁQUINA DE ESTADOS DEL ROBOT
# ==============================================================================

class MazeSolver:
    def __init__(self):
        self.robot = Robot()
        
        # 1. Configuración de actuadores (Motores en modo velocidad infinita)
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # 2. Encoders de posición en cada rueda
        self.left_encoder = self.robot.getDevice('left wheel sensor')
        self.right_encoder = self.robot.getDevice('right wheel sensor')
        self.left_encoder.enable(TIME_STEP)
        self.right_encoder.enable(TIME_STEP)
        
        # 3. Sensores ToF de distancia (VL53L1X frontal, VL53L0X laterales)
        self.tof_front = self.robot.getDevice('tof_front')
        self.tof_left = self.robot.getDevice('tof_left')
        self.tof_right = self.robot.getDevice('tof_right')
        self.tof_front.enable(TIME_STEP)
        self.tof_left.enable(TIME_STEP)
        self.tof_right.enable(TIME_STEP)
        
        # 4. IMU ruidosa (MPU-6050) vs IMU ideal (Ground-Truth para comparar)
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(TIME_STEP)
        self.imu_ideal = self.robot.getDevice('imu_ideal')
        self.imu_ideal.enable(TIME_STEP)
        
        # 5. Cámaras y pantalla OLED
        self.upper_camera = self.robot.getDevice('main_camera')
        self.upper_camera.enable(TIME_STEP)
        self.lower_camera = self.robot.getDevice('camera')
        self.lower_camera.enable(TIME_STEP)
        self.oled = self.robot.getDevice('oled_display')
        self.robot.batterySensorEnable(TIME_STEP)
        
        # 6. Sensores de Verificación Global (God-Mode Ground Truth)
        self.gps = self.robot.getDevice('gps')
        if self.gps: self.gps.enable(TIME_STEP)
        self.compass = self.robot.getDevice('compass')
        if self.compass: self.compass.enable(TIME_STEP)
        
        self.start_gps_x = None
        self.start_gps_y = None
        self.current_yaw = 0.0
        self.last_telemetry_time = 0.0
        
        # 7. Memoria Topológica del Algoritmo DFS (El hilo de oro)
        self.state = "CALIBRATE"
        self.grid_x = 0                     # Coordenada X relativa de la celda actual
        self.grid_y = 0                     # Coordenada Y relativa de la celda actual
        self.target_cell = (0, 0)
        self.visited = {(0, 0)}             # Conjunto matemático de celdas visitadas
        self.route_stack = [(0, 0)]         # Pila LIFO con la ruta activa desde el inicio
        self.start_l = 0.0
        self.start_r = 0.0
        self.move_start_time = 0.0          # Marca de tiempo en que inicia el avance de la celda
        self.start_front_d = 0.0            # Lectura ToF frontal al inicio del avance de la celda
        
        self.yaw_offset = None
        self.yaw_offset_ideal = None
        self.pitch_offset = 0.0
        self.target_yaw = 0.0
        
        # 8. Estado del Terreno (Plano, Rampa Arriba, Rampa Abajo)
        self.terrain_phase = "FLAT"
        self.filtered_pitch = 0.0
        self.flat_ticks = 0
        self.settle_time = 0.0
        self.backup_start_time = 0.0
        
        # 9. Buffers de mediana móvil para limpiar picos de ruido ToF
        self.front_tof_buffer = collections.deque(maxlen=5)
        self.left_tof_buffer = collections.deque(maxlen=5)
        self.right_tof_buffer = collections.deque(maxlen=5)
        self.lateral_history = collections.deque(maxlen=8) # Historial para estimar ángulo físico con paredes
        
        # 10. Variables compartidas con el hilo de visión
        self.lock = threading.Lock()
        self.upper_frame_data = None
        self.lower_frame_data = None
        self.shared_aruco_text = ""
        self.shared_color_text = ""
        self.found_red_goal = False
        self.last_aruco_id = -1
        self.aruco_consecutive = 0
        self.aruco_clear_time = 0.0
        
        # 11. Modelo estocástico de deriva temporal del MPU-6050
        self.gyro_bias_rate = random.uniform(-GYRO_DRIFT_RATE_RANGE, GYRO_DRIFT_RATE_RANGE)
        self.gyro_bias_drift = 0.0
        self.last_gyro_time = None
        
        # 12. Asimetría física y deslizamiento de motores
        self.motor_bias_left = MOTOR_BIAS_LEFT
        self.motor_bias_right = MOTOR_BIAS_RIGHT
        self.motor_slip_std = MOTOR_SLIP_STD
        
        # 13. Controlador PID Lateral para muros
        self.wall_pid_kp = WALL_PID_KP
        self.wall_pid_ki = WALL_PID_KI
        self.wall_pid_kd = WALL_PID_KD
        self.wall_pid_integral = 0.0
        self.wall_pid_prev_error = 0.0
        self.wall_pid_active = False
        self.last_pid_correction = 0.0
        
        # Lanzamos el procesador de visión en paralelo
        self.vision_thread = VisionProcessor(self)
        self.vision_thread.start()

    def set_motors(self, left_speed, right_speed):
        """
        Aplica velocidad a los motores inyectando asimetría de fabricación
        y micro-patinaje estocástico de las llantas.
        """
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

    def reset_wall_pid(self):
        """Limpia la memoria del PID lateral para que no acumule error viejo."""
        self.wall_pid_integral = 0.0
        self.wall_pid_prev_error = 0.0
        self.wall_pid_active = False
        self.last_pid_correction = 0.0
        self.lateral_history.clear()

    def compute_lateral_pid(self, left_d, right_d, dt):
        """
        Controlador PID lateral de 3 zonas y centrado:
        - 2 paredes (< 22 cm): centrado equidistante perfecto en medio del pasillo.
        - 1 pared: mantiene al robot en la banda ideal de 10-15 cm de la pared más cercana:
          * Si d < 10 cm: empuja hacia afuera para evitar rozar.
          * Si 10 cm <= d <= 15 cm: zona muerta ideal, avanza recto y suave sin oscilar.
          * Si 15 cm < d < 22 cm: atrae suavemente hacia el carril para entrar alineado a cruces.
        - Sin paredes (>= 22 cm): se desactiva limpiamente sin meter ruido.
        """
        left_in_range = left_d < CORRIDOR_MAX_LATERAL_DIST
        right_in_range = right_d < CORRIDOR_MAX_LATERAL_DIST
        
        error = 0.0
        
        if left_in_range and right_in_range:
            # Dos paredes: centrado equidistante en el medio del pasillo
            error = (left_d - right_d) / 2.0
        elif left_in_range:
            # Solo pared izquierda: banda de mantenimiento 10-15 cm
            if left_d < WALL_MIN_DISTANCE:
                error = left_d - WALL_MIN_DISTANCE # Negativo -> corrección hacia la derecha
            elif left_d > WALL_MAX_DISTANCE:
                error = left_d - WALL_MAX_DISTANCE # Positivo -> corrección hacia la izquierda
            else:
                error = 0.0 # En la zona ideal de 10 a 15 cm
        elif right_in_range:
            # Solo pared derecha: banda de mantenimiento 10-15 cm
            if right_d < WALL_MIN_DISTANCE:
                error = WALL_MIN_DISTANCE - right_d # Positivo -> corrección hacia la izquierda
            elif right_d > WALL_MAX_DISTANCE:
                error = WALL_MAX_DISTANCE - right_d # Negativo -> corrección hacia la derecha
            else:
                error = 0.0 # En la zona ideal de 10 a 15 cm
        else:
            # Espacio abierto o paredes fuera de rango: no perturbamos el rumbo
            if self.wall_pid_active:
                self.reset_wall_pid()
            self.last_pid_correction = 0.0
            return 0.0
            
        self.wall_pid_active = True
        safe_dt = max(0.005, min(0.1, dt))
        
        # Término P (Proporcional)
        p_term = self.wall_pid_kp * error
        
        # Término I (Integral) con anti-windup
        self.wall_pid_integral += error * safe_dt
        self.wall_pid_integral = max(-0.030, min(0.030, self.wall_pid_integral))
        i_term = self.wall_pid_ki * self.wall_pid_integral
        
        # Término D (Derivativo) para amortiguar y evitar oscilaciones
        d_term = self.wall_pid_kd * (error - self.wall_pid_prev_error) / safe_dt
        self.wall_pid_prev_error = error
        
        pid_output = p_term + i_term + d_term
        max_rad = math.radians(WALL_PID_MAX_CORRECTION_DEG)
        self.last_pid_correction = max(-max_rad, min(max_rad, pid_output))
        return self.last_pid_correction

    def check_front_wall_emergency(self, front_d):
        """
        Frenada de emergencia EXCLUSIVA para el muro frontal:
        Si tenemos la pared enfrente a menos de 10 cm, nos paramos de inmediato.
        """
        return front_d < FRONT_WALL_STOP_THRESHOLD

    def get_grid_offset(self, target_heading):
        """Calcula el desplazamiento (dx, dy) según el rumbo cardinal."""
        if target_heading == 0: return (1, 0)
        elif target_heading == 90: return (0, 1)
        elif target_heading == 180: return (-1, 0)
        elif target_heading == 270 or target_heading == -90: return (0, -1)
        return (0, 0)
        
    def get_heading_to_target(self, curr_x, curr_y, target_x, target_y):
        """Averigua hacia qué rumbo cardinal hay que apuntar para ir a la celda vecina."""
        dx = target_x - curr_x
        dy = target_y - curr_y
        if abs(dx) >= abs(dy):
            return 0 if dx >= 0 else 180
        else:
            return 90 if dy > 0 else 270
        
    def get_filtered_front_tof(self):
        """Filtro de mediana móvil de 5 muestras para el sensor frontal."""
        valid = [x for x in self.front_tof_buffer if not math.isnan(x)]
        if len(valid) > 0:
            return statistics.median(valid)
        return 4.0

    def get_filtered_left_tof(self):
        """Filtro de mediana móvil para el sensor izquierdo."""
        valid = [x for x in self.left_tof_buffer if not math.isnan(x)]
        if len(valid) > 0:
            return statistics.median(valid)
        return 2.0

    def get_filtered_right_tof(self):
        """Filtro de mediana móvil para el sensor derecho."""
        valid = [x for x in self.right_tof_buffer if not math.isnan(x)]
        if len(valid) > 0:
            return statistics.median(valid)
        return 2.0
        
    def get_battery_voltage(self):
        """Estima el voltaje de la batería LiPo a partir de los Joules consumidos."""
        joules = self.robot.batterySensorGetValue()
        if math.isnan(joules) or joules < 0:
            return 0.0
        max_j = 25920.0
        min_v = 6.0
        max_v = 8.7
        j_clamped = max(0.0, min(max_j, joules))
        return min_v + (j_clamped / max_j) * (max_v - min_v)
        
    def update_oled(self):
        """Dibuja en la pantallita OLED del chasis lo que detecta la visión."""
        if not self.oled: return
        w = self.oled.getWidth()
        h = self.oled.getHeight()
        self.oled.setColor(0x000000)
        self.oled.fillRectangle(0, 0, w, h)
        
        with self.lock:
            aruco_txt = self.shared_aruco_text
            color_txt = self.shared_color_text
            if time.time() > self.aruco_clear_time:
                self.shared_aruco_text = ""
                
        self.oled.setFont("Arial", 10, True)
        if aruco_txt:
            self.oled.setColor(0x00FF00) # Verde para ArUco
            self.oled.drawText(aruco_txt, 5, 10)
        if color_txt:
            self.oled.setColor(0xFFFFFF) # Blanco para Color
            self.oled.drawText(color_txt, 5, 30)

    def get_true_yaw(self):
        """Obtiene la orientación física REAL del robot (ground truth libre de ruido)."""
        if self.imu_ideal:
            raw_ideal = get_yaw_from_quaternion(self.imu_ideal.getQuaternion())
            if self.yaw_offset_ideal is not None:
                return math.degrees(normalize_angle(raw_ideal - self.yaw_offset_ideal)) % 360.0
            return math.degrees(raw_ideal) % 360.0
        if self.compass:
            vals = self.compass.getValues()
            if vals and not (math.isnan(vals[0]) or math.isnan(vals[1])):
                rad = math.atan2(vals[0], vals[1])
                bearing = (rad - 1.5708) / math.pi * 180.0
                return bearing % 360.0
        return 0.0

    def print_god_mode_telemetry(self, event_label=""):
        """Imprime la telemetría completa comparando la percepción del robot con la realidad física."""
        true_x, true_y = 0.0, 0.0
        if self.gps:
            g_vals = self.gps.getValues()
            if g_vals and not math.isnan(g_vals[0]):
                true_x, true_y = g_vals[0], g_vals[1]
                
        true_yaw = self.get_true_yaw()
        front_dist = self.get_filtered_front_tof()
        left_dist = self.get_filtered_left_tof()
        right_dist = self.get_filtered_right_tof()
        current_yaw_deg = math.degrees(self.current_yaw) if self.current_yaw is not None else 0.0
        
        if self.start_gps_x is None and (true_x != 0.0 or true_y != 0.0):
            self.start_gps_x = true_x
            self.start_gps_y = true_y
            
        dx = (true_x - self.start_gps_x) if self.start_gps_x is not None else 0.0
        dy = (true_y - self.start_gps_y) if self.start_gps_y is not None else 0.0
        real_cell_x = int(round(dx / CELL_SIZE))
        real_cell_y = int(round(dy / CELL_SIZE))
        expected_dx = self.grid_x * CELL_SIZE
        expected_dy = self.grid_y * CELL_SIZE
        err_x_cm = (dx - expected_dx) * 100.0
        err_y_cm = (dy - expected_dy) * 100.0
        
        yaw_drift_deg = math.degrees(normalize_angle(math.radians(current_yaw_deg - true_yaw)))
        pid_deg = math.degrees(self.last_pid_correction)
        status_suffix = f" [{event_label}]" if event_label else ""
        msg = (
            f"\n[DEBUG-SYNC] State: {self.state}{status_suffix}\n"
            f" -> CASILLA ESTIMADA: ({self.grid_x}, {self.grid_y}) | CASILLA REAL (GPS vs Verde 0,0): ({real_cell_x}, {real_cell_y}) | Offset Celda: (dX={err_x_cm:+.1f}cm, dY={err_y_cm:+.1f}cm)\n"
            f" -> IMU YAW: {current_yaw_deg:6.1f}° | COMPASS REAL: {true_yaw:6.1f}° | IMU DRIFT: {yaw_drift_deg:+5.1f}°\n"
            f" -> MOTOR VOLTAJE: 7.8V (Max: 386.1 RPM) | L={self.motor_bias_left:.3f}, R={self.motor_bias_right:.3f} | Micro-Slip Std: {self.motor_slip_std:.3f}\n"
            f" -> ToF NOISY -> F (VL53L1X): {front_dist:.3f}m | L (VL53L0X): {left_dist:.3f}m | R (VL53L0X): {right_dist:.3f}m\n"
            f" -> WALL CONTROL: Lateral PID (10-15cm band)={pid_deg:+.1f}° (Active={self.wall_pid_active})\n"
            f" -> Pitch (Filtered): {self.filtered_pitch:.1f}° | Terrain: {self.terrain_phase}"
        )
        log_debug(msg)

    def update(self):
        """Bucle principal de ejecución que se llama en cada ciclo de simulación."""
        t = self.robot.getTime()
        if t > 240.0:
            print("\n[TIMEOUT] Se alcanzó el tiempo máximo de simulación (240s).\n")
            flag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "finished.flag")
            try:
                with open(flag_file, "w") as f:
                    f.write("TIMEOUT")
            except Exception:
                pass
            sys.exit(1)
            
        # 1. Simulación de deriva temporal intrínseca del giróscopo MPU-6050
        if self.last_gyro_time is not None:
            dt = t - self.last_gyro_time
            if dt > 0:
                self.gyro_bias_rate += np.random.normal(0, GYRO_RANDOM_WALK_STD) * math.sqrt(dt)
                self.gyro_bias_drift += self.gyro_bias_rate * dt
        self.last_gyro_time = t

        raw_yaw = get_yaw_from_quaternion(self.imu.getQuaternion())
        raw_yaw_ideal = get_yaw_from_quaternion(self.imu_ideal.getQuaternion())
        
        # Le inyectamos la deriva al ángulo medido
        drifted_yaw = normalize_angle(raw_yaw + self.gyro_bias_drift)
        
        if self.yaw_offset is not None:
            current_yaw = normalize_angle(drifted_yaw - self.yaw_offset)
        else:
            current_yaw = 0.0
        self.current_yaw = current_yaw
            
        if self.yaw_offset_ideal is not None:
            ideal_yaw = normalize_angle(raw_yaw_ideal - self.yaw_offset_ideal)
        else:
            ideal_yaw = 0.0
        
        # Lectura y filtrado de los ToF
        raw_f = self.tof_front.getValue()
        val_f = 4.0 if (math.isnan(raw_f) or raw_f < 0) else min(4.0, raw_f / 1000.0)
        self.front_tof_buffer.append(val_f)
        
        raw_l = self.tof_left.getValue()
        val_l = 2.0 if (math.isnan(raw_l) or raw_l < 0) else min(2.0, raw_l / 1000.0)
        self.left_tof_buffer.append(val_l)
        
        raw_r = self.tof_right.getValue()
        val_r = 2.0 if (math.isnan(raw_r) or raw_r < 0) else min(2.0, raw_r / 1000.0)
        self.right_tof_buffer.append(val_r)
        
        # Pasamos las imágenes capturadas al hilo de visión
        upper_img = self.upper_camera.getImage()
        lower_img = self.lower_camera.getImage()
        
        with self.lock:
            if upper_img:
                self.upper_frame_data = np.frombuffer(upper_img, np.uint8).reshape((self.upper_camera.getHeight(), self.upper_camera.getWidth(), 4))
            if lower_img:
                self.lower_frame_data = np.frombuffer(lower_img, np.uint8).reshape((self.lower_camera.getHeight(), self.lower_camera.getWidth(), 4))
                
        self.update_oled()
        
        # ======================================================================
        # MÁQUINA DE ESTADOS FINITA (FSM)
        # ======================================================================
        
        # --- ESTADO 1: CALIBRATE (Reposo inicial) ---
        if self.state == "CALIBRATE":
            if t - self.last_telemetry_time >= 2.5:
                volts = self.get_battery_voltage()
                deg = math.degrees(current_yaw)
                print(f"[CALIBRATE] Yaw: {deg:5.1f}° | ToF -> F: {val_f:.2f}m | L: {val_l:.2f}m | R: {val_r:.2f}m | Bat: {volts:.2f}V")
                self.last_telemetry_time = t
                
            # Dejamos quieto el robot 3 segundos para que los sensores se estabilicen
            if t < 3.0:
                self.set_motors(0.0, 0.0)
            else:
                self.yaw_offset = drifted_yaw
                self.yaw_offset_ideal = raw_yaw_ideal
                self.pitch_offset = self.imu.getRollPitchYaw()[1]
                self.filtered_pitch = 0.0
                self.terrain_phase = "FLAT"
                if self.gps:
                    g = self.gps.getValues()
                    if g and not math.isnan(g[0]):
                        self.start_gps_x = g[0]
                        self.start_gps_y = g[1]
                print(f"[CALIBRATE] Calibración completa. Yaw Offset: {math.degrees(self.yaw_offset):.2f}°, Pitch Offset: {math.degrees(self.pitch_offset):.2f}°")
                self.print_god_mode_telemetry("CALIBRATION COMPLETE")
                self.state = "SCAN"
                
        # --- ESTADO 2: SCAN (Punto de control alcanzado) ---
        elif self.state == "SCAN":
            self.set_motors(0.0, 0.0)
            print(f"[SCAN] Checkpoint alcanzado en la celda ({self.grid_x}, {self.grid_y}).")
            
            with self.lock:
                do_return = self.found_red_goal
            if do_return:
                print("\n[MAZE] ¡META ROJA ALCANZADA! Iniciando RETURN_HOME...\n")
                self.print_god_mode_telemetry("RED GOAL REACHED")
                self.state = "RETURN_HOME"
                return
            self.state = "DECIDE"
            
        # --- ESTADO 3: DECIDE (Toma de decisiones con algoritmo DFS) ---
        elif self.state == "DECIDE":
            self.set_motors(0.0, 0.0)
            print(f"[DECIDE] Procesando celda ({self.grid_x}, {self.grid_y}) | Stack: {self.route_stack}")
            
            filtered_df = self.get_filtered_front_tof()
            dr = self.get_filtered_right_tof()
            dl = self.get_filtered_left_tof()
            
            robot_heading = get_compass_heading(current_yaw)
            # Prioridad de exploración: 1. Recto, 2. Derecha, 3. Izquierda
            directions = [
                (filtered_df, robot_heading),
                (dr, (robot_heading - 90) % 360),
                (dl, (robot_heading + 90) % 360)
            ]
            
            valid_neighbors = []
            for tof_dist, target_heading in directions:
                # Si la distancia es mayor a 28 cm, es un pasillo abierto
                if tof_dist > NEIGHBOR_WALL_THRESHOLD:
                    dx, dy = self.get_grid_offset(target_heading)
                    nx = self.grid_x + dx
                    ny = self.grid_y + dy
                    # Solo visitamos si no hemos estado antes ahí
                    if (nx, ny) not in self.visited:
                        valid_neighbors.append((nx, ny, target_heading))
            
            if valid_neighbors:
                chosen_nx, chosen_ny, target_heading = valid_neighbors[0]
                self.visited.add((chosen_nx, chosen_ny))
                self.route_stack.append((chosen_nx, chosen_ny))
                self.target_yaw = math.radians(target_heading)
                self.target_cell = (chosen_nx, chosen_ny)
                print(f"[DECIDE] EXPLORAR: Hacia ({chosen_nx}, {chosen_ny}) desde ({self.grid_x}, {self.grid_y}) rumbo {target_heading}°")
                self.print_god_mode_telemetry(f"TARGET ({chosen_nx}, {chosen_ny})")
                self.state = "TURN"
            else:
                # Callejón sin salida: desandamos el camino con el hilo de oro (BACKTRACK)
                if len(self.route_stack) > 1:
                    dead_end = self.route_stack.pop()
                    parent_x, parent_y = self.route_stack[-1]
                    heading_deg = self.get_heading_to_target(self.grid_x, self.grid_y, parent_x, parent_y)
                    self.target_yaw = math.radians(heading_deg)
                    self.target_cell = (parent_x, parent_y)
                    print(f"[DECIDE] RETROCESO DFS: Saliendo del dead-end {dead_end} hacia ({parent_x}, {parent_y}) rumbo {heading_deg}°")
                    self.print_god_mode_telemetry(f"BACKTRACK ({parent_x}, {parent_y})")
                    self.state = "TURN"
                else:
                    # En la celda de inicio (0, 0), verificamos si la dirección trasera (180°) no ha sido explorada aún
                    behind_heading = (robot_heading + 180) % 360
                    dx, dy = self.get_grid_offset(behind_heading)
                    bx = self.grid_x + dx
                    by = self.grid_y + dy
                    if (bx, by) not in self.visited:
                        self.visited.add((bx, by))
                        self.route_stack.append((bx, by))
                        self.target_yaw = math.radians(behind_heading)
                        self.target_cell = (bx, by)
                        print(f"[DECIDE] EXPLORAR RETAGUARDIA EN (0,0): Rumbo {behind_heading}° hacia ({bx}, {by})")
                        self.print_god_mode_telemetry(f"TARGET REAR ({bx}, {by})")
                        self.state = "TURN"
                    else:
                        print("\n*** SIMULACIÓN COMPLETA: TODAS LAS CELDAS ACCESIBLES FUERON EXPLORADAS. ESTACIONADO EN (0,0) ***\n")
                        self.print_god_mode_telemetry("EXPLORATION COMPLETE")
                        self.set_motors(0.0, 0.0)
                        self.state = "FINISHED"
                    
        # --- ESTADO 4: RETURN_HOME (Vuelta triunfal a casa tras encontrar la meta) ---
        elif self.state == "RETURN_HOME":
            self.set_motors(0.0, 0.0)
            if len(self.route_stack) > 1:
                curr = self.route_stack.pop()
                target_x, target_y = self.route_stack[-1]
                heading_deg = self.get_heading_to_target(self.grid_x, self.grid_y, target_x, target_y)
                self.target_yaw = math.radians(heading_deg)
                self.target_cell = (target_x, target_y)
                print(f"[RETURN_HOME] Volviendo desde {curr} hacia ({target_x}, {target_y}) rumbo {heading_deg}°")
                self.print_god_mode_telemetry(f"RETURN_HOME ({target_x}, {target_y})")
                self.state = "TURN"
            else:
                if len(self.route_stack) == 1:
                    self.route_stack.pop()
                self.set_motors(0.0, 0.0)
                print("\n*** ¡SIMULACIÓN COMPLETA: REGRESAMOS SANOS Y SALVOS A (0, 0)! ***\n")
                self.print_god_mode_telemetry("VICTORY RETURNED HOME")
                self.state = "FINISHED"
                
        # --- ESTADO 5: TURN (Giro sobre su propio eje) ---
        elif self.state == "TURN":
            error = normalize_angle(self.target_yaw - current_yaw)
            turn_speed = TURN_KP * error
            if abs(turn_speed) < MIN_TURN_SPEED:
                turn_speed = math.copysign(MIN_TURN_SPEED, turn_speed)
            self.set_motors(-turn_speed, turn_speed)
            if abs(error) < 0.04: # Margen de giro de ~2.3 grados
                self.set_motors(0.0, 0.0)
                self.state = "SETTLE"
                self.settle_time = t
                
        # --- ESTADO 6: SETTLE (Pausa breve para estabilizar balanceo) ---
        elif self.state == "SETTLE":
            self.set_motors(0.0, 0.0)
            if t - self.settle_time > 0.20: # 200 ms de estabilización tras el giro
                self.reset_wall_pid()
                self.move_start_time = t
                self.start_front_d = self.get_filtered_front_tof()
                self.terrain_phase = "FLAT"
                self.flat_ticks = 0
                raw_p = normalize_angle(self.imu.getRollPitchYaw()[1] - self.pitch_offset)
                self.filtered_pitch = math.degrees(raw_p)
                self.state = "MOVE"
                
        # --- ESTADO 7: MOVE (Avance celda a celda) ---
        elif self.state == "MOVE":
            # 1. Odometría sin encoders: Dead-Reckoning temporal a 7.8V estabilizados
            elapsed_move_time = max(0.0, t - self.move_start_time)
            dist_dr = elapsed_move_time * DEAD_RECKONING_LINEAR_SPEED
            
            front_d = self.get_filtered_front_tof()
            delta_front_d = (self.start_front_d - front_d) if (self.start_front_d < 3.0 and front_d < 3.0) else 0.0
            
            # 2. Filtrado de inclinación (Pitch) para saber si subimos o bajamos rampas
            raw_p = normalize_angle(self.imu.getRollPitchYaw()[1] - self.pitch_offset)
            raw_pitch_deg = math.degrees(raw_p)
            self.filtered_pitch = (0.2 * raw_pitch_deg) + (0.8 * self.filtered_pitch)
            abs_pitch = abs(self.filtered_pitch)
            
            left_d = self.get_filtered_left_tof()
            right_d = self.get_filtered_right_tof()
            
            if t - self.last_telemetry_time >= 2.5:
                self.print_god_mode_telemetry("PERIODIC")
                self.last_telemetry_time = t
                
            # 3. Detección de rampas y desniveles
            if self.terrain_phase == "FLAT":
                if self.filtered_pitch > 10.0:
                    self.terrain_phase = "UP"
                    self.flat_ticks = 0
            elif self.terrain_phase == "UP":
                if self.filtered_pitch < -10.0:
                    self.terrain_phase = "DOWN"
                elif abs_pitch < 3.0:
                    self.flat_ticks += 1
                    if self.flat_ticks >= 6:
                        self.terrain_phase = "FLAT"
                else:
                    self.flat_ticks = 0
            elif self.terrain_phase == "DOWN":
                if abs_pitch < 3.0:
                    self.flat_ticks += 1
                    if self.flat_ticks >= 5:
                        self.terrain_phase = "FLAT"
                        self.set_motors(0.0, 0.0)
                        self.grid_x, self.grid_y = self.target_cell
                        self.print_god_mode_telemetry("RAMP COMPLETE")
                        with self.lock:
                            self.state = "SCAN"
                        return
                else:
                    self.flat_ticks = 0
                    
            # 4. REPULSIÓN DE EMERGENCIA EXCLUSIVA ANTE MURO FRONTAL (< 10 cm)
            # Solo se activa en plano; si estamos en rampa el cabeceo del chasis puede acercar el sensor al suelo
            if self.terrain_phase == "FLAT" and abs_pitch < 6.0 and self.check_front_wall_emergency(front_d):
                self.set_motors(0.0, 0.0)
                self.terrain_phase = "FLAT"
                self.print_god_mode_telemetry("FRONT WALL EMERGENCY BRAKE")
                # Si ya avanzamos casi toda la celda (>= 25 cm), confirmamos que llegamos
                if dist_dr >= 0.25:
                    self.grid_x, self.grid_y = self.target_cell
                else:
                    print(f"[EMERGENCY] Obstrucción frontal temprana a {dist_dr:.2f}m < 0.25m. Manteniendo ({self.grid_x}, {self.grid_y})")
                    if self.target_cell in self.route_stack:
                        self.route_stack.remove(self.target_cell)
                self.backup_start_time = t
                self.state = "BACKUP"
                return
                
            # 5. Condición de fin de celda (Recorrimos 30 cm o llegamos al centro de celda con muro frontal)
            # Exclusivo de terreno PLANO; en rampas (UP/DOWN) el fin de casilla lo define el nivelado del chasis
            cell_reached = False
            if self.terrain_phase == "FLAT" and abs_pitch < 5.0:
                # Caso A: Muro frontal al frente (distancia al muro en centro de casilla es ~15 cm)
                if dist_dr >= 0.22 and front_d <= FRONT_WALL_CLEAR_THRESHOLD:
                    cell_reached = True
                # Caso B: Recorrido completo por Dead-Reckoning (30 cm a 8.4 cm/s = 3.57s)
                elif dist_dr >= CELL_SIZE:
                    cell_reached = True
                # Caso C: Validado por ToF diferencial si ya avanzó la celda completa (30 cm)
                elif dist_dr >= 0.29 and delta_front_d >= CELL_SIZE:
                    cell_reached = True
                
            if cell_reached:
                self.set_motors(0.0, 0.0)
                self.grid_x, self.grid_y = self.target_cell
                self.terrain_phase = "FLAT"
                self.print_god_mode_telemetry("CELL STEP COMPLETE")
                with self.lock:
                    self.state = "SCAN"
                return
                    
            # 6. CONTROL LATERAL PID (Banda 10-15 cm y Centrado Equidistante)
            current_yaw = self.current_yaw
            dt = TIME_STEP / 1000.0
            if self.terrain_phase == "FLAT" and abs_pitch <= 3.0:
                correction = self.compute_lateral_pid(left_d, right_d, dt)
                dynamic_target_yaw = normalize_angle(self.target_yaw + correction)
                
                # RE-ALINEACIÓN ACTIVA DE RUMBO CON LAS PAREDES (Absorción de Deriva Sim2Real)
                # Estimamos la orientación física real del carro respecto a la(s) pared(es)
                # comparando el desplazamiento lateral en el tiempo con la velocidad de avance.
                left_in_range = left_d < CORRIDOR_MAX_LATERAL_DIST
                right_in_range = right_d < CORRIDOR_MAX_LATERAL_DIST
                
                metric = None
                if left_in_range and right_in_range:
                    metric = (left_d - right_d) / 2.0
                elif left_in_range:
                    metric = left_d
                elif right_in_range:
                    metric = -right_d
                    
                if metric is not None:
                    self.lateral_history.append((t, metric))
                    
                # Si tenemos muestras suficientes y avanzamos de forma estable en la celda:
                if len(self.lateral_history) >= 5 and (0.06 < dist_dr < 0.26):
                    t_old, m_old = self.lateral_history[0]
                    dt_hist = t - t_old
                    if dt_hist > 0.08:
                        ds = max(0.005, DEAD_RECKONING_LINEAR_SPEED * dt_hist)
                        # theta_walls: ángulo físico relativo al eje del pasillo
                        theta_walls = - (metric - m_old) / ds
                        theta_walls = max(-math.radians(15.0), min(math.radians(15.0), theta_walls))
                        
                        # Discrepancia angular entre el giróscopo y la física real de las paredes
                        yaw_drift_err = normalize_angle(current_yaw - self.target_yaw - theta_walls)
                        
                        # Absorción suave en el yaw_offset (elimina la deriva progresivamente sin sacudidas)
                        alpha = 0.035
                        drift_step = alpha * yaw_drift_err
                        self.yaw_offset = normalize_angle(self.yaw_offset + drift_step)
                        current_yaw = normalize_angle(self.current_yaw - drift_step)
                        self.current_yaw = current_yaw
            else:
                self.reset_wall_pid()
                dynamic_target_yaw = self.target_yaw
                
            error = normalize_angle(dynamic_target_yaw - current_yaw)
            base_speed = BASE_SPEED_RAMP if self.terrain_phase in ("UP", "DOWN") else BASE_SPEED_FLAT
            left_s = base_speed - (3.0 * error)
            right_s = base_speed + (3.0 * error)
            self.set_motors(left_s, right_s)
            
        # --- ESTADO 8: BACKUP (Echarse para atrás si estamos pegados a un muro frontal) ---
        elif self.state == "BACKUP":
            self.set_motors(BACKUP_SPEED, BACKUP_SPEED)
            clearance = self.get_filtered_front_tof()
            # Retrocedemos hasta despejar 15 cm o por seguridad tras 1.2 segundos
            if clearance >= FRONT_WALL_CLEAR_THRESHOLD or (t - self.backup_start_time > 1.2):
                self.set_motors(0.0, 0.0)
                print(f"[BACKUP] Despeje frontal seguro: {clearance:.3f}m >= {FRONT_WALL_CLEAR_THRESHOLD:.2f}m.")
                with self.lock:
                    self.state = "SCAN"

        # --- ESTADO 9: FINISHED (Fin de la misión) ---
        elif self.state == "FINISHED":
            self.set_motors(0.0, 0.0)
            true_x, true_y = 0.0, 0.0
            if self.gps:
                g_vals = self.gps.getValues()
                if g_vals and not math.isnan(g_vals[0]):
                    true_x, true_y = g_vals[0], g_vals[1]
            dx = (true_x - self.start_gps_x) if self.start_gps_x is not None else 0.0
            dy = (true_y - self.start_gps_y) if self.start_gps_y is not None else 0.0
            real_cell_x = int(round(dx / CELL_SIZE))
            real_cell_y = int(round(dy / CELL_SIZE))
            print("\n==================================================================")
            print("[REPORTE FINAL DE MISIÓN - CASILLAS]")
            print(f" -> Casilla final estimada por el robot : ({self.grid_x}, {self.grid_y})")
            print(f" -> Casilla final REAL (GPS vs Verde 0,0): ({real_cell_x}, {real_cell_y})")
            if self.grid_x == real_cell_x and self.grid_y == real_cell_y:
                print(" -> VERIFICACIÓN: ¡COINCIDENCIA EXACTA! El robot contó y mapeó cada casilla sin errores.")
            else:
                print(f" -> VERIFICACIÓN: Discrepancia detectada: dX={self.grid_x - real_cell_x}, dY={self.grid_y - real_cell_y}")
            print("==================================================================\n")
            flag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "finished.flag")
            try:
                with open(flag_file, "w") as f:
                    f.write("FINISHED")
            except Exception:
                pass
            sys.exit(0)

if __name__ == '__main__':
    solver = MazeSolver()
    while solver.robot.step(TIME_STEP) != -1:
        solver.update()
