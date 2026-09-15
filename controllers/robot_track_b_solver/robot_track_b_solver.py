"""
Controlador del Robot para la Pista B (Niveles) - Webots Sim2Real
Candidates Principiantes 2026 - RoBorregos

Hardware y arquitectura física idénticos a Pista A:
- Motores diferenciales N20 comandados a 7.8V (Dead-reckoning de precisión)
- Sensores ToF láser (Frontal VL53L1X, Laterales VL53L0X)
- IMU MPU-6050 (Acelerómetro + Giróscopo)
- Cámara inferior picada al suelo (PixyMon por SPI):
  * Discriminación cromática de baldosas (Cyan, Amarillo, Naranja, Magenta, Rojo, Verde)
  * Detección de líneas blancas de obstáculo y hueco libre en Sección 2
  * Detección y seguimiento de la pelota de golf naranja en Sección 1
- Cámara frontal (Webcam USB):
  * Detección de la pelota a mayor distancia y obstáculos
- Pantalla OLED I2C (128x64): Telemetría visual en tiempo real
- Pala frontal (Scoop) para retención y transporte de la pelota de golf
"""

from controller import Robot
import math
import cv2
import numpy as np
import time
import sys
import os

# --- Logger persistente hacia god_mode_track_b.log ---
LOG_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "god_mode_track_b.log")

def log_robot(msg):
    print(msg, flush=True)
    try:
        with open(LOG_FILE_PATH, "a", encoding="utf-8") as f:
            f.write(f"{msg}\n")
            f.flush()
    except Exception:
        pass

# --- Constantes Cinemáticas y Visión ---
TIME_STEP = 32
WHEEL_RADIUS = 0.02
TRACK_WIDTH = 0.09
CELL_SIZE = 0.30

class TrackBRobot(Robot):
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        
        # --- Motores Diferenciales ---
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # --- Encoders de Ruedas ---
        self.left_sensor = self.getDevice("left wheel sensor")
        self.right_sensor = self.getDevice("right wheel sensor")
        if self.left_sensor: self.left_sensor.enable(self.time_step)
        if self.right_sensor: self.right_sensor.enable(self.time_step)
        
        # --- Sensores de Distancia ToF Láser ---
        self.tof_front = self.getDevice("tof_front")
        self.tof_left = self.getDevice("tof_left")
        self.tof_right = self.getDevice("tof_right")
        self.tof_front.enable(self.time_step)
        self.tof_left.enable(self.time_step)
        self.tof_right.enable(self.time_step)
        
        # --- IMU y Sensores Inerciales ---
        self.imu = self.getDevice("imu")
        self.imu.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)
        
        # 1. Cámara inferior picada al suelo (PixyMon)
        self.camera_floor = self.getDevice("camera_down")
        if not self.camera_floor:
            self.camera_floor = self.getDevice("camera")
        self.camera_floor.enable(self.time_step)
        self.cam_floor_w = self.camera_floor.getWidth()
        self.cam_floor_h = self.camera_floor.getHeight()
        
        # 2. Cámara frontal (Webcam)
        self.camera_front = self.getDevice("main_camera")
        if self.camera_front:
            self.camera_front.enable(self.time_step * 2)
            self.cam_front_w = self.camera_front.getWidth()
            self.cam_front_h = self.camera_front.getHeight()
        
        # --- Pantalla OLED ---
        self.display = self.getDevice("oled_display")
        self.update_display("PISTA B: READY", "INICIO")
        
        # --- Variables de Percepción Visual ---
        self.current_floor_color = "UNKNOWN"
        self.white_line_detected = False
        self.white_line_offset = 0.0  # Desviación respecto al centro
        self.ball_detected = False
        self.ball_bearing = 0.0       # Ángulo hacia la pelota
        self.ball_area = 0.0          # Área del blob de la pelota
        
        log_robot(f"[ROBOT INIT] Robot Track B Solver inicializado. Cámara de suelo: {self.cam_floor_w}x{self.cam_floor_h}")

    def update_display(self, line1, line2=""):
        """Muestra texto en pantalla OLED de 128x64."""
        if self.display:
            self.display.setColor(0x000000)
            self.display.fillRectangle(0, 0, 128, 64)
            self.display.setColor(0x00FF00)
            self.display.setFont("Arial", 10, True)
            self.display.drawText(line1, 4, 12)
            if line2:
                self.display.setColor(0xFFFFFF)
                self.display.drawText(line2, 4, 34)

    def set_speeds(self, v_left, v_right):
        """Asigna velocidades angulares limitadas a los motores."""
        max_vel = 6.28
        vl = max(-max_vel, min(max_vel, v_left))
        vr = max(-max_vel, min(max_vel, v_right))
        self.left_motor.setVelocity(vl)
        self.right_motor.setVelocity(vr)

    def stop(self):
        self.set_speeds(0.0, 0.0)

    def get_floor_image(self):
        """Captura cuadro de la cámara inferior picada al suelo en formato OpenCV (BGR y HSV)."""
        raw = self.camera_floor.getImage()
        if raw is None:
            return None, None
        frame = np.frombuffer(raw, np.uint8).reshape((self.cam_floor_h, self.cam_floor_w, 4))
        bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        return bgr, hsv

    def get_front_image(self):
        """Captura cuadro de la cámara frontal (Webcam)."""
        if not self.camera_front:
            return None, None
        raw = self.camera_front.getImage()
        if raw is None:
            return None, None
        frame = np.frombuffer(raw, np.uint8).reshape((self.cam_front_h, self.cam_front_w, 4))
        bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        return bgr, hsv

    def analyze_floor_vision(self, hsv):
        """
        Procesamiento de visión por computadora para la cámara picada al suelo:
        1. Identificación estricta del color de la baldosa según el reglamento:
           - Verde [35..85]: Inicio o Meta
           - Rojo [0..10 o 160..180]: Checkpoints 1 y 2
           - Naranja [11..22]: Sigue ADELANTE
           - Amarillo [23..38]: Giro a la IZQUIERDA
           - Cyan [85..105]: Giro a la DERECHA
           - Magenta [130..160]: Giro hacia ATRÁS (180°)
        2. Detección de líneas de cinta blanca (Sección 2).
        3. Detección de la pelota de golf naranja (Sección 1).
        """
        if hsv is None:
            return

        h, w = hsv.shape[:2]
        
        # --- 1. CLASIFICACIÓN DE COLOR DE BALDOSA (Región frontal del suelo) ---
        # En la imagen hacia abajo, las filas superiores (0..60) corresponden al suelo frente a la pala
        cx = w // 2
        roi = hsv[5:55, max(0, cx - 35):min(w, cx + 35)]
        
        # Máscaras HSV
        mask_green = cv2.inRange(roi, np.array([35, 70, 50]), np.array([85, 255, 255]))
        mask_red1 = cv2.inRange(roi, np.array([0, 100, 80]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(roi, np.array([160, 100, 80]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        mask_orange = cv2.inRange(roi, np.array([11, 100, 100]), np.array([22, 255, 255]))
        mask_yellow = cv2.inRange(roi, np.array([23, 100, 100]), np.array([38, 255, 255]))
        mask_cyan = cv2.inRange(roi, np.array([85, 100, 100]), np.array([105, 255, 255]))
        mask_magenta = cv2.inRange(roi, np.array([130, 100, 100]), np.array([160, 255, 255]))
        
        counts = {
            "GREEN": cv2.countNonZero(mask_green),
            "RED": cv2.countNonZero(mask_red),
            "ORANGE": cv2.countNonZero(mask_orange),
            "YELLOW": cv2.countNonZero(mask_yellow),
            "CYAN": cv2.countNonZero(mask_cyan),
            "MAGENTA": cv2.countNonZero(mask_magenta),
        }
        
        best_color, best_count = max(counts.items(), key=lambda x: x[1])
        min_pixels = 150
        if best_count > min_pixels:
            self.current_floor_color = best_color
        else:
            self.current_floor_color = "WHITE_OR_NEUTRAL"

        # --- 2. DETECCIÓN DE LÍNEAS BLANCAS (Sección 2) ---
        # Evaluar exclusivamente sobre el suelo visible (filas 0..60)
        floor_region = hsv[0:60, :]
        mask_white = cv2.inRange(floor_region, np.array([0, 0, 200]), np.array([180, 50, 255]))
        white_pixels = cv2.countNonZero(mask_white)
        
        if white_pixels > 250:
            self.white_line_detected = True
            # Calcular centroide horizontal de la línea blanca
            moments = cv2.moments(mask_white)
            if moments["m00"] > 0:
                line_cx = moments["m01"] / moments["m00"]
                self.white_line_offset = (line_cx - (w / 2.0)) / (w / 2.0)
        else:
            self.white_line_detected = False
            self.white_line_offset = 0.0

        # --- 3. DETECCIÓN DE PELOTA DE GOLF (Sección 1) ---
        # Pelota naranja brillante: H in [10, 22], S > 140, V > 140
        mask_ball = cv2.inRange(hsv, np.array([10, 140, 140]), np.array([22, 255, 255]))
        ball_pixels = cv2.countNonZero(mask_ball)
        
        if ball_pixels > 80:
            self.ball_detected = True
            moments = cv2.moments(mask_ball)
            if moments["m00"] > 0:
                bcx = moments["m10"] / moments["m00"]
                self.ball_bearing = (bcx - (w / 2.0)) / (w / 2.0) # -1.0 izquierda, +1.0 derecha
                self.ball_area = ball_pixels
        else:
            self.ball_detected = False
            self.ball_bearing = 0.0
            self.ball_area = 0.0

    def run(self):
        """Ciclo principal de telemetría y percepción."""
        step_count = 0
        
        while self.step(self.time_step) != -1:
            step_count += 1
            
            # 1. Percepción visual desde cámara picada al suelo
            bgr_floor, hsv_floor = self.get_floor_image()
            self.analyze_floor_vision(hsv_floor)
            
            # 2. Telemetría de sensores ToF e IMU
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            d_front = self.tof_front.getValue() / 1000.0
            d_left = self.tof_left.getValue() / 1000.0
            d_right = self.tof_right.getValue() / 1000.0
            
            # 3. Actualizar pantalla OLED con telemetría en vivo
            line1 = f"PISO: {self.current_floor_color}"
            if self.ball_detected:
                line2 = f"PELOTA: {self.ball_bearing:+.2f} ({int(self.ball_area)}p)"
            elif self.white_line_detected:
                line2 = f"LINEA BLANCA ({self.white_line_offset:+.2f})"
            else:
                line2 = f"ToF F:{d_front:.2f}m Y:{math.degrees(yaw):.0f}°"
                
            self.update_display(line1, line2)
            
            # 4. Log periódico de telemetría cada ~0.5 segundos
            if step_count % 15 == 0:
                log_robot(f"[ROBOT TRACK B #{step_count:04d}] "
                          f"Piso: {self.current_floor_color:<10} | "
                          f"Pelota: {self.ball_detected} (area={int(self.ball_area)}) | "
                          f"Línea: {self.white_line_detected} | "
                          f"ToF: [L:{d_left:.2f}, F:{d_front:.2f}, R:{d_right:.2f}]")

if __name__ == "__main__":
    robot = TrackBRobot()
    robot.run()
