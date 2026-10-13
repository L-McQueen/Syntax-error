"""Capa de abstracción de hardware (HAL) para Orange Pi Zero 2W.
Controla sensores I2C (/dev/i2c-2), Arduino Uno (/dev/ttyS5), visión ArUco y PixyCam SPI."""

import math
import os
import sys
import threading
import time

import cv2
import gpiod
from mpu6050 import mpu6050
import serial
import smbus2
import spidev
import VL53L0X
import vl53l1x

# Buses y direcciones
I2C_BUS_NUMBER = 2          # /dev/i2c-2 (Pines 3 y 5)
I2C_ADDR_TOF_FRONT = 0x29   # VL53L1X Frontal
I2C_ADDR_TOF_LEFT  = 0x30   # VL53L0X Lateral Izquierdo
I2C_ADDR_TOF_RIGHT = 0x31   # VL53L0X Lateral Derecho
I2C_ADDR_IMU       = 0x68   # MPU-6050
I2C_ADDR_OLED      = 0x3C   # SSD1306

UART_DEVICE = "/dev/ttyS5"
UART_BAUDRATE = 115200

SPI_BUS = 1
SPI_DEVICE = 0              # /dev/spidev1.0 para PixyCam v2
SPI_SPEED_HZ = 2000000

CAMERA_DEVICE_INDEX = 1     # /dev/video1 (Cámara USB)

# Pines XSHUT en gpiochip0
GPIO_CHIP_NAME = "/dev/gpiochip0"
PIN_XSHUT_FRONT = 228       # PH4 (Pin 16)
PIN_XSHUT_LEFT  = 256       # PI0 (Pin 29)
PIN_XSHUT_RIGHT = 271       # PI15 (Pin 31)


class RobotHAL:
    def __init__(self, enable_vision=True, enable_pixy=False):
        self.lock = threading.Lock()
        self.is_running = True

        # Sensores
        self.dist_front_cm = 999.0
        self.dist_left_cm = 999.0
        self.dist_right_cm = 999.0

        self.yaw_deg = 0.0
        self.pitch_deg = 0.0
        self.gyro_bias_z = 0.0
        self.gyro_bias_x = 0.0
        self.last_imu_time = time.time()

        # Visión y actuadores
        self.confirmed_aruco_id = None
        self.pixy_blocks = []
        self.target_left_pwm = 0
        self.target_right_pwm = 0
        self.target_servo_angle = 90

        # Dispositivos
        self.serial_port = None
        self.imu_sensor = None
        self.tof_front = None
        self.tof_left = None
        self.tof_right = None
        self.cap = None
        self.spi = None

        self.enable_vision = enable_vision
        self.enable_pixy = enable_pixy

        self._init_serial()
        self._init_tof_sensors()
        self._init_imu()
        if self.enable_vision:
            self._init_vision()
        if self.enable_pixy:
            self._init_pixy()

        self._start_threads()
        print("[HAL] Hardware Abstraction Layer iniciado correctamente.")

    def _init_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=UART_DEVICE,
                baudrate=UART_BAUDRATE,
                timeout=0.05,
                write_timeout=0.05
            )
            time.sleep(0.05)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            print(f"[HAL] UART conectado en {UART_DEVICE} a {UART_BAUDRATE} baud.")
        except Exception as e:
            print(f"[HAL-ERROR] No se pudo abrir UART: {e}")
            self.serial_port = None

    def _init_tof_sensors(self):
        chip = None
        lines = None
        if os.path.exists(GPIO_CHIP_NAME):
            try:
                chip = gpiod.Chip(GPIO_CHIP_NAME)
                lines = chip.get_lines([PIN_XSHUT_FRONT, PIN_XSHUT_LEFT, PIN_XSHUT_RIGHT])
                lines.request(consumer="tof_xshut", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0, 0, 0])
                time.sleep(0.05)
            except Exception as e:
                print(f"[HAL-WARN] Error configurando XSHUT con gpiod: {e}")

        # 1. ToF Izquierdo en 0x30
        if lines:
            try:
                lines.set_values([0, 1, 0])
                time.sleep(0.02)
            except Exception:
                pass
        try:
            self.tof_left = VL53L0X.VL53L0X(i2c_bus=I2C_BUS_NUMBER, i2c_address=0x29)
            self.tof_left.open()
            self.tof_left.change_address(I2C_ADDR_TOF_LEFT)
            self.tof_left.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
            print(f"[HAL] ToF Izquierdo VL53L0X reasignado a 0x{I2C_ADDR_TOF_LEFT:02X}")
        except Exception as e:
            print(f"[HAL-WARN] Fallo al iniciar ToF Izquierdo: {e}")
            self.tof_left = None

        # 2. ToF Derecho en 0x31
        if lines:
            try:
                lines.set_values([0, 1, 1])
                time.sleep(0.02)
            except Exception:
                pass
        try:
            self.tof_right = VL53L0X.VL53L0X(i2c_bus=I2C_BUS_NUMBER, i2c_address=0x29)
            self.tof_right.open()
            self.tof_right.change_address(I2C_ADDR_TOF_RIGHT)
            self.tof_right.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
            print(f"[HAL] ToF Derecho VL53L0X reasignado a 0x{I2C_ADDR_TOF_RIGHT:02X}")
        except Exception as e:
            print(f"[HAL-WARN] Fallo al iniciar ToF Derecho: {e}")
            self.tof_right = None

        # 3. ToF Frontal en 0x29 nativo
        if lines:
            try:
                lines.set_values([1, 1, 1])
                time.sleep(0.02)
            except Exception:
                pass
        try:
            self.tof_front = vl53l1x.VL53L1X(i2c_bus=I2C_BUS_NUMBER, i2c_address=I2C_ADDR_TOF_FRONT)
            self.tof_front.open()
            self.tof_front.start_ranging(1)
            print(f"[HAL] ToF Frontal VL53L1X activo en 0x{I2C_ADDR_TOF_FRONT:02X}")
        except Exception as e:
            print(f"[HAL-WARN] Fallo al iniciar ToF Frontal: {e}")
            self.tof_front = None

    def _init_imu(self):
        try:
            self.imu_sensor = mpu6050(I2C_ADDR_IMU, bus=I2C_BUS_NUMBER)
            print("[HAL] Calibrando IMU MPU-6050 (reposo)...")
            bias_z_accum = 0.0
            bias_x_accum = 0.0
            samples = 100
            for _ in range(samples):
                g = self.imu_sensor.get_gyro_data()
                bias_z_accum += g['z']
                bias_x_accum += g['x']
                time.sleep(0.01)

            self.gyro_bias_z = bias_z_accum / samples
            self.gyro_bias_x = bias_x_accum / samples

            accel = self.imu_sensor.get_accel_data()
            self.pitch_deg = math.atan2(accel['y'], math.sqrt(accel['x']**2 + accel['z']**2)) * (180.0 / math.pi)
            print(f"[HAL] IMU Calibrado. Sesgo Z: {self.gyro_bias_z:.3f}°/s | Sesgo X: {self.gyro_bias_x:.3f}°/s")
            self.last_imu_time = time.time()
        except Exception as e:
            print(f"[HAL-WARN] Error inicializando MPU-6050: {e}")
            self.imu_sensor = None

    def _init_vision(self):
        try:
            self.cap = cv2.VideoCapture(CAMERA_DEVICE_INDEX)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_params.minMarkerPerimeterRate = 0.1
            self.aruco_params.errorCorrectionRate = 0.3
            self.aruco_params.polygonalApproxAccuracyRate = 0.03
            if self.cap.isOpened():
                print(f"[HAL] Cámara /dev/video{CAMERA_DEVICE_INDEX} abierta para ArUco.")
            else:
                self.cap = None
        except Exception as e:
            print(f"[HAL-WARN] Fallo al iniciar cámara: {e}")
            self.cap = None

    def _init_pixy(self):
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(SPI_BUS, SPI_DEVICE)
            self.spi.max_speed_hz = SPI_SPEED_HZ
            self.spi.mode = 0
            print(f"[HAL] PixyCam v2 conectada en SPI{SPI_BUS}.{SPI_DEVICE}")
        except Exception as e:
            print(f"[HAL-WARN] Fallo al iniciar PixyCam SPI: {e}")
            self.spi = None

    def _start_threads(self):
        threading.Thread(target=self._loop_sensors, daemon=True, name="HAL-Sensors").start()
        threading.Thread(target=self._loop_motors, daemon=True, name="HAL-Motors").start()
        if self.enable_vision and self.cap:
            threading.Thread(target=self._loop_vision, daemon=True, name="HAL-Vision").start()

    def _loop_sensors(self):
        while self.is_running:
            now = time.time()
            dt = now - self.last_imu_time
            self.last_imu_time = now

            if self.imu_sensor and dt > 0:
                try:
                    gyro = self.imu_sensor.get_gyro_data()
                    accel = self.imu_sensor.get_accel_data()

                    gz = gyro['z'] - self.gyro_bias_z
                    gx = gyro['x'] - self.gyro_bias_x

                    delta_yaw = gz * dt if abs(gz) > 0.25 else 0.0
                    pitch_acc = math.atan2(accel['y'], math.sqrt(accel['x']**2 + accel['z']**2)) * (180.0 / math.pi)

                    with self.lock:
                        if delta_yaw != 0.0:
                            self.yaw_deg = (self.yaw_deg + delta_yaw) % 360.0
                        self.pitch_deg = 0.96 * (self.pitch_deg + gx * dt) + 0.04 * pitch_acc
                except Exception:
                    pass

            if self.tof_front:
                try:
                    d = self.tof_front.get_distance()
                    if d > 0:
                        with self.lock:
                            self.dist_front_cm = d / 10.0
                except Exception:
                    pass

            if self.tof_left:
                try:
                    d = self.tof_left.get_distance()
                    if d > 0:
                        with self.lock:
                            self.dist_left_cm = d / 10.0
                except Exception:
                    pass

            if self.tof_right:
                try:
                    d = self.tof_right.get_distance()
                    if d > 0:
                        with self.lock:
                            self.dist_right_cm = d / 10.0
                except Exception:
                    pass

            time.sleep(0.01)

    def _loop_motors(self):
        while self.is_running:
            if self.serial_port:
                with self.lock:
                    vl = self.target_left_pwm
                    vr = self.target_right_pwm
                    servo = self.target_servo_angle

                try:
                    self.serial_port.write(f"M,{vl},{vr},{servo}\n".encode('ascii'))
                    if self.serial_port.in_waiting > 0:
                        self.serial_port.read(self.serial_port.in_waiting)
                except Exception:
                    try:
                        self.serial_port.close()
                        time.sleep(0.1)
                        self.serial_port.open()
                    except Exception:
                        pass
            time.sleep(0.02)

    def _loop_vision(self):
        candidate_id = None
        candidate_count = 0
        while self.is_running and self.cap:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None and len(ids) > 0:
                cid = int(ids[0][0])
                if cid == candidate_id:
                    candidate_count += 1
                else:
                    candidate_id = cid
                    candidate_count = 1

                if candidate_count >= 3:
                    with self.lock:
                        self.confirmed_aruco_id = candidate_id
            else:
                candidate_id = None
                candidate_count = 0

            time.sleep(0.02)

    # API Pública
    def get_tof_distances(self):
        with self.lock:
            return (self.dist_front_cm, self.dist_left_cm, self.dist_right_cm)

    def get_yaw_deg(self):
        with self.lock:
            return self.yaw_deg

    def reset_yaw(self, new_yaw=0.0):
        with self.lock:
            self.yaw_deg = float(new_yaw)

    def get_pitch_deg(self):
        with self.lock:
            return self.pitch_deg

    def is_on_ramp(self, threshold_deg=6.0):
        with self.lock:
            p = self.pitch_deg
        if p > threshold_deg:
            return 'UP'
        elif p < -threshold_deg:
            return 'DOWN'
        return 'FLAT'

    def set_motor_speeds(self, speed_left, speed_right):
        if -1.0 <= speed_left <= 1.0 and -1.0 <= speed_right <= 1.0:
            pwm_l = int(speed_left * 255)
            pwm_r = int(speed_right * 255)
        else:
            pwm_l = int(speed_left)
            pwm_r = int(speed_right)

        with self.lock:
            self.target_left_pwm = max(-255, min(255, pwm_l))
            self.target_right_pwm = max(-255, min(255, pwm_r))

    def set_servo_angle(self, angle_deg):
        with self.lock:
            self.target_servo_angle = int(max(0, min(180, angle_deg)))

    def get_aruco_id(self):
        with self.lock:
            return self.confirmed_aruco_id

    def clear_aruco_id(self):
        with self.lock:
            self.confirmed_aruco_id = None

    def stop_all(self):
        with self.lock:
            self.target_left_pwm = 0
            self.target_right_pwm = 0
        if self.serial_port:
            try:
                self.serial_port.write(f"M,0,0,{self.target_servo_angle}\n".encode('ascii'))
            except Exception:
                pass

    def close(self):
        self.is_running = False
        self.stop_all()
        time.sleep(0.05)
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
        if self.tof_front:
            try:
                self.tof_front.stop_ranging()
                self.tof_front.close()
            except Exception:
                pass
        if self.tof_left:
            try:
                self.tof_left.stop_ranging()
            except Exception:
                pass
        if self.tof_right:
            try:
                self.tof_right.stop_ranging()
            except Exception:
                pass
        print("[HAL] Recursos cerrados correctamente.")
