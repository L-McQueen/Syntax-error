"""Test combinado de sensores en vivo con pantalla OLED SSD1306 (/dev/i2c-2).
Monitorea en tiempo real: IMU MPU-6050 (Yaw y Cabeceo) y Sensor ToF VL53L0X (Mediana N=5 + EMA adaptativo)."""

import math
import sys
import time
from collections import deque

from PIL import Image, ImageDraw, ImageFont
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from mpu6050 import mpu6050
import smbus2
import VL53L0X

# Configuración del bus I2C y periféricos
I2C_PORT = 2
OLED_ADDR = 0x3C
MPU_ADDR = 0x68
TOF_ADDR = 0x29

SCREEN_WIDTH = 128
SCREEN_HEIGHT = 64

# Parámetros del pipeline de filtrado ToF (Modo BETTER a 15 Hz)
WINDOW_SIZE = 5
history_mediana = deque(maxlen=WINDOW_SIZE)
DEADBAND_CM = 0.4          # 4 mm de zona muerta para ruido térmico
ALPHA_SLOW = 0.25          # Suavizado para micro-variaciones
ALPHA_FAST = 0.90          # Respuesta rápida ante obstáculos o giros
STEP_THRESHOLD_CM = 1.5    # Umbral de conmutación rápida


class MPU6050Direct:
    """Controlador directo de registros MPU-6050 por smbus2 con filtro DLPF."""
    def __init__(self, bus_num=2, addr=0x68):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr
        self.bus.write_byte_data(self.addr, 0x6B, 0x00)  # Despertar chip
        time.sleep(0.02)
        self.bus.write_byte_data(self.addr, 0x1A, 0x03)  # DLPF ~44 Hz
        self.bus.write_byte_data(self.addr, 0x1B, 0x00)  # Gyro +/- 250 deg/s
        self.bus.write_byte_data(self.addr, 0x1C, 0x00)  # Accel +/- 2g

    def _read_word(self, reg):
        h = self.bus.read_byte_data(self.addr, reg)
        l = self.bus.read_byte_data(self.addr, reg + 1)
        val = (h << 8) | l
        return val - 65536 if val >= 32768 else val

    def get_accel_data(self):
        return {
            'x': (self._read_word(0x3B) / 16384.0) * 9.80665,
            'y': (self._read_word(0x3D) / 16384.0) * 9.80665,
            'z': (self._read_word(0x3F) / 16384.0) * 9.80665,
        }

    def get_gyro_data(self):
        return {
            'x': self._read_word(0x43) / 131.0,
            'y': self._read_word(0x45) / 131.0,
            'z': self._read_word(0x47) / 131.0,
        }

    def get_temp(self):
        return (self._read_word(0x41) / 340.0) + 36.53


def main():
    print("=" * 68)
    print("  SUITE DE SENSORES: OLED SSD1306 + MPU-6050 + ToF VL53L0X (I2C-2)")
    print("=" * 68)

    # 1. Conectar pantalla OLED SSD1306 (0x3C)
    try:
        serial_i2c = i2c(port=I2C_PORT, address=OLED_ADDR)
        device = ssd1306(serial_i2c, width=SCREEN_WIDTH, height=SCREEN_HEIGHT)
        font = ImageFont.load_default()
        print(f"[OK] Pantalla OLED conectada en 0x{OLED_ADDR:02X}")
    except Exception as e:
        print(f"[ERROR] No se pudo inicializar OLED: {e}")
        sys.exit(1)

    # 2. Conectar sensor inercial MPU-6050 (0x68)
    try:
        try:
            imu = mpu6050(MPU_ADDR, bus=I2C_PORT)
        except Exception:
            imu = MPU6050Direct(I2C_PORT, MPU_ADDR)
        print(f"[OK] Sensor MPU-6050 conectado en 0x{MPU_ADDR:02X}")
    except Exception as e:
        print(f"[ERROR] No se pudo inicializar MPU-6050: {e}")
        sys.exit(1)

    # 3. Conectar sensor ToF VL53L0X (0x29) en modo BETTER (15 Hz)
    tof = None
    try:
        tof = VL53L0X.VL53L0X(i2c_bus=I2C_PORT, i2c_address=TOF_ADDR)
        tof.open()
        tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
        print(f"[OK] Sensor ToF VL53L0X conectado en 0x{TOF_ADDR:02X} (Modo BETTER ~15 Hz)")
    except Exception as e:
        print(f"[WARN] ToF VL53L0X no detectado en 0x{TOF_ADDR:02X}: {e}")

    # 4. Calibración en reposo del giróscopo con animación visual en OLED
    print("\n>>> Calibrando giróscopo (mantén el robot inmóvil)...")
    samples = 120
    gz_sum = 0.0
    gx_sum = 0.0

    for i in range(samples):
        g = imu.get_gyro_data()
        gz_sum += g['z']
        gx_sum += g['x']

        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1), outline=1)
        draw.text((16, 10), "CALIBRANDO IMU...", font=font, fill=1)
        draw.text((14, 24), "!MANTEN INMOVIL!", font=font, fill=1)
        draw.rectangle((14, 40, 114, 48), outline=1)
        bar_w = int(96 * ((i + 1) / samples))
        if bar_w > 0:
            draw.rectangle((16, 42, 16 + bar_w, 46), fill=1)
        device.display(img)
        time.sleep(0.012)

    bias_z = gz_sum / samples
    bias_x = gx_sum / samples
    print(f"[OK] Calibración completa. Sesgo Z: {bias_z:+.3f}°/s | Sesgo X: {bias_x:+.3f}°/s")

    # 5. Variables de navegación y filtrado en vivo
    yaw_deg = 0.0
    init_acc = imu.get_accel_data()
    pitch_deg = math.atan2(init_acc['y'], math.sqrt(init_acc['x']**2 + init_acc['z']**2)) * (180.0 / math.pi)

    dist_filtrada_cm = None
    dist_raw_cm = 0.0

    last_time = time.time()
    deadzone_gyro = 0.25

    print("\n>>> DIAGNÓSTICO EN VIVO ACTIVO (Presiona Ctrl + C para terminar) <<<\n")

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            # --- A. Procesamiento IMU (Yaw + Cabeceo) ---
            accel = imu.get_accel_data()
            gyro = imu.get_gyro_data()

            gz_corr = gyro['z'] - bias_z
            gx_corr = gyro['x'] - bias_x

            # Integración de rumbo (Yaw en Z) con zona muerta
            if abs(gz_corr) > deadzone_gyro:
                yaw_deg = (yaw_deg + gz_corr * dt) % 360.0

            # Filtro complementario de cabeceo (Pitch en Y)
            pitch_acc = math.atan2(accel['y'], math.sqrt(accel['x']**2 + accel['z']**2)) * (180.0 / math.pi)
            pitch_deg = 0.96 * (pitch_deg + gx_corr * dt) + 0.04 * pitch_acc

            # --- B. Procesamiento ToF (Mediana N=5 + Banda Muerta + EMA Adaptativo) ---
            tof_valido = False
            if tof is not None:
                try:
                    d_mm = tof.get_distance()
                    if 0 < d_mm < 8000:
                        dist_raw_cm = d_mm / 10.0
                        history_mediana.append(dist_raw_cm)
                        sorted_samples = sorted(history_mediana)
                        dist_mediana = sorted_samples[len(sorted_samples) // 2]

                        if dist_filtrada_cm is None:
                            dist_filtrada_cm = dist_mediana
                        else:
                            delta = abs(dist_mediana - dist_filtrada_cm)
                            if delta > DEADBAND_CM:
                                alpha = ALPHA_FAST if delta > STEP_THRESHOLD_CM else ALPHA_SLOW
                                dist_filtrada_cm = (alpha * dist_mediana) + ((1.0 - alpha) * dist_filtrada_cm)
                        tof_valido = True
                except Exception:
                    pass

            # --- C. Renderizado gráfico en pantalla OLED SSD1306 (128x64) ---
            img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
            draw = ImageDraw.Draw(img)

            # Barra superior invertida con título
            draw.rectangle((0, 0, SCREEN_WIDTH - 1, 11), fill=1)
            draw.text((4, 1), "SENSORES EN VIVO | I2C-2", font=font, fill=0)

            # Fila 1: YAW (Rumbo)
            turn_lbl = "DER" if gz_corr > deadzone_gyro else ("IZQ" if gz_corr < -deadzone_gyro else "FIJO")
            draw.text((2, 14), f"YAW : {yaw_deg:5.1f}*", font=font, fill=1)
            draw.text((82, 14), f"[{turn_lbl}]", font=font, fill=1)

            # Fila 2: CABECEO (Pitch)
            pitch_lbl = "^SUBE" if pitch_deg > 6.0 else ("vBAJA" if pitch_deg < -6.0 else "=PLANO")
            draw.text((2, 26), f"PIT : {pitch_deg:+5.1f}*", font=font, fill=1)
            draw.text((82, 26), f"[{pitch_lbl}]", font=font, fill=1)

            # Fila 3: SENSOR DE DISTANCIA ToF
            if tof_valido and dist_filtrada_cm is not None:
                draw.text((2, 38), f"ToF : {dist_filtrada_cm:5.1f} cm", font=font, fill=1)
                draw.text((82, 38), f"{int(dist_filtrada_cm * 10):4d}mm", font=font, fill=1)
            else:
                draw.text((2, 38), "ToF : ---.- cm", font=font, fill=1)
                draw.text((82, 38), "[SIN ECO]", font=font, fill=1)

            # Fila 4: Barra gráfica dinámica de proximidad ToF (Escala de 0 a 50 cm)
            draw.rectangle((2, 52, 125, 60), outline=1)
            if tof_valido and dist_filtrada_cm is not None:
                # La barra se llena cuando el obstáculo está cerca (0 a 50 cm)
                bar_fill = int(max(0, min(121, (1.0 - (dist_filtrada_cm / 50.0)) * 121)))
                if bar_fill > 0:
                    draw.rectangle((4, 54, 4 + bar_fill, 58), fill=1)

            device.display(img)

            # Salida en consola de terminal
            tof_str = f"{dist_filtrada_cm:5.1f} cm" if (tof_valido and dist_filtrada_cm is not None) else "---.- cm"
            sys.stdout.write(
                f"\rYaw: {yaw_deg:5.1f}° [{turn_lbl}] | Pitch: {pitch_deg:+5.1f}° [{pitch_lbl}] | ToF: {tof_str}   "
            )
            sys.stdout.flush()
            time.sleep(0.04)

    except KeyboardInterrupt:
        print(f"\n\n[FIN] Diagnóstico terminado por el operador.")
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)
        draw.rectangle((2, 2, SCREEN_WIDTH - 3, SCREEN_HEIGHT - 3), outline=1)
        draw.text((20, 20), "!TEST FINALIZADO!", font=font, fill=1)
        draw.text((18, 36), "SENSORES APAGADOS", font=font, fill=1)
        device.display(img)
        time.sleep(0.8)

    finally:
        if tof is not None:
            try:
                tof.stop_ranging()
                tof.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
