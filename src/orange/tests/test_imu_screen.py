"""Test combinado: MPU-6050 y OLED SSD1306 (Yaw en Z y Cabeceo en Y sobre /dev/i2c-2)."""

import time
import sys
import math

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

try:
    from mpu6050 import mpu6050
    USE_NATIVE = True
except ImportError:
    USE_NATIVE = False

import smbus2

I2C_PORT = 2
OLED_ADDR = 0x3C
MPU_ADDR = 0x68
SCREEN_WIDTH = 128
SCREEN_HEIGHT = 64


class MPU6050Driver:
    """Controlador directo de registros MPU-6050 por smbus2."""
    def __init__(self, bus_num=2, addr=0x68):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr
        self.bus.write_byte_data(self.addr, 0x6B, 0x00)  # Despertar sensor
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
    print("=" * 65)
    print("   TEST COMBINADO: MPU-6050 + OLED SSD1306 (I2C-2)           ")
    print("=" * 65)

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
        imu = mpu6050(MPU_ADDR, bus=I2C_PORT) if USE_NATIVE else MPU6050Driver(I2C_PORT, MPU_ADDR)
        print(f"[OK] Sensor MPU-6050 conectado en 0x{MPU_ADDR:02X}")
    except Exception as e:
        print(f"[ERROR] No se pudo inicializar MPU-6050: {e}")
        sys.exit(1)

    # 3. Calibración en reposo con barra visual en pantalla
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

    # 4. Bucle en vivo de visualización
    print("\n>>> MODO EN VIVO ACTIVO (Ctrl + C para terminar) <<<\n")
    yaw_deg = 0.0
    init_acc = imu.get_accel_data()
    pitch_deg = math.atan2(init_acc['y'], math.sqrt(init_acc['x']**2 + init_acc['z']**2)) * (180.0 / math.pi)

    last_time = time.time()
    deadzone = 0.25

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            accel = imu.get_accel_data()
            gyro = imu.get_gyro_data()
            temp = imu.get_temp() if hasattr(imu, 'get_temp') else 0.0

            gz_corr = gyro['z'] - bias_z
            gx_corr = gyro['x'] - bias_x

            # Rumbo (Yaw sobre eje Z)
            if abs(gz_corr) > deadzone:
                yaw_deg = (yaw_deg + gz_corr * dt) % 360.0

            # Cabeceo real (Pitch sobre eje Y)
            pitch_acc = math.atan2(accel['y'], math.sqrt(accel['x']**2 + accel['z']**2)) * (180.0 / math.pi)
            pitch_deg = 0.96 * (pitch_deg + gx_corr * dt) + 0.04 * pitch_acc

            # Renderizado en pantalla OLED
            img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
            draw = ImageDraw.Draw(img)

            # Barra superior invertida con temperatura
            draw.rectangle((0, 0, SCREEN_WIDTH - 1, 11), fill=1)
            draw.text((4, 1), f"IMU EN VIVO | {temp:.0f}C", font=font, fill=0)

            # Fila 1: YAW (Rumbo)
            turn_lbl = "GIRO DER" if gz_corr > deadzone else ("GIRO IZQ" if gz_corr < -deadzone else "REPOSO")
            draw.text((2, 14), f"YAW    : {yaw_deg:5.1f}*", font=font, fill=1)
            draw.text((76, 14), f"[{turn_lbl}]", font=font, fill=1)

            # Fila 2: CABECEO (Pitch)
            pitch_badge = "[SUBIDA ^]" if pitch_deg > 6.0 else ("[BAJADA v]" if pitch_deg < -6.0 else "[PLANO  -]")
            draw.text((2, 26), f"CABECEO: {pitch_deg:+5.1f}*", font=font, fill=1)
            draw.text((70, 26), pitch_badge, font=font, fill=1)

            # Fila 3: Aceleraciones (m/s^2)
            draw.text((2, 38), f"Ax:{accel['x']:+4.1f} Ay:{accel['y']:+4.1f} Az:{accel['z']:+4.1f}", font=font, fill=1)

            # Fila 4: Indicador gráfico horizontal de giro (Yaw)
            draw.line((14, 55, 114, 55), fill=1)
            draw.line((64, 52, 64, 58), fill=1)  # Marca central
            bar_offset = int(max(-45, min(45, gz_corr * 1.5)))
            if bar_offset > 0:
                draw.rectangle((64, 53, 64 + bar_offset, 57), fill=1)
            elif bar_offset < 0:
                draw.rectangle((64 + bar_offset, 53, 64, 57), fill=1)

            device.display(img)

            # Salida compacta en consola
            sys.stdout.write(
                f"\rYaw: {yaw_deg:5.1f}° | Cabeceo: {pitch_deg:+5.1f}° {pitch_badge} | Ax:{accel['x']:+4.1f} Ay:{accel['y']:+4.1f} Az:{accel['z']:+4.1f}   "
            )
            sys.stdout.flush()
            time.sleep(0.03)

    except KeyboardInterrupt:
        print(f"\n\n[FIN] Yaw final: {yaw_deg:.1f}° | Cabeceo final: {pitch_deg:+.1f}°\n")
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)
        draw.rectangle((2, 2, SCREEN_WIDTH - 3, SCREEN_HEIGHT - 3), outline=1)
        draw.text((18, 20), "!TEST FINALIZADO!", font=font, fill=1)
        draw.text((16, 36), "IMU + OLED LISTOS", font=font, fill=1)
        device.display(img)
        time.sleep(1.0)


if __name__ == '__main__':
    main()
