"""Test de sensor inercial MPU-6050 (Yaw en Z y Cabeceo en Y sobre /dev/i2c-2)."""

import time
import sys
import math

try:
    from mpu6050 import mpu6050
    USE_NATIVE = True
except ImportError:
    USE_NATIVE = False

import smbus2

I2C_BUS = 2
MPU_ADDR = 0x68


class MPU6050Driver:
    """Controlador directo de registros MPU-6050 por smbus2."""
    def __init__(self, bus_num=2, addr=0x68):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr
        # Despertar el sensor (PWR_MGMT_1 = 0)
        self.bus.write_byte_data(self.addr, 0x6B, 0x00)
        time.sleep(0.02)
        # Filtro pasabajas interno DLPF a ~44 Hz (CONFIG = 3)
        self.bus.write_byte_data(self.addr, 0x1A, 0x03)
        # Giróscopo +/- 250 deg/s (GYRO_CONFIG = 0)
        self.bus.write_byte_data(self.addr, 0x1B, 0x00)
        # Acelerómetro +/- 2g (ACCEL_CONFIG = 0)
        self.bus.write_byte_data(self.addr, 0x1C, 0x00)

    def _read_word(self, reg):
        h = self.bus.read_byte_data(self.addr, reg)
        l = self.bus.read_byte_data(self.addr, reg + 1)
        val = (h << 8) | l
        return val - 65536 if val >= 32768 else val

    def get_accel_data(self):
        scale = 16384.0 * 9.80665  # m/s^2
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
    print("      TEST INDIVIDUAL: MPU-6050 (YAW Y CABECEO REAL)          ")
    print("=" * 65)

    # 1. Inicialización del sensor
    try:
        sensor = mpu6050(MPU_ADDR, bus=I2C_BUS) if USE_NATIVE else MPU6050Driver(I2C_BUS, MPU_ADDR)
        print(f"[OK] Sensor MPU-6050 conectado en /dev/i2c-{I2C_BUS} (0x{MPU_ADDR:02X})")
    except Exception as e:
        print(f"[ERROR] No se pudo inicializar MPU-6050: {e}")
        sys.exit(1)

    # 2. Calibración en reposo de Gz (Rumbo) y Gx (Cabeceo)
    print("\n>>> CALIBRANDO GIRÓSCOPO (MANTÉN EL ROBOT INMÓVIL) <<<")
    samples = 150
    gz_sum = 0.0
    gx_sum = 0.0
    for _ in range(samples):
        g = sensor.get_gyro_data()
        gz_sum += g['z']
        gx_sum += g['x']
        time.sleep(0.01)

    bias_z = gz_sum / samples
    bias_x = gx_sum / samples
    print(f"[LISTO] Sesgo Z (Yaw): {bias_z:+.4f}°/s | Sesgo X (Cabeceo): {bias_x:+.4f}°/s")

    # 3. Bucle en tiempo real
    print("\n" + "=" * 65)
    print(">>> MODO EN VIVO: GIRA O INCLINA EL ROBOT (Ctrl + C para salir) <<<")
    print("=" * 65 + "\n")

    yaw = 0.0
    init_acc = sensor.get_accel_data()
    pitch = math.atan2(init_acc['y'], math.sqrt(init_acc['x']**2 + init_acc['z']**2)) * (180.0 / math.pi)

    last_time = time.time()
    deadzone = 0.25

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            g = sensor.get_gyro_data()
            a = sensor.get_accel_data()

            gz_corr = g['z'] - bias_z
            gx_corr = g['x'] - bias_x

            # Rumbo (Yaw en eje Z)
            if abs(gz_corr) > deadzone:
                yaw = (yaw + gz_corr * dt) % 360.0

            # Cabeceo real (Eje Y: Inclinación nariz arriba/abajo)
            pitch_acc = math.atan2(a['y'], math.sqrt(a['x']**2 + a['z']**2)) * (180.0 / math.pi)
            pitch = 0.96 * (pitch + gx_corr * dt) + 0.04 * pitch_acc

            # Estado
            if pitch > 6.0:
                badge = "SUBIDA ^"
            elif pitch < -6.0:
                badge = "BAJADA v"
            else:
                badge = "PLANO  -"

            sys.stdout.write(
                f"\rYaw: {yaw:5.1f}° | Cabeceo: {pitch:+5.1f}° [{badge}] | Ax:{a['x']:+4.1f} Ay:{a['y']:+4.1f} Az:{a['z']:+4.1f}  "
            )
            sys.stdout.flush()
            time.sleep(0.02)

    except KeyboardInterrupt:
        print(f"\n\n[FIN] Yaw final: {yaw:.1f}° | Cabeceo final: {pitch:+.1f}°\n")


if __name__ == '__main__':
    main()
