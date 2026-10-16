#!/usr/bin/env python3
"""
Test de Transmisión UART hacia ESP32-C3-Zero con datos del sensor MPU-6050.
Puerto: /dev/ttyS5 (Pin 11 Orange Pi - UART5_TX) a 115200 baud.
"""

import time
import sys
import math
import serial

UART_DEVICE = "/dev/ttyS5"
UART_BAUDRATE = 115200
I2C_BUS = 2
MPU_ADDR = 0x68

# Intentar inicializar I2C para MPU-6050 real
has_real_imu = False
bus = None

try:
    import smbus2
    bus = smbus2.SMBus(I2C_BUS)
    # Despertar MPU-6050
    bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)
    time.sleep(0.02)
    # Filtro pasabajas DLPF
    bus.write_byte_data(MPU_ADDR, 0x1A, 0x03)
    has_real_imu = True
    print(f"[OK] Sensor MPU-6050 detectado y activo en /dev/i2c-{I2C_BUS} (0x{MPU_ADDR:02X})")
except Exception as e:
    print(f"[AVISO] MPU-6050 no detectado en I2C ({e}).")
    print("        -> Se usara generador de datos inerciales simulados para la prueba UART.")


def read_real_imu():
    """Lee Pitch y Yaw aproximados del MPU-6050 fisico."""
    def read_word(reg):
        h = bus.read_byte_data(MPU_ADDR, reg)
        l = bus.read_byte_data(MPU_ADDR, reg + 1)
        val = (h << 8) | l
        return val - 65536 if val >= 32768 else val

    # Acelerometro
    ax = read_word(0x3B) / 16384.0
    ay = read_word(0x3D) / 16384.0
    az = read_word(0x3F) / 16384.0
    # Giroscopo Z
    gz = read_word(0x47) / 131.0

    pitch = math.atan2(ay, math.sqrt(ax * ax + az * az)) * (180.0 / math.pi)
    roll = math.atan2(-ax, az) * (180.0 / math.pi)
    return pitch, roll, gz


def main():
    print("=" * 65)
    print("  TEST UART: ORANGE PI ZERO 2W -> ESP32-C3-ZERO (MPU-6050)")
    print("=" * 65)
    print(f"Puerto Serial:  {UART_DEVICE} @ {UART_BAUDRATE} baud (Pin 11 TX)")
    print("Destino ESP32:  GPIO 20 (RX) con GND comun")
    print("Presiona Ctrl+C para finalizar.\n")

    # Abrir puerto UART5
    try:
        ser = serial.Serial(UART_DEVICE, UART_BAUDRATE, timeout=1.0)
        time.sleep(0.1)
        ser.reset_output_buffer()
        print(f"[OK] Puerto serial {UART_DEVICE} abierto correctamente.\n")
    except Exception as e:
        print(f"[ERROR] No se pudo abrir {UART_DEVICE}: {e}")
        print("Sugerencia: Ejecuta con 'sudo' o revisa si /dev/ttyS5 existe.")
        sys.exit(1)

    paquete = 0
    yaw_acumulado = 0.0
    t_prev = time.time()

    try:
        while True:
            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now

            paquete += 1

            if has_real_imu:
                try:
                    pitch, roll, gz = read_real_imu()
                    yaw_acumulado += gz * dt
                    # Acotar yaw a -180..+180
                    yaw_acumulado = (yaw_acumulado + 180) % 360 - 180
                except Exception:
                    pitch, roll = 0.0, 0.0
            else:
                # Datos de prueba suaves para verificar el enlace
                pitch = 5.0 * math.sin(t_now * 2.0)
                roll = 2.0 * math.cos(t_now * 1.5)
                yaw_acumulado = (yaw_acumulado + 15.0 * dt) % 360

            # Formato de trama claro y conciso
            linea = f"IMU: Yaw={yaw_acumulado:+.1f} deg | Pitch={pitch:+.1f} deg | Pkt={paquete}\n"

            # Enviar por UART hacia el ESP32
            ser.write(linea.encode("ascii"))
            ser.flush()

            print(f"[TX -> ESP32] {linea.strip()}")
            time.sleep(0.25)  # Enviar 4 veces por segundo

    except KeyboardInterrupt:
        print("\n[FIN] Prueba finalizada por el usuario.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
