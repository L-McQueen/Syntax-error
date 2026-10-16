"""Test unificado para la terna de sensores Time-of-Flight (VL53L1X Frontal y VL53L0X Laterales).
Opera en modo BETTER (15 Hz) con filtro idéntico en cascada (Mediana N=5 + Banda Muerta + EMA adaptativo).
Permite calibrar el rango del sensor frontal en vivo con teclas 1 (SHORT), 2 (MEDIUM) y 3 (LONG)."""

import os
import select
import sys
import termios
import time
import tty
from collections import deque

import gpiod
import VL53L0X
import vl53l1x

# Buses y direcciones I2C
I2C_BUS = 2
ADDR_FRONT = 0x29  # VL53L1X nativo
ADDR_LEFT = 0x30   # VL53L0X reasignado
ADDR_RIGHT = 0x31  # VL53L0X reasignado

# Pines físicos XSHUT en gpiochip0
GPIO_CHIP = "/dev/gpiochip0"
PIN_XSHUT_FRONT = 228  # Pin 16 (PH4) - Cable Gris
PIN_XSHUT_LEFT = 256   # Pin 29 (PI0) - Cable Verde
PIN_XSHUT_RIGHT = 271  # Pin 31 (PI15) - Cable Azul

# Modos de rango del sensor frontal VL53L1X
FRONT_MODES = {
    '1': ('SHORT', 1),   # Rango hasta 1.3 m (óptimo para casillas de 30 cm en laberinto)
    '2': ('MEDIUM', 2),  # Rango hasta 3.0 m
    '3': ('LONG', 3),    # Rango hasta 4.0 m
}


class ToFFilter:
    """Cadena de filtrado digital en cascada para sensor ToF:
    1. Sanitización de rango
    2. Filtro de Mediana N=5 (destrucción de picos espurios)
    3. Banda Muerta de 4 mm (congelamiento de ruido térmico)
    4. EMA con alpha adaptativo (suave en micro-ajustes, instantáneo en saltos)"""

    def __init__(self, window_size=5, deadband_cm=0.4, alpha_slow=0.25, alpha_fast=0.90, step_cm=1.5):
        self.history = deque(maxlen=window_size)
        self.deadband_cm = deadband_cm
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.step_cm = step_cm
        self.filtered_cm = None

    def update(self, dist_mm):
        # Descartar lecturas de error o fuera de rango (VL53L0X/L1X entregan 8190 o <= 0)
        if dist_mm <= 0 or dist_mm >= 8000:
            return None

        raw_cm = dist_mm / 10.0

        # Etapa 1: Mediana móvil
        self.history.append(raw_cm)
        sorted_h = sorted(self.history)
        median_cm = sorted_h[len(sorted_h) // 2]

        # Etapa 2 y 3: Banda muerta + EMA adaptativo
        if self.filtered_cm is None:
            self.filtered_cm = median_cm
        else:
            delta = abs(median_cm - self.filtered_cm)
            if delta > self.deadband_cm:
                alpha = self.alpha_fast if delta > self.step_cm else self.alpha_slow
                self.filtered_cm = (alpha * median_cm) + ((1.0 - alpha) * self.filtered_cm)

        return self.filtered_cm

    def reset(self):
        self.history.clear()
        self.filtered_cm = None


def main():
    print("=" * 72)
    print("      SUITE UNIFICADA DE SENSORES ToF (FRONTAL + LATERALES)")
    print("      Bus: /dev/i2c-2 | Frecuencia: ~15 Hz (Timing Budget: 66 ms)")
    print("=" * 72)
    print("  FRONTAL: VL53L1X en 0x29 (XSHUT Pin 16)")
    print("  IZQ    : VL53L0X en 0x30 (XSHUT Pin 29)")
    print("  DER    : VL53L0X en 0x31 (XSHUT Pin 31)")
    print("=" * 72)
    print("TECLAS FRONTAL: [1] SHORT (1.3m) | [2] MEDIUM (3.0m) | [3] LONG (4.0m)")
    print("Presiona Ctrl+C para salir.\n")

    # 1. Configuración de líneas GPIO XSHUT
    lines = None
    if os.path.exists(GPIO_CHIP):
        try:
            chip = gpiod.Chip(GPIO_CHIP)
            lines = chip.get_lines([PIN_XSHUT_FRONT, PIN_XSHUT_LEFT, PIN_XSHUT_RIGHT])
            lines.request(consumer="test_tof", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0, 0, 0])
            time.sleep(0.05)
            print("[GPIO] Líneas XSHUT inicializadas en LOW (sensores apagados).")
        except Exception as e:
            print(f"[GPIO-INFO] Control XSHUT omitido ({e}). Continuando con buses abiertos...")

    # Instancias de filtros
    filter_front = ToFFilter()
    filter_left = ToFFilter()
    filter_right = ToFFilter()

    tof_left = None
    tof_right = None
    tof_front = None

    # 2. Inicialización ToF Lateral Izquierdo (XSHUT Pin 29 -> reasignar a 0x30)
    if lines:
        try:
            lines.set_values([0, 1, 0])
            time.sleep(0.03)
        except Exception:
            pass
    try:
        tof_left = VL53L0X.VL53L0X(i2c_bus=I2C_BUS, i2c_address=0x29)
        tof_left.open()
        tof_left.change_address(ADDR_LEFT)
        tof_left.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
        print(f"[OK] ToF Izquierdo VL53L0X activo en 0x{ADDR_LEFT:02X} (BETTER ~15 Hz)")
    except Exception as e:
        print(f"[WARN] ToF Izquierdo no detectado en 0x{ADDR_LEFT:02X}: {e}")
        tof_left = None

    # 3. Inicialización ToF Lateral Derecho (XSHUT Pin 31 -> reasignar a 0x31)
    if lines:
        try:
            lines.set_values([0, 1, 1])
            time.sleep(0.03)
        except Exception:
            pass
    try:
        tof_right = VL53L0X.VL53L0X(i2c_bus=I2C_BUS, i2c_address=0x29)
        tof_right.open()
        tof_right.change_address(ADDR_RIGHT)
        tof_right.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
        print(f"[OK] ToF Derecho VL53L0X activo en 0x{ADDR_RIGHT:02X} (BETTER ~15 Hz)")
    except Exception as e:
        print(f"[WARN] ToF Derecho no detectado en 0x{ADDR_RIGHT:02X}: {e}")
        tof_right = None

    # 4. Inicialización ToF Frontal VL53L1X (XSHUT Pin 16 -> 0x29 nativo)
    current_front_name = 'SHORT'
    current_front_num = 1
    if lines:
        try:
            lines.set_values([1, 1, 1])
            time.sleep(0.03)
        except Exception:
            pass
    try:
        tof_front = vl53l1x.VL53L1X(i2c_bus=I2C_BUS, i2c_address=ADDR_FRONT)
        tof_front.open()
        try:
            # Fijar timing budget a 66 ms (66000 us) con 70 ms inter-measurement para 15 Hz
            tof_front.set_timing(66000, 70)
        except Exception:
            pass
        tof_front.start_ranging(current_front_num)
        print(f"[OK] ToF Frontal VL53L1X activo en 0x{ADDR_FRONT:02X} (Modo {current_front_name} ~15 Hz)")
    except Exception as e:
        print(f"[WARN] ToF Frontal no detectado en 0x{ADDR_FRONT:02X}: {e}")
        tof_front = None

    # 5. Bucle de Telemetría en Vivo
    print("\n>>> TELEMETRÍA EN VIVO (Presiona 1, 2 o 3 para calibrar el frontal) <<<\n")

    # Configurar terminal para captura no bloqueante de teclas
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while True:
            # Lectura de teclado para conmutar rango del VL53L1X frontal al vuelo
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key in FRONT_MODES and tof_front is not None:
                    current_front_name, current_front_num = FRONT_MODES[key]
                    tof_front.stop_ranging()
                    try:
                        tof_front.set_timing(66000, 70)
                    except Exception:
                        pass
                    tof_front.start_ranging(current_front_num)
                    filter_front.reset()

            # --- Lectura Frontal (VL53L1X) ---
            if tof_front is not None:
                try:
                    df_mm = tof_front.get_distance()
                    val_f = filter_front.update(df_mm)
                    str_front = f"{val_f:5.1f} cm ({int(val_f*10):4d}mm)" if val_f is not None else "  ---.- cm (Eco) "
                except Exception:
                    str_front = "  ERROR cm        "
            else:
                str_front = "  [NO DETECTADO]  "

            # --- Lectura Izquierda (VL53L0X) ---
            if tof_left is not None:
                try:
                    dl_mm = tof_left.get_distance()
                    val_l = filter_left.update(dl_mm)
                    str_left = f"{val_l:5.1f} cm ({int(val_l*10):4d}mm)" if val_l is not None else "  ---.- cm (Eco) "
                except Exception:
                    str_left = "  ERROR cm        "
            else:
                str_left = "  [NO DETECTADO]  "

            # --- Lectura Derecha (VL53L0X) ---
            if tof_right is not None:
                try:
                    dr_mm = tof_right.get_distance()
                    val_r = filter_right.update(dr_mm)
                    str_right = f"{val_r:5.1f} cm ({int(val_r*10):4d}mm)" if val_r is not None else "  ---.- cm (Eco) "
                except Exception:
                    str_right = "  ERROR cm        "
            else:
                str_right = "  [NO DETECTADO]  "

            # Salida en una sola línea fija con retorno de carro
            print(
                f"\r[F:{current_front_name:<6}] Frontal: {str_front} | "
                f"Izq: {str_left} | "
                f"Der: {str_right}   ",
                end="",
                flush=True
            )

            time.sleep(0.04)

    except KeyboardInterrupt:
        print("\n\n[INFO] Prueba finalizada por el operador.")

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        if tof_left:
            try:
                tof_left.stop_ranging()
                tof_left.close()
            except Exception:
                pass
        if tof_right:
            try:
                tof_right.stop_ranging()
                tof_right.close()
            except Exception:
                pass
        if tof_front:
            try:
                tof_front.stop_ranging()
            except Exception:
                pass
        if lines:
            try:
                lines.release()
            except Exception:
                pass


if __name__ == '__main__':
    main()
