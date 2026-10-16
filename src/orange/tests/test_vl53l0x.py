"""Test interactivo para sensor ToF VL53L0X en modo BETTER con mediana (N=5) y EMA adaptativo."""

import select
import sys
import termios
import time
import tty
from collections import deque
import VL53L0X

# Mapeo de teclas numéricas para conmutación en vivo si se desea comparar
MODES = {
    '1': ('GOOD', VL53L0X.Vl53l0xAccuracyMode.GOOD),
    '2': ('BETTER', VL53L0X.Vl53l0xAccuracyMode.BETTER),
    '3': ('BEST', VL53L0X.Vl53l0xAccuracyMode.BEST),
    '4': ('LONG_RANGE', VL53L0X.Vl53l0xAccuracyMode.LONG_RANGE),
    '5': ('HIGH_SPEED', VL53L0X.Vl53l0xAccuracyMode.HIGH_SPEED),
}

current_mode_name = 'BETTER'
tof = VL53L0X.VL53L0X(i2c_bus=2, i2c_address=0x29)
tof.open()
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

print("=== MODO BETTER (15 Hz) | Mediana(N=5) + Banda Muerta 4mm + EMA Adaptativo ===")
print("=== TECLAS: 1:GOOD | 2:BETTER | 3:BEST | 4:LONG | 5:SPEED | Ctrl+C: Salir ===\n")

# Parámetros del pipeline de filtrado
WINDOW_SIZE = 5
history_mediana = deque(maxlen=WINDOW_SIZE)
DEADBAND_CM = 0.4          # 4 mm: descarta el ruido térmico residual del sensor
ALPHA_SLOW = 0.25          # Suavizado para micro-variaciones (seguimiento de pared)
ALPHA_FAST = 0.90          # Respuesta inmediata ante giros u obstáculos bruscos
STEP_THRESHOLD_CM = 1.5    # Umbral de salto para activar respuesta rápida

dist_filtrada_cm = None

# Configurar terminal en modo cbreak para lectura de teclas sin presionar Enter
fd = sys.stdin.fileno()
old_term = termios.tcgetattr(fd)

try:
    tty.setcbreak(fd)
    while True:
        # Detección de teclas para cambiar de modo al vuelo
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key in MODES:
                current_mode_name, mode_enum = MODES[key]
                tof.stop_ranging()
                tof.start_ranging(mode_enum)
                history_mediana.clear()
                dist_filtrada_cm = None

        dist_mm = tof.get_distance()

        # 1. Sanitización de lectura (valores <= 0 o >= 8000 mm son inválidos / fuera de rango)
        if 0 < dist_mm < 8000:
            dist_raw_cm = dist_mm / 10.0

            # 2. Filtro de Mediana (Ventana deslizante N=5 para matar picos espurios)
            history_mediana.append(dist_raw_cm)
            sorted_samples = sorted(history_mediana)
            dist_mediana_cm = sorted_samples[len(sorted_samples) // 2]

            # 3 y 4. Banda Muerta + EMA Adaptativo
            if dist_filtrada_cm is None:
                dist_filtrada_cm = dist_mediana_cm
            else:
                delta = abs(dist_mediana_cm - dist_filtrada_cm)
                if delta > DEADBAND_CM:
                    alpha = ALPHA_FAST if delta > STEP_THRESHOLD_CM else ALPHA_SLOW
                    dist_filtrada_cm = (alpha * dist_mediana_cm) + ((1.0 - alpha) * dist_filtrada_cm)

            n_samples = len(history_mediana)
            print(
                f"\r[{current_mode_name:<10}] Crudo:{dist_raw_cm:5.1f} cm | "
                f"Mediana({n_samples}):{dist_mediana_cm:5.1f} cm | "
                f"Filtrado:{dist_filtrada_cm:5.1f} cm ({int(dist_filtrada_cm * 10):4d} mm)   ",
                end="",
                flush=True
            )
        else:
            print(f"\r[{current_mode_name:<10}] Crudo:  ----  cm | Fuera de rango o sin eco...                  ", end="", flush=True)

        time.sleep(0.04)

except KeyboardInterrupt:
    print("\n\n[INFO] Prueba finalizada por el usuario.")
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
    try:
        tof.stop_ranging()
        tof.close()
    except Exception:
        pass
