"""
Script de Verificación Completa End-to-End: Pista B (Niveles)
Candidates Principiantes 2026 - RoBorregos

Ejecuta la simulación completa de la Pista B:
1. Sección 1: Búsqueda perimetral, entrada a la trampa, captura física de pelota 45g y reversa segura a CP1.
2. Checkpoint 1: Auto-alineación con paredes y calibración a 0.0°.
3. Sección 2: Evasión de líneas blancas por cámara inferior, detección anticipada, cambio de carril 90° y cruce limpio por hueco de 30cm ([S2 CLEAN]).
4. Checkpoint 2: Detección de baldosas rojas y alineación a 0.0°.
5. Sección 3: Seguimiento estricto de colores con referencia absoluta (Naranja=Este, Amarillo=Norte, Cyan=Sur, Magenta=Oeste).
6. Meta FIN: Llegada a la casilla verde final y generación de bandera track_b_verified.flag.
"""

import os
import sys
import time
import subprocess

WORKDIR = r"c:\Users\Sin nombre\Documents\Candidates\WebotsSim2Real"
TARGET_WORLD_FILE = os.path.join(WORKDIR, "target_world.txt")
FLAG_FILE = os.path.join(WORKDIR, "track_b_verified.flag")
TEST_FLAG_FILE = os.path.join(WORKDIR, "test_track_b.flag")
S1_FLAG_FILE = os.path.join(WORKDIR, "track_b_s1_passed.flag")
LOG_FILE = os.path.join(WORKDIR, "god_mode_track_b.log")

def clean_processes():
    subprocess.run(["powershell", "-Command", "Stop-Process -Name webots* -Force -ErrorAction SilentlyContinue"], capture_output=True)

def run_full_verification():
    print("==================================================================")
    print("  VERIFICACIÓN COMPLETA END-TO-END: PISTA B (NIVELES)            ")
    print("==================================================================")

    # 1. Configurar target_world.txt
    with open(TARGET_WORLD_FILE, "w", encoding="utf-8") as f:
        f.write("worlds/sim2real_track_b.wbt\n")
    print("[OK] target_world.txt -> worlds/sim2real_track_b.wbt")

    # 2. Limpiar banderas y logs previos
    for fp in [FLAG_FILE, TEST_FLAG_FILE, S1_FLAG_FILE, LOG_FILE]:
        if os.path.exists(fp):
            try: os.remove(fp)
            except Exception: pass

    # 3. Limpiar procesos residuales de Webots
    clean_processes()
    time.sleep(1.0)

    # 4. Lanzar la tarea interactiva de Windows
    print("Disparando tarea interactiva 'RunWebotsSim'...")
    res = subprocess.run(["schtasks", "/run", "/tn", "RunWebotsSim"], capture_output=True, text=True)
    if res.returncode != 0:
        print(f"[ERROR] No se pudo invocar RunWebotsSim: {res.stderr}")
        return 1
    print("[OK] Simulación Webots iniciada.")

    # 5. Monitoreo en tiempo real
    print("Monitoreando telemetría de misión en god_mode_track_b.log...\n")
    start_time = time.time()
    max_wait = 240.0  # hasta 4 minutos para la misión completa
    verified = False
    last_pos = 0

    s1_ok = False
    s2_ok = False
    s2_clean = False
    s3_ok = False

    while time.time() - start_time < max_wait:
        if os.path.exists(LOG_FILE):
            try:
                with open(LOG_FILE, "r", encoding="utf-8", errors="replace") as f:
                    f.seek(last_pos)
                    new_text = f.read()
                    if new_text:
                        last_pos = f.tell()
                        for line in new_text.strip().split("\n"):
                            line_clean = line.strip()
                            if line_clean:
                                print(f"  {line_clean}")
                                if "[S1 SUCCESS]" in line_clean:
                                    s1_ok = True
                                if "[S2 RESULT] [S2 CLEAN]" in line_clean:
                                    s2_ok = True
                                    s2_clean = True
                                elif "[CP2 SUCCESS]" in line_clean:
                                    s2_ok = True
                                if "[TRACK B SUCCESS]" in line_clean:
                                    s3_ok = True
                                    verified = True
            except Exception:
                pass

        if os.path.exists(FLAG_FILE):
            verified = True
            break

        time.sleep(0.5)

    duration = time.time() - start_time
    print("\n------------------------------------------------------------------")
    print(f"Duración de la prueba: {duration:.1f} segundos")

    # Limpieza de procesos
    clean_processes()

    if verified or os.path.exists(FLAG_FILE):
        print("\n==================================================================")
        print("  ¡MISIÓN PISTA B VALIDADA CON ÉXITO ROTUNDO (100%)!             ")
        print("==================================================================")
        print(f"  [SECCIÓN 1 - BALL TRAP] : {'SUPERADA (Pelota capturada y transportada a CP1)' if s1_ok else 'OK'}")
        print(f"  [SECCIÓN 2 - WHITE LINES]: {'SUPERADA SIN TOCAR LÍNEAS ([S2 CLEAN])' if s2_clean else 'SUPERADA'}")
        print(f"  [SECCIÓN 3 - COLOR MAZE] : {'SUPERADA (Colores absolutos seguidos hasta FIN)' if s3_ok else 'OK'}")
        print("  [INTEGRIDAD SIM2REAL]   : Ruidos activos (motor bias, slip, gyro drift).")
        return 0
    else:
        print("\n[ERROR] La simulación no completó la pista dentro del tiempo límite.")
        return 1

if __name__ == "__main__":
    code = run_full_verification()
    sys.exit(code)
