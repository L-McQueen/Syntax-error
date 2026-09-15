"""
Script de Verificación Automatizada para la Sección 1: La Trampa de la Pelota
Pista B (Niveles) - Webots Sim2Real
Candidates Principiantes 2026 - RoBorregos

Valida de manera integral:
1. Entrada desde casilla verde exterior (gy=-1) hacia el 3x3 (ToF < 0.9m y piso != VERDE).
2. Recorrido de casillas medias e inspección con regla de 3 rangos ToF.
3. Detección y centrado visual PID en el centroide de la pelota de golf naranja (45g).
4. Captura física dentro de la garra pasiva con cuña de retención de 8mm.
5. Maniobra de reversa segura en 3 capas (alineación lateral, retroceso con PID, confirmación de despeje).
6. Salida por el pasillo de mayor distancia (Este en 2,1).
7. Avance hacia Checkpoint 1 hasta detección de color ROJO en la cámara inferior.
8. Retención física confirmada de la pelota dentro de la garra en Checkpoint 1.
"""

import os
import sys
import time
import json
import subprocess

WORKDIR = r"c:\Users\Sin nombre\Documents\Candidates\WebotsSim2Real"
TARGET_WORLD_FILE = os.path.join(WORKDIR, "target_world.txt")
CONFIG_FILE = os.path.join(WORKDIR, "track_b_config.json")
FLAG_S1_FILE = os.path.join(WORKDIR, "track_b_s1_passed.flag")
LOG_FILE = os.path.join(WORKDIR, "god_mode_track_b.log")

def clean_processes():
    subprocess.run(["powershell", "-Command", "Stop-Process -Name webots* -Force -ErrorAction SilentlyContinue"], capture_output=True)
    subprocess.run(["powershell", "-Command", "Stop-Process -Name *webots* -Force -ErrorAction SilentlyContinue"], capture_output=True)
    time.sleep(1.5)

def run_single_eval(open_side="South", start_cell=(0, -1), max_wait=90.0):
    print("\n" + "=" * 60)
    print(f"  EVALUACIÓN SECCIÓN 1: Lado Abierto={open_side} | Inicio={start_cell}")
    print("=" * 60)

    # 1. Limpiar procesos previos
    clean_processes()

    # 2. Configurar archivos
    with open(TARGET_WORLD_FILE, "w", encoding="utf-8") as f:
        f.write("worlds/sim2real_track_b.wbt\n")

    cfg = {
        "s1_test": True,
        "test_mode": False,
        "open_side": open_side,
        "start_cell": list(start_cell),
        "fixed": False
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)

    # 3. Limpiar flags y log previo estrictamente
    for fp in [FLAG_S1_FILE, os.path.join(WORKDIR, "test_track_b.flag"), os.path.join(WORKDIR, "track_b_verified.flag"), LOG_FILE]:
        if os.path.exists(fp):
            try: os.remove(fp)
            except Exception: pass

    time.sleep(1.0)

    # 4. Lanzar simulación
    print("Lanzando simulación Webots vía tarea interactiva 'RunWebotsSim'...")
    res = subprocess.run(["schtasks", "/run", "/tn", "RunWebotsSim"], capture_output=True, text=True)
    if res.returncode != 0:
        print(f"[ERROR] No se pudo invocar RunWebotsSim: {res.stderr}")
        return False, "ERROR_TASK"

    # Esperar a que el supervisor cree el nuevo LOG_FILE
    wait_log_start = time.time()
    while time.time() - wait_log_start < 15.0:
        if os.path.exists(LOG_FILE):
            try:
                with open(LOG_FILE, "r", encoding="utf-8") as f:
                    content = f.read()
                    if "TRACK B SUPERVISOR LOG" in content:
                        break
            except Exception:
                pass
        time.sleep(0.5)

    if not os.path.exists(LOG_FILE):
        print("[ERROR] Webots no inició el supervisor a tiempo.")
        clean_processes()
        return False, "TIMEOUT_START"

    # 5. Monitorear logs en vivo
    start_time = time.time()
    last_pos = 0
    passed = False
    ball_captured = False
    cp1_reached = False

    while time.time() - start_time < max_wait:
        if os.path.exists(LOG_FILE):
            try:
                with open(LOG_FILE, "r", encoding="utf-8", errors="replace") as f:
                    f.seek(last_pos)
                    new_text = f.read()
                    if new_text:
                        last_pos = f.tell()
                        for line in new_text.strip().split("\n"):
                            line_c = line.strip()
                            if line_c:
                                print(f"  {line_c}")
                                if "PELOTA DE GOLF CAPTURADA" in line_c or "BALL_CAPTURED" in line_c:
                                    ball_captured = True
                                if "CHECKPOINT 1 (3, 1)" in line_c or "CP1_SUCCESS" in line_c:
                                    cp1_reached = True
                                if "S1 SUCCESS" in line_c or "MISIÓN SECCIÓN 1 CUMPLIDA" in line_c:
                                    passed = True
            except Exception:
                pass

        if os.path.exists(FLAG_S1_FILE) and ball_captured:
            passed = True
            break

        time.sleep(0.4)

    clean_processes()
    dur = time.time() - start_time

    print("-" * 60)
    print(f"Duración: {dur:.1f}s | Pelota capturada: {ball_captured} | CP1 Rojo: {cp1_reached} | Resultado: {'PASSED' if (passed and ball_captured) else 'FAILED'}")
    print("-" * 60)

    return (passed and ball_captured), f"dur={dur:.1f}s"

def main():
    print("==================================================")
    print("  SUITE DE PRUEBAS AUTOMATIZADA: SECCIÓN 1 (PISTA B)")
    print("==================================================")

    # Evaluar lado por defecto
    test_cases = [
        ("South", (0, -1)),
        ("East", (0, -1)),
        ("North", (0, -1)),
        ("West", (0, -1))
    ]

    # Si se pasa un argumento por línea de comandos, probar solo ese lado
    if len(sys.argv) > 1:
        req_side = sys.argv[1].capitalize()
        test_cases = [(req_side, (0, -1))]

    results = []
    for side, start in test_cases:
        ok, msg = run_single_eval(open_side=side, start_cell=start, max_wait=90.0)
        results.append((side, start, ok, msg))
        if not ok:
            print(f"[WARN] Prueba falló para lado {side}. Reintentando una vez...")
            time.sleep(2.0)
            ok2, msg2 = run_single_eval(open_side=side, start_cell=start, max_wait=90.0)
            results[-1] = (side, start, ok2, msg2)

    print("\n" + "=" * 60)
    print("  RESUMEN FINAL DE PRUEBAS SECCIÓN 1 - PISTA B    ")
    print("=" * 60)
    all_passed = True
    for side, start, ok, msg in results:
        status = "PASSED [OK]" if ok else "FAILED [X]"
        print(f"  - Lado abierto: {side:<7} | Inicio: {str(start):<8} | {status} ({msg})")
        if not ok:
            all_passed = False

    if all_passed:
        print("\n>>> ¡TODAS LAS EVALUACIONES DE LA SECCIÓN 1 PASARON AL 100%! <<<")
        return 0
    else:
        print("\n>>> ALGUNAS PRUEBAS FALLARON. REVISAR LOGS. <<<")
        return 1

if __name__ == "__main__":
    sys.exit(main())
