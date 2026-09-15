"""
Batería de Verificación Automatizada: Pista B (Niveles) con Aleatoriedad Procedural Completa
Candidates Principiantes 2026 - RoBorregos

Ejecuta múltiples pruebas consecutivas con configuraciones 100% aleatorias:
1. Sección 1: Posición exterior de inicio aleatoria (Sur, Norte u Oeste) y lado abierto de la trampa aleatorio (N/S/E/W).
2. Sección 2: Huecos de líneas blancas aleatorios en cada barrera (inferior, superior, centro).
3. Sección 3: Laberinto de colores procedural con Random Walk y baldosa verde FIN exterior dinámica.
"""

import os
import sys
import time
import json
import subprocess

WORKDIR = r"c:\Users\Sin nombre\Documents\Candidates\WebotsSim2Real"
TARGET_WORLD_FILE = os.path.join(WORKDIR, "target_world.txt")
CONFIG_FILE = os.path.join(WORKDIR, "track_b_config.json")
FLAG_FILE = os.path.join(WORKDIR, "track_b_verified.flag")
TEST_FLAG_FILE = os.path.join(WORKDIR, "test_track_b.flag")
S1_FLAG_FILE = os.path.join(WORKDIR, "track_b_s1_passed.flag")
LOG_FILE = os.path.join(WORKDIR, "god_mode_track_b.log")

def clean_processes():
    subprocess.run(["powershell", "-Command", "Stop-Process -Name webots* -Force -ErrorAction SilentlyContinue"], capture_output=True)

def run_single_random_trial(trial_num, seed=None):
    print("\n" + "=" * 70)
    print(f"  EJECUTANDO PRUEBA ALEATORIA #{trial_num} (Semilla: {seed})")
    print("=" * 70)

    # 1. Configurar track_b_config.json en modo aleatorio
    cfg = {
        "fixed": False,
        "s1_test": False,
        "test_mode": False
    }
    if seed is not None:
        cfg["seed"] = seed

    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)

    # 2. Configurar target_world.txt
    with open(TARGET_WORLD_FILE, "w", encoding="utf-8") as f:
        f.write("worlds/sim2real_track_b.wbt\n")

    # 3. Limpiar flags y logs
    for fp in [FLAG_FILE, TEST_FLAG_FILE, S1_FLAG_FILE, LOG_FILE]:
        if os.path.exists(fp):
            try: os.remove(fp)
            except Exception: pass

    clean_processes()
    time.sleep(1.0)

    # 4. Lanzar simulación
    print("Lanzando tarea interactiva 'RunWebotsSim'...")
    res = subprocess.run(["schtasks", "/run", "/tn", "RunWebotsSim"], capture_output=True, text=True)
    if res.returncode != 0:
        print(f"[ERROR] No se pudo invocar RunWebotsSim: {res.stderr}")
        return False, 0.0, {}

    # 5. Monitoreo de telemetría
    start_time = time.time()
    max_wait = 180.0
    last_pos = 0
    trial_info = {
        "start_cell": None,
        "open_side": None,
        "s2_gaps": [],
        "s3_path": None,
        "finish_cell": None,
        "s1_ok": False,
        "s2_ok": False,
        "s2_clean": False,
        "s3_ok": False,
    }

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
                            if not line_clean: continue
                            if "[S1 INICIO]" in line_clean:
                                trial_info["start_cell"] = line_clean.split(":")[-1].strip()
                                print(f"  >> {line_clean}")
                            elif "[S1 BALL TRAP]" in line_clean:
                                trial_info["open_side"] = line_clean.split("Lado abierto:")[-1].split("(")[0].strip()
                                print(f"  >> {line_clean}")
                            elif "[S2 WHITE LINE" in line_clean and "Frontera" in line_clean:
                                print(f"  >> {line_clean}")
                            elif "[S3 RANDOM WALK]" in line_clean:
                                trial_info["s3_path"] = line_clean.split("Camino generado")[-1].strip()
                                print(f"  >> {line_clean}")
                            elif "[S3] Meta FIN" in line_clean:
                                trial_info["finish_cell"] = line_clean.split(":")[-1].strip()
                                print(f"  >> {line_clean}")
                            elif "[S1 SUCCESS]" in line_clean:
                                trial_info["s1_ok"] = True
                                print(f"  [S1 PASSED] {line_clean}")
                            elif "[S2 TOUCH]" in line_clean:
                                trial_info["s2_touched"] = True
                                trial_info["s2_clean"] = False
                                print(f"  [S2 TOUCHED] {line_clean}")
                            elif "[S2 CLEAN]" in line_clean or "Paso limpio" in line_clean:
                                trial_info["s2_ok"] = True
                                if not trial_info.get("s2_touched", False):
                                    trial_info["s2_clean"] = True
                                print(f"  [S2 CLEAN] {line_clean}")
                            elif "[CP2 SUCCESS]" in line_clean:
                                trial_info["s2_ok"] = True
                                if not trial_info.get("s2_touched", False):
                                    trial_info["s2_clean"] = True
                            elif "[TRACK B SUCCESS]" in line_clean:
                                trial_info["s3_ok"] = True
                                print(f"  [S3 FINISH] {line_clean}")
            except Exception:
                pass

        if trial_info["s3_ok"] or os.path.exists(FLAG_FILE):
            break

        time.sleep(0.5)

    duration = time.time() - start_time
    clean_processes()

    success = trial_info["s1_ok"] and trial_info["s2_ok"] and trial_info.get("s2_clean", False) and trial_info["s3_ok"]
    return success, duration, trial_info

def main():
    print("==================================================================")
    print("  BATERÍA DE EVALUACIÓN PROCEDURAL ALEATORIA: PISTA B             ")
    print("  Candidates Principiantes 2026 - RoBorregos                      ")
    print("==================================================================")

    # 3 Semillas aleatorias distintas para cubrir diversidad topológica
    seeds = [104231, 582914, 917302]
    results = []

    for idx, seed in enumerate(seeds, 1):
        success, duration, info = run_single_random_trial(idx, seed)
        results.append((idx, seed, success, duration, info))

    print("\n" + "=" * 75)
    print("  RESUMEN DE LA BATERÍA DE PRUEBAS ALEATORIAS - PISTA B           ")
    print("=" * 75)
    print(f"{'#':<3} | {'Semilla':<8} | {'Inicio':<8} | {'Trampa':<7} | {'S2':<8} | {'S3 Meta':<10} | {'T (s)':<6} | {'Resultado':<10}")
    print("-" * 75)

    all_passed = True
    for idx, seed, success, duration, info in results:
        status = "ÉXITO (100%)" if success else "FALLO"
        if not success: all_passed = False
        s_cell = str(info.get("start_cell", "N/A"))
        t_side = str(info.get("open_side", "N/A"))
        s2_res = "CLEAN" if info.get("s2_clean") else "FAIL"
        f_cell = str(info.get("finish_cell", "N/A"))
        print(f"{idx:<3} | {seed:<8} | {s_cell:<8} | {t_side:<7} | {s2_res:<8} | {f_cell:<10} | {duration:<6.1f} | {status:<10}")

    print("=" * 75)
    if all_passed:
        print("  ¡TODAS LAS PRUEBAS ALEATORIAS FUERON SUPERADAS CON ÉXITO!      ")
        print("==================================================================")
        return 0
    else:
        print("  ALGUNAS PRUEBAS REQUIRIERON REVISIÓN.                           ")
        print("==================================================================")
        return 1

if __name__ == "__main__":
    sys.exit(main())
