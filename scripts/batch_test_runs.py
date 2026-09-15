import os
import sys
import time
import subprocess
import re

WORKDIR = r"c:\Users\Sin nombre\Documents\Candidates\WebotsSim2Real"
LOG_FILE = os.path.join(WORKDIR, "god_mode_live.log")
FLAG_FILE = os.path.join(WORKDIR, "finished.flag")

def clean_processes():
    try:
        subprocess.run(["schtasks", "/end", "/tn", "RunWebotsSim"], capture_output=True)
        subprocess.run(["powershell", "-Command", "Stop-Process -Name webots* -Force -ErrorAction SilentlyContinue"], capture_output=True)
    except Exception:
        pass

def run_single(run_idx):
    clean_processes()
    time.sleep(2.5)
    
    if os.path.exists(FLAG_FILE):
        try: os.remove(FLAG_FILE)
        except Exception: pass
        
    with open(LOG_FILE, "w", encoding="utf-8") as f:
        f.write("")
        
    start_time = time.time()
    print(f"\n[Run {run_idx}] Launching simulation...", flush=True)
    subprocess.run(["schtasks", "/run", "/tn", "RunWebotsSim"], capture_output=True)
    
    time.sleep(4.0)
    max_duration = 140.0
    completed = False
    
    webots_seen = False
    zero_count = 0
    while time.time() - start_time < max_duration:
        if os.path.exists(FLAG_FILE):
            time.sleep(2.0)
            completed = True
            break
            
        p = subprocess.run(["powershell", "-Command", "Get-Process *webots* -ErrorAction SilentlyContinue | Measure-Object | Select-Object -ExpandProperty Count"], capture_output=True, text=True)
        count = int(p.stdout.strip()) if p.stdout.strip().isdigit() else 0
        
        if count > 0:
            webots_seen = True
            zero_count = 0
        elif webots_seen:
            zero_count += 1
            if zero_count >= 3:
                completed = True
                break
            
        time.sleep(1.5)
        
    clean_processes()
    dur = time.time() - start_time
    
    log_text = ""
    if os.path.exists(LOG_FILE):
        with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
            log_text = f.read()
            
    found_red = ("RED TILE CENTER REACHED" in log_text or 
                 "Floor Color Detected: RED" in log_text or 
                 "META ROJA ALCANZADA" in log_text or 
                 "Color de piso detectado: RED" in log_text)
                 
    returned_home = ("SURVIVED AND SAFELY PARKED AT (0, 0)" in log_text or 
                     "REGRESAMOS SANOS Y SALVOS" in log_text)
                     
    all_explored = ("ALL ACCESSIBLE CELLS EXPLORED" in log_text or 
                    "TODAS LAS CELDAS ACCESIBLES FUERON EXPLORADAS" in log_text)
    
    final_offset_match = re.findall(r"Offset Celda: \(dX=([+-]?\d+\.?\d*)cm, dY=([+-]?\d+\.?\d*)cm\)", log_text)
    offset_str = f"(dX={final_offset_match[-1][0]}cm, dY={final_offset_match[-1][1]}cm)" if final_offset_match else "N/A"
    
    drift_match = re.findall(r"IMU DRIFT:\s*([+-]?\d+\.?\d*)°", log_text)
    max_drift = max([abs(float(d)) for d in drift_match]) if drift_match else 0.0
    
    backup_count = log_text.count("[BACKUP]")
    front_brake_count = log_text.count("FRONT WALL EMERGENCY BRAKE")
    pid_active_count = log_text.count("Active=True")
    
    status = "SUCCESS" if (returned_home or all_explored) else "INCOMPLETE"
    print(f"[Run {run_idx}] Outcome: {status} | RED: {found_red} | Home: {returned_home} | Offset: {offset_str} | Drift: {max_drift:.1f}° | FrontBrakes: {front_brake_count} | Backups: {backup_count} | PIDActive: {pid_active_count} | Dur: {dur:.1f}s", flush=True)
    
    return {
        "run": run_idx,
        "status": status,
        "success": (returned_home or all_explored),
        "found_red": found_red,
        "returned_home": returned_home,
        "offset": offset_str,
        "drift": max_drift,
        "front_brakes": front_brake_count,
        "backups": backup_count,
        "pid_active": pid_active_count,
        "dur": dur
    }

def main():
    total_runs = 5
    results = []
    for i in range(1, total_runs + 1):
        res = run_single(i)
        results.append(res)
        time.sleep(2.0)
        
    successes = sum(1 for r in results if r["success"])
    print(f"\n==========================================", flush=True)
    print(f"BATCH SUMMARY: {successes}/{total_runs} SUCCESSFUL RUNS ({(successes/total_runs)*100:.1f}%)", flush=True)
    print(f"==========================================", flush=True)

if __name__ == "__main__":
    main()
