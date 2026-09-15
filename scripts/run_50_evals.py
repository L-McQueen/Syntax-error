import os
import sys
import time
import json
import subprocess
import re

WORKDIR = r"c:\Users\Sin nombre\Documents\Candidates\WebotsSim2Real"
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
WORLD_FILE = os.path.join(WORKDIR, "worlds", "sim2real_maze.wbt")
LOG_FILE = os.path.join(WORKDIR, "god_mode_live.log")
FLAG_FILE = os.path.join(WORKDIR, "finished.flag")
PROGRESS_FILE = os.path.join(WORKDIR, "batch_progress.txt")
RESULTS_JSON = os.path.join(WORKDIR, "batch_50_results.json")
SUMMARY_MD = os.path.join(WORKDIR, "batch_50_summary.md")

os.environ["HEADLESS"] = "1"

TOTAL_RUNS = 50

def clean_stray_processes():
    try:
        subprocess.run(["powershell", "-Command", "Stop-Process -Name webots* -Force -ErrorAction SilentlyContinue"], capture_output=True)
    except Exception:
        pass

def parse_run_log(log_text):
    found_red = "[VISION] Floor Color Detected: RED" in log_text or "RED TILE CENTER REACHED" in log_text or "RED GOAL REACHED" in log_text
    returned_home = "SURVIVED AND SAFELY PARKED AT (0, 0)" in log_text
    all_explored = "ALL ACCESSIBLE CELLS EXPLORED" in log_text
    timeout = "[TIMEOUT]" in log_text
    
    # Extract ArUco IDs
    aruco_ids = list(set(re.findall(r"\[VISION\] ARUCO FOUND! ID: (\d+)", log_text)))
    
    # Extract colors
    colors = list(set(re.findall(r"\[VISION\] Floor Color Detected: (\w+)", log_text)))
    
    # Extract final cell offset
    final_offset_match = re.findall(r"Offset Celda: \(dX=([+-]?\d+\.?\d*)cm, dY=([+-]?\d+\.?\d*)cm\)", log_text)
    final_dx = float(final_offset_match[-1][0]) if final_offset_match else None
    final_dy = float(final_offset_match[-1][1]) if final_offset_match else None
    
    status = "UNKNOWN"
    if returned_home:
        status = "SUCCESS_RETURNED_HOME"
    elif all_explored:
        status = "ALL_EXPLORED"
    elif timeout:
        status = "TIMEOUT"
    elif found_red:
        status = "GOAL_REACHED_RETURN_FAILED"
    else:
        status = "INCOMPLETE"
        
    return {
        "status": status,
        "found_red": found_red,
        "returned_home": returned_home,
        "all_explored": all_explored,
        "timeout": timeout,
        "aruco_ids": aruco_ids,
        "colors": colors,
        "final_offset_cm": {"dx": final_dx, "dy": final_dy} if final_dx is not None else None
    }

def update_summary_md(results):
    total = len(results)
    if total == 0:
        return
        
    success_count = sum(1 for r in results if r["returned_home"])
    red_found_count = sum(1 for r in results if r["found_red"])
    all_explored_count = sum(1 for r in results if r["all_explored"])
    timeout_count = sum(1 for r in results if r["timeout"])
    
    valid_drifts = []
    for r in results:
        if r["returned_home"] and r["final_offset_cm"]:
            d = (r["final_offset_cm"]["dx"]**2 + r["final_offset_cm"]["dy"]**2)**0.5
            valid_drifts.append(d)
            
    avg_drift = sum(valid_drifts) / len(valid_drifts) if valid_drifts else 0.0
    
    lines = [
        "# Batch 50 Simulation Evaluation Report (Fast Mode - No Rendering)",
        "",
        f"- **Progress**: {total} / {TOTAL_RUNS} completed",
        f"- **Returned Home to (0, 0) Rate**: {success_count}/{total} ({success_count/total*100:.1f}%)",
        f"- **Goal (RED) Found Rate**: {red_found_count}/{total} ({red_found_count/total*100:.1f}%)",
        f"- **Fully Explored / Backtracked**: {all_explored_count}/{total} ({all_explored_count/total*100:.1f}%)",
        f"- **Timeouts**: {timeout_count}/{total}",
        f"- **Average Parking Drift at (0, 0)**: {avg_drift:.2f} cm",
        "",
        "## Per-Run Breakdown",
        "",
        "| Run | Outcome | RED Goal Found | Returned (0,0) | Final Offset (dX, dY) | ArUco Seen | Colors Seen | Duration |",
        "|-----|---------|----------------|----------------|-----------------------|------------|-------------|----------|"
    ]
    
    for r in results:
        offset_str = f"({r['final_offset_cm']['dx']:+.1f}, {r['final_offset_cm']['dy']:+.1f}) cm" if r.get("final_offset_cm") else "N/A"
        aruco_str = ", ".join(r["aruco_ids"]) if r["aruco_ids"] else "None"
        color_str = ", ".join(r["colors"]) if r["colors"] else "None"
        dur_str = f"{r['duration_sec']:.1f}s"
        lines.append(f"| {r['run']} | {r['status']} | {r['found_red']} | {r['returned_home']} | {offset_str} | {aruco_str} | {color_str} | {dur_str} |")
        
    with open(SUMMARY_MD, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

def main():
    print(f"Starting 50 Fast Evaluations in {WORKDIR}...")
    results = []
    
    # Clean previous files
    if os.path.exists(RESULTS_JSON):
        try: os.remove(RESULTS_JSON)
        except Exception: pass
    if os.path.exists(SUMMARY_MD):
        try: os.remove(SUMMARY_MD)
        except Exception: pass
        
    for run_idx in range(1, TOTAL_RUNS + 1):
        clean_stray_processes()
        time.sleep(1.0)
        
        # Clean flag and log
        if os.path.exists(FLAG_FILE):
            try: os.remove(FLAG_FILE)
            except Exception: pass
            
        with open(LOG_FILE, "w", encoding="utf-8") as f:
            f.write("")
            
        start_wall_time = time.time()
        print(f"\n[RUN {run_idx}/{TOTAL_RUNS}] Launching Webots in fast mode no-rendering...", flush=True)
        
        with open(PROGRESS_FILE, "w", encoding="utf-8") as f:
            f.write(f"RUN {run_idx}/{TOTAL_RUNS} IN_PROGRESS | Elapsed: {time.time() - start_wall_time:.1f}s\n")
            
        cmd = [
            WEBOTS_EXE,
            "--batch",
            "--stdout",
            "--stderr",
            "--mode=fast",
            "--no-rendering",
            WORLD_FILE
        ]
        
        sim_log_path = os.path.join(WORKDIR, "simulation_log.txt")
        with open(sim_log_path, "w", encoding="utf-8") as out_f:
            proc = subprocess.Popen(cmd, stdout=out_f, stderr=subprocess.STDOUT, cwd=WORKDIR)
            
            # Wait for completion or timeout
            poll_interval = 1.0
            max_wall_time = 90.0 # 90 wall seconds max per run
            completed = False
            while time.time() - start_wall_time < max_wall_time:
                ret = proc.poll()
                if ret is not None:
                    completed = True
                    break
                # Check if flag exists
                if os.path.exists(FLAG_FILE):
                    # Give it 2 seconds for supervisor to call simulationQuit
                    time.sleep(2.0)
                    ret = proc.poll()
                    if ret is not None:
                        completed = True
                        break
                    else:
                        proc.terminate()
                        completed = True
                        break
                time.sleep(poll_interval)
                
            if not completed:
                print(f"[RUN {run_idx}] Wall-clock timeout exceeded ({max_wall_time}s). Terminating...", flush=True)
                proc.kill()
                
        dur_wall = time.time() - start_wall_time
        
        # Read god_mode_live.log
        try:
            with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
                log_content = f.read()
        except Exception:
            log_content = ""
            
        parsed = parse_run_log(log_content)
        parsed["run"] = run_idx
        parsed["duration_sec"] = dur_wall
        results.append(parsed)
        
        print(f"[RUN {run_idx}/{TOTAL_RUNS}] Status: {parsed['status']} | Red: {parsed['found_red']} | Returned: {parsed['returned_home']} | Wall Time: {dur_wall:.1f}s", flush=True)
        
        # Save JSON
        with open(RESULTS_JSON, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)
            
        # Update progress and markdown summary
        update_summary_md(results)
        with open(PROGRESS_FILE, "w", encoding="utf-8") as f:
            f.write(f"COMPLETED {run_idx}/{TOTAL_RUNS} | Last: {parsed['status']} | Red Found: {sum(1 for r in results if r['found_red'])} | Returned: {sum(1 for r in results if r['returned_home'])}\n")
            
        clean_stray_processes()
        
    print("\n=======================================================", flush=True)
    print("  ALL 50 EVALUATIONS COMPLETED SUCCESSFULLY!           ", flush=True)
    print("=======================================================", flush=True)
    with open(os.path.join(WORKDIR, "BATCH_DONE"), "w") as f:
        f.write("DONE")

if __name__ == "__main__":
    main()
