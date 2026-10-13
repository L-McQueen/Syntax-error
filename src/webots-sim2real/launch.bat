@echo off
cd /d "c:\Users\Sin nombre\Documents\Candidates\WebotsSim2Real"
set HEADLESS=1
set PYTHONUNBUFFERED=1
set WORLD=worlds/sim2real_maze.wbt
if exist target_world.txt (
    set /p WORLD=<target_world.txt
)
"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" --stdout --stderr --batch --mode=fast --no-rendering %WORLD% > simulation_log.txt 2>&1

