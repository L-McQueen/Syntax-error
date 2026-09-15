@echo off
cd /d "c:\Users\Sin nombre\Documents\Candidates\WebotsSim2Real"
set HEADLESS=1
"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" --stdout --stderr --batch --mode=fast --no-rendering worlds/sim2real_maze.wbt > simulation_log.txt 2>&1
