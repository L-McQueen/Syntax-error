# Script para ejecutar Webots Sim2Real con telemetria en vivo y registro en simulation_log.txt
Set-Location "c:\Users\Sin nombre\Documents\Candidates\WebotsSim2Real"
$webots = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
$world = "worlds/sim2real_maze.wbt"

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host "  Lanzando Webots Sim2Real con Ground Truth (God Mode)    " -ForegroundColor Green
Write-Host "==========================================================" -ForegroundColor Cyan

# Ejecuta Webots transmitiendo los logs en vivo por consola y guardando en simulation_log.txt
& $webots --stdout --stderr --mode=realtime $world | Tee-Object -FilePath "simulation_log.txt"
