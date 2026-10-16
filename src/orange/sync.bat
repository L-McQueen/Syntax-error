@echo off
set SCRIPT_DIR=%~dp0
set ORANGE_IP=192.168.137.75
set ORANGE_USER=orangepi

echo ========================================================
echo   SINCRONIZANDO CODIGO CON ORANGE PI ZERO 2W
echo   Destino: %ORANGE_USER%@%ORANGE_IP%:~/orange/
echo ========================================================

ssh %ORANGE_USER%@%ORANGE_IP% "mkdir -p ~/orange/tests && rm -f ~/orange/*.py ~/orange/tests/*.py"
scp "%SCRIPT_DIR%*.py" %ORANGE_USER%@%ORANGE_IP%:~/orange/
scp "%SCRIPT_DIR%tests\*.py" %ORANGE_USER%@%ORANGE_IP%:~/orange/tests/

if %ERRORLEVEL% EQU 0 (
    echo [Listo :V] Archivos sincronizados correctamente en ~/orange/
) else (
    echo [ERROR] Fallo la transferencia SCP. Verifica la conexion WiFi.
)
