# Script de sincronización PowerShell para Orange Pi Zero 2W
$ORANGE_IP = "192.168.137.75"
$ORANGE_USER = "orangepi"
$LOCAL_DIR = $PSScriptRoot

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "  SINCRONIZANDO CODIGO CON ORANGE PI ZERO 2W" -ForegroundColor Cyan
Write-Host "  Destino: $ORANGE_USER@$ORANGE_IP:~/orange/" -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan

ssh "$ORANGE_USER@$ORANGE_IP" "mkdir -p ~/orange/tests && rm -f ~/orange/test.py"
scp "$LOCAL_DIR\*.py" "$ORANGE_USER@$ORANGE_IP:~/orange/"
scp "$LOCAL_DIR\tests\*.py" "$ORANGE_USER@$ORANGE_IP:~/orange/tests/"

if ($LASTEXITCODE -eq 0) {
    Write-Host "[EXITO] Archivos sincronizados en ~/orange/" -ForegroundColor Green
} else {
    Write-Host "[ERROR] Fallo la transferencia." -ForegroundColor Red
}
