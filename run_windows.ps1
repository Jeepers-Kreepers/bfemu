# bfemu - PowerShell launcher (alternative to run_windows.bat)
$ErrorActionPreference = "Stop"
Set-Location -Path $PSScriptRoot
$port = if ($args.Count -ge 1) { $args[0] } else { 5761 }

Write-Host "bfemu - virtual Betaflight 4.5.4" -ForegroundColor Cyan
Write-Host "Configurator: tcp://127.0.0.1:$port   Web monitor: http://127.0.0.1:8088"

$py = (Get-Command python -ErrorAction SilentlyContinue) ?? (Get-Command py -ErrorAction SilentlyContinue)
if (-not $py) {
    Write-Host "[X] Python not found in PATH. Install Python 3.10+ and tick 'Add to PATH'." -ForegroundColor Red
    Read-Host "Press Enter to close"
    exit 1
}

New-Item -ItemType Directory -Force -Path "$PSScriptRoot\logs" | Out-Null
& $py.Source -u -m bfemu --tcp "127.0.0.1:$port" --http 8088 --log "$PSScriptRoot\logs\bfemu.log"
$code = $LASTEXITCODE
Write-Host "`nExit code: $code. Log: logs\bfemu.log"
Read-Host "Press Enter to close"
