$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $PSScriptRoot

Set-Location $Root

docker compose `
    -f docker/docker-compose.yml `
    up -d

Write-Host ""
Write-Host "Container started."
Write-Host "Open:"
Write-Host ""
Write-Host "    http://localhost:6080/vnc.html"
Write-Host ""