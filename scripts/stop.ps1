$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $PSScriptRoot

Set-Location $Root

docker compose `
    -f docker/docker-compose.yml `
    down