param(
    [switch]$Headless,
    [switch]$SkipImageBuild
)

$ErrorActionPreference = "Stop"
$taskRoot = Split-Path -Parent $PSScriptRoot
Set-Location $taskRoot
$composeArguments = @("compose", "-f", "docker/docker-compose.yml", "up", "-d", "--force-recreate")
if (-not $SkipImageBuild) {
    $composeArguments += "--build"
}
docker @composeArguments
if ($LASTEXITCODE -ne 0) {
    throw "Container startup failed with exit code $LASTEXITCODE"
}
Write-Host "Gazebo and RViz: http://localhost:6080/vnc.html"
$demoArguments = @("exec", "-it", "gazebo-autonomy-test", "bash", "/workspace/scripts/run_demo.sh")
if ($Headless) {
    $demoArguments += @("gui:=false", "rviz:=false")
}
docker @demoArguments
if ($LASTEXITCODE -ne 0) {
    throw "Demo failed with exit code $LASTEXITCODE"
}
