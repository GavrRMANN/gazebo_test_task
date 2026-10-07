#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: bash scripts/run_demo_host.sh [--skip-image-build] [--headless]"
}

build_image=true
headless=false
for argument in "$@"; do
    case "$argument" in
        --skip-image-build) build_image=false ;;
        --headless) headless=true ;;
        --help|-h) usage; exit 0 ;;
        *) echo "Unknown argument: $argument" >&2; usage >&2; exit 2 ;;
    esac
done

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v docker >/dev/null 2>&1; then
    echo "Docker is required. Install Docker Engine with Compose on Linux or Docker Desktop on macOS." >&2
    exit 1
fi
docker compose version >/dev/null
if ! docker info >/dev/null 2>&1; then
    echo "Docker is unavailable. Start Docker and check access to its daemon." >&2
    exit 1
fi

compose_arguments=(compose -f docker/docker-compose.yml up -d --force-recreate)
if "$build_image"; then
    compose_arguments+=(--build)
fi
docker "${compose_arguments[@]}"
echo "Gazebo and RViz: http://localhost:6080/vnc.html"

demo_arguments=(exec -i)
if [[ -t 0 && -t 1 ]]; then
    demo_arguments+=(-t)
fi
demo_arguments+=(gazebo-autonomy-test bash /workspace/scripts/run_demo.sh)
if "$headless"; then
    demo_arguments+=(gui:=false rviz:=false)
fi
exec docker "${demo_arguments[@]}"
