#!/usr/bin/env bash
# Re-opens Isaac Sim GUI inside an already-running container.
# Useful when you closed the window but the container is still up.
set -euo pipefail

CONTAINER=$(docker ps --filter "ancestor=isaac-sim-quickstart:latest" --format '{{.Names}}' | head -1)

if [[ -z "$CONTAINER" ]]; then
    # Fallback: find by base image
    CONTAINER=$(docker ps --filter "ancestor=nvcr.io/nvidia/isaac-sim:5.1.0" --format '{{.Names}}' | head -1)
fi

if [[ -z "$CONTAINER" ]]; then
    echo "ERROR: No running Isaac Sim container found."
    echo "Start one with:  docker compose up -d"
    exit 1
fi

echo "Found container: $CONTAINER"
echo "Launching Isaac Sim GUI on DISPLAY=${DISPLAY:-:20}..."

docker exec -d \
    -e DISPLAY="${DISPLAY:-:20}" \
    -e XAUTHORITY=/tmp/.host-Xauthority \
    "$CONTAINER" \
    /isaac-sim/isaac-sim.sh --enable isaacsim.code_editor.vscode

echo "Isaac Sim GUI launched — it should appear on your display shortly."
