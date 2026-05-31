#!/bin/bash
# Syncs the local james_perception repo to the Jetson over SSH.
# Run this after making changes on the laptop to test them on the Jetson
# without needing to commit and push first.
#
# Usage:
#   ./sync_james_perception.sh              # sync to default host (jetson-orin)
#   ./sync_james_perception.sh james@192.168.1.100  # override host

set -euo pipefail

JETSON="${1:-jetson-orin}"
LOCAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_DIR="~/git/james_perception"

echo "=== Syncing to $JETSON:$REMOTE_DIR ==="

rsync -av \
    --exclude=".git" \
    --exclude="build/" \
    --exclude="install/" \
    --exclude="log/" \
    "$LOCAL_DIR/" "$JETSON:$REMOTE_DIR/"

echo ""
echo "=== Done! Build on Jetson: ==="
echo "  docker exec -it robojames bash"
echo "  cd ~/git/james_perception"
echo "  colcon build --symlink-install --packages-select james_perception"
