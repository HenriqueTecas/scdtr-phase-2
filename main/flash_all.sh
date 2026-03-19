#!/bin/bash

# Flash all Picos with the same firmware (node ID assigned dynamically at boot).
# Usage: ./flash_all.sh

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
FIRMWARE="$PROJECT_DIR/.pio/build/pico/firmware.uf2"

echo "=== SCDTR Multi-Pico Flashing Script ==="
echo "Node IDs are assigned automatically at boot via flash unique ID."
echo ""

# Build once
echo "Building firmware..."
cd "$PROJECT_DIR"
pio run
if [ $? -ne 0 ]; then
    echo "ERROR: Build failed"
    exit 1
fi
echo "Build OK: $FIRMWARE"
echo ""

# Flash to each Pico in turn
for BOARD in 1 2 3; do
    echo "=========================================="
    echo "Flash Pico #$BOARD  (same firmware for all)"
    echo "=========================================="
    echo "1. Hold BOOTSEL button on Pico #$BOARD"
    echo "2. Plug it in via USB"
    echo "3. Release BOOTSEL"
    echo ""
    read -p "Press Enter when Pico #$BOARD is in BOOTSEL mode..."

    MOUNT=""
    for candidate in "/media/$USER/RPI-RP2" "/media/RPI-RP2" "/run/media/$USER/RPI-RP2"; do
        [ -d "$candidate" ] && MOUNT="$candidate" && break
    done

    if [ -z "$MOUNT" ]; then
        echo "ERROR: RPI-RP2 not found. Searching..."
        ls -la /media/$USER/ 2>/dev/null || echo "No media mounts"
        read -p "Enter mount path manually (or Ctrl+C to abort): " MOUNT
    fi

    echo "Copying firmware to $MOUNT ..."
    cp "$FIRMWARE" "$MOUNT/"
    if [ $? -eq 0 ]; then
        echo "Pico #$BOARD flashed OK – it will restart automatically."
    else
        echo "ERROR: copy failed"
    fi
    echo ""
    read -p "Press Enter to continue to next Pico (or Ctrl+C to exit)..."
    echo ""
done

echo "=========================================="
echo "All Picos flashed with the same firmware!"
echo "Each Pico will negotiate its own node ID (1/2/3) at boot."
echo "=========================================="
