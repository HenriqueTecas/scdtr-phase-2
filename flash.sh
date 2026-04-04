#!/bin/bash
# flash.sh — Build and flash firmware to all connected Picos
# Usage:
#   ./flash.sh           flash all detected Picos
#   ./flash.sh /dev/ttyACM0   flash a specific port

set -e
export PATH="$HOME/.local/bin:$PATH"

SKETCH="$( cd "$(dirname "$0")/main" && pwd )"
FQBN="rp2040:rp2040:rpipico"
PICOTOOL="$HOME/.arduino15/packages/rp2040/tools/pqt-picotool/4.1.0-1aec55e/picotool"

echo "=== SCDTR Flash Tool ==="
echo "Sketch: $SKETCH"
echo ""

# ── Step 1: Build ─────────────────────────────────────────────────────────────
echo "[1/3] Compiling..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH" 2>&1 | grep -E "error:|Sketch uses|warning:"
echo ""

# Find the compiled UF2
UF2=$(find "$HOME/.cache/arduino" -name "*.uf2" -newer "$SKETCH/main.ino" 2>/dev/null | head -1)
if [ -z "$UF2" ]; then
    # fallback: look anywhere
    UF2=$(find "$HOME/.cache/arduino" -name "*.uf2" 2>/dev/null | sort -t/ -k9 | tail -1)
fi
echo "UF2: $UF2"
echo ""

# ── Step 2: Find boards ───────────────────────────────────────────────────────
if [ -n "$1" ]; then
    PORTS="$1"
else
    PORTS=$(arduino-cli board list 2>/dev/null | grep "rp2040" | awk '{print $1}' | tr '\n' ' ')
fi

# Also check for boards in BOOTSEL mode (RPI-RP2 mass storage)
BOOTSEL_DRIVES=$(ls /media/"$USER"/RPI-RP2* 2>/dev/null || true)

echo "[2/3] Detected ports: ${PORTS:-none}"
echo "      BOOTSEL drives: ${BOOTSEL_DRIVES:-none}"
echo ""

# ── Step 3: Flash ─────────────────────────────────────────────────────────────
echo "[3/3] Flashing..."

FLASHED=0

# Flash any boards in BOOTSEL mode via UF2 copy (most reliable)
if [ -n "$BOOTSEL_DRIVES" ] && [ -n "$UF2" ]; then
    for drive in $BOOTSEL_DRIVES; do
        echo "  → UF2 copy to $drive"
        cp "$UF2" "$drive/"
        sync
        echo "     Done."
        FLASHED=$((FLASHED+1))
    done
fi

# Flash boards running firmware via arduino-cli upload (uses serial reset)
for port in $PORTS; do
    echo "  → Uploading to $port"

    # Try soft reboot into BOOTSEL first (1200 baud touch)
    python3 -c "
import serial, time
try:
    s = serial.Serial('$port', 1200, timeout=0.5)
    s.dtr = False
    time.sleep(0.1)
    s.close()
except:
    pass
" 2>/dev/null
    sleep 2

    # Now upload
    if arduino-cli upload -p "$port" --fqbn "$FQBN" "$SKETCH" 2>&1 | tee /dev/stderr | grep -q "bytes"; then
        echo "     Done."
        FLASHED=$((FLASHED+1))
    else
        echo "     FAILED — put this board in BOOTSEL mode and re-run."
    fi
done

echo ""
if [ "$FLASHED" -eq 0 ]; then
    echo "No boards flashed."
    echo ""
    echo "  If boards are not detected, put each Pico in BOOTSEL mode:"
    echo "  1. Hold the BOOTSEL button"
    echo "  2. Plug in the USB cable"
    echo "  3. Release BOOTSEL — it appears as RPI-RP2 drive"
    echo "  4. Re-run: ./flash.sh"
    exit 1
else
    echo "Flashed $FLASHED board(s) successfully."
fi
