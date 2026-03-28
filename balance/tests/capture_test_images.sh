#!/bin/bash
# capture_test_images.sh  (v3 — pose-based)
# ------------------------------------------
# Captures test images synchronized with balance_v1.yaml pose sequence.
# Run on the Pi 5 host (not in Docker).
#
# Usage:
#   ./capture_test_images.sh [--outdir PATH] [--device /dev/videoX]
#
# Workflow:
#   1. Start pose_test with balance_v1.yaml in another terminal
#      (configure 10 second hold per pose)
#   2. Run this script
#   3. Press ENTER for each pose when the arm has settled
#
# shoulder_pan stays at 1.50 throughout — cup stays in same
# lateral position so the ROI works consistently.
#
# After capturing:
#   scp -r bueche@bueche-rpi5.local:~/ball_test_images ~/Desktop/

set -e

DEVICE=${DEVICE:-/dev/video0}
WIDTH=640
HEIGHT=480
OUTDIR=$HOME/ball_test_images
SKIP_EXISTING=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --outdir)        OUTDIR="$2"; shift ;;
        --device)        DEVICE="$2"; shift ;;
        --skip-existing) SKIP_EXISTING=true ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
    shift
done

mkdir -p "$OUTDIR"

# ── Camera settings ───────────────────────────────────────────────────────
echo "Setting camera exposure for dark cup / visible ball highlight..."
v4l2-ctl --device=$DEVICE --set-ctrl=auto_exposure=1
v4l2-ctl --device=$DEVICE --set-ctrl=exposure_time_absolute=50
v4l2-ctl --device=$DEVICE --set-ctrl=brightness=-30
v4l2-ctl --device=$DEVICE --set-ctrl=backlight_compensation=0
v4l2-ctl --device=$DEVICE --set-ctrl=gain=0

echo ""
echo "=========================================================="
echo "  Ball Detector Test Image Capture  (v3 — pose-based)"
echo "=========================================================="
echo "  Camera : $DEVICE  ${WIDTH}x${HEIGHT}"
echo "  Output : $OUTDIR"
echo "=========================================================="
echo ""
echo "  Run pose_test with balance_v1.yaml in another terminal."
echo "  Press ENTER here for each pose when arm has settled."
echo "  shoulder_pan=1.50 throughout — ROI is stable."
echo ""

echo "Warming up camera (3 seconds)..."
ffmpeg -f v4l2 -input_format mjpeg \
       -video_size ${WIDTH}x${HEIGHT} -framerate 30 \
       -i "$DEVICE" -t 3 -f null - 2>/dev/null
echo "Camera ready."
echo ""

CAPTURED=0; SKIPPED=0; FAILED=0

capture_frame() {
    local name="$1"
    local filepath="$OUTDIR/${name}.jpg"
    if $SKIP_EXISTING && [ -f "$filepath" ]; then
        echo "  SKIP  (exists): ${name}.jpg"
        ((SKIPPED++)) || true; return
    fi
    ffmpeg -f v4l2 -input_format mjpeg \
           -video_size ${WIDTH}x${HEIGHT} -framerate 30 \
           -i "$DEVICE" -frames:v 1 -update 1 -y \
           "$filepath" 2>/dev/null
    if [ -f "$filepath" ]; then
        size=$(du -h "$filepath" | cut -f1)
        echo "  ✓  ${name}.jpg  ($size)"
        ((CAPTURED++)) || true
    else
        echo "  ✗  FAILED: ${name}.jpg"
        ((FAILED++)) || true
    fi
}

prompt_and_capture() {
    local name="$1"; local pose_desc="$2"
    local ball_desc="$3"; local detail="${4:-}"
    echo ""
    echo "── $name"
    echo "   Pose : $pose_desc"
    echo "   Ball : $ball_desc"
    [ -n "$detail" ] && echo "   Note : $detail"
    echo -n "   ENTER=capture  s=skip  q=quit : "
    read answer
    case "${answer:0:1}" in
        s|S) echo "   Skipped."; ((SKIPPED++)) || true; return ;;
        q|Q) echo ""; print_summary; exit 0 ;;
    esac
    capture_frame "$name"
}

print_summary() {
    echo ""
    echo "=========================================================="
    echo "  Done. Captured=$CAPTURED  Skipped=$SKIPPED  Failed=$FAILED"
    echo "=========================================================="
    ls "$OUTDIR"/*.jpg 2>/dev/null | \
        while read f; do echo "  $(basename $f)  ($(du -h "$f"|cut -f1))"; done
    echo ""
    echo "Copy to Mac:"
    echo "  scp -r bueche@bueche-rpi5.local:$OUTDIR ~/Desktop/"
}

# ══════════════════════════════════════════════════════════════════════════
# SHOT LIST — mirrors balance_v1.yaml
# ══════════════════════════════════════════════════════════════════════════

echo "── BASELINE ────────────────────────────────────────────────"

prompt_and_capture "pose_centre_no_ball" \
    "pose 1 — centre (wrist_roll=1.5999, wrist_flex=2.7)" \
    "Remove ball from cup before this shot." \
    "Verifies cup detection alone, no ball."

echo ""
echo "── CENTRE ──────────────────────────────────────────────────"

prompt_and_capture "pose_centre" \
    "pose 1 — centre (wrist_roll=1.5999, wrist_flex=2.7)" \
    "Ball near cup centre. Expected norm ~(0.0, 0.0)." \
    "Replace ball before this shot."

echo ""
echo "── WRIST_ROLL TILTS (left/right) ───────────────────────────"

prompt_and_capture "pose_tilt_right" \
    "pose 2 — tilt right (wrist_roll=1.2)" \
    "Ball rolls to one side. Note which image direction." \
    "Key shot for axis mapping."

prompt_and_capture "pose_tilt_left" \
    "pose 3 — tilt left (wrist_roll=1.9)" \
    "Ball rolls opposite side from tilt_right." \
    "Should be roughly mirror of tilt_right."

prompt_and_capture "pose_centre_2" \
    "pose 4 — centre (wrist_roll=1.5999)" \
    "Ball returns near centre." \
    "Checks repeatability."

echo ""
echo "── WRIST_FLEX / ELBOW TILTS (forward/back) ─────────────────"

prompt_and_capture "pose_tilt_forward" \
    "pose 5 — forward (wrist_flex=2.0)" \
    "Ball rolls forward. Note which image direction." \
    "Key shot for axis mapping."

prompt_and_capture "pose_tilt_back" \
    "pose 6 — back (elbow_flex=1.2705)" \
    "Ball rolls backward. Opposite of tilt_forward." \
    ""

prompt_and_capture "pose_centre_3" \
    "pose 7 — centre (wrist_roll=1.5999)" \
    "Ball returns near centre." \
    "Checks repeatability across sequence."

echo ""
echo "── REPEAT TILTS (poses 7-9) ────────────────────────────────"
echo "   These repeat the tilt poses — used to check consistency."

prompt_and_capture "pose_tilt_right_2" \
    "pose 7 — tilt right again (wrist_roll=1.2)" \
    "Should match pose_tilt_right closely." \
    ""

prompt_and_capture "pose_tilt_left_2" \
    "pose 8 — tilt left again (wrist_roll=1.9)" \
    "Should match pose_tilt_left closely." \
    ""

prompt_and_capture "pose_tilt_back_2" \
    "pose 9 — back again (elbow_flex=1.2705)" \
    "Should match pose_tilt_back closely." \
    ""

echo ""
echo "── EDGE CASE ───────────────────────────────────────────────"

prompt_and_capture "no_cup" \
    "Move arm so cup is out of frame." \
    "Expected: cup=N  ball=N" \
    "Tests graceful not-detected handling."

print_summary
