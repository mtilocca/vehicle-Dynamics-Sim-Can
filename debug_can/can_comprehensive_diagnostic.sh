#!/bin/bash
# can_comprehensive_diagnostic.sh
# Run this while simulation is running to diagnose CAN TX issues

echo "╔══════════════════════════════════════════════════════════════════════════╗"
echo "║                  CAN Transmission Diagnostic Tool                        ║"
echo "╚══════════════════════════════════════════════════════════════════════════╝"
echo ""

if ! pgrep -f "sim_main" > /dev/null; then
    echo "⚠️  WARNING: sim_main is not running!"
    echo "   Please start simulation first:"
    echo "   ./build/src/sim/sim_main config/scenarios/slalom.json &"
    echo ""
    read -p "Press Enter to start simulation automatically, or Ctrl+C to abort..."
    
    ./build/src/sim/sim_main config/scenarios/slalom.json > /tmp/sim_output.log 2>&1 &
    SIM_PID=$!
    echo "▶️  Started simulation (PID: $SIM_PID)"
    sleep 2
fi

echo "📊 TEST 1: Frame Count Per Second"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Counting frames for 3 seconds..."

TOTAL_FRAMES=$(timeout 3 candump vcan0 2>/dev/null | wc -l)
FRAMES_PER_SEC=$((TOTAL_FRAMES / 3))

echo "Total frames (3 sec):  $TOTAL_FRAMES"
echo "Frames per second:     $FRAMES_PER_SEC"
echo ""

if [ $FRAMES_PER_SEC -lt 100 ]; then
    echo "❌ CRITICAL: Very low frame rate!"
    echo "   Expected: ~370 frames/sec (all 7 IDs)"
    echo "   Actual:   $FRAMES_PER_SEC frames/sec"
elif [ $FRAMES_PER_SEC -lt 200 ]; then
    echo "⚠️  WARNING: Frame rate too low"
    echo "   Expected: ~370 frames/sec (all 7 IDs)"
    echo "   Actual:   $FRAMES_PER_SEC frames/sec"
    echo "   Issue: Some frame IDs not transmitting"
elif [ $FRAMES_PER_SEC -lt 350 ]; then
    echo "⚠️  NOTICE: Frame rate slightly low"
    echo "   Expected: ~370 frames/sec"
    echo "   Actual:   $FRAMES_PER_SEC frames/sec"
else
    echo "✅ OK: Frame rate looks good"
fi

echo ""
echo "📡 TEST 2: Which Frame IDs Are Being Sent"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

timeout 2 candump vcan0 2>/dev/null | \
    awk '{print $3}' | \
    cut -d'#' -f1 | \
    sort | \
    uniq -c | \
    sort -rn > /tmp/can_counts.txt

echo "Frame ID | Count | Rate (Hz) | Expected | Status"
echo "─────────┼───────┼───────────┼──────────┼─────────"

check_frame() {
    local id=$1
    local expected_hz=$2
    local count=$(grep -w "$id" /tmp/can_counts.txt | awk '{print $1}')
    
    if [ -z "$count" ]; then
        count=0
    fi
    
    local hz=$((count / 2))  # 2 second sample
    
    if [ $hz -eq 0 ]; then
        printf " %7s │ %5d │   %5d   │   %4d   │ ❌ MISSING\n" "$id" "$count" "$hz" "$expected_hz"
    elif [ $hz -lt $((expected_hz / 2)) ]; then
        printf " %7s │ %5d │   %5d   │   %4d   │ ⚠️  LOW\n" "$id" "$count" "$hz" "$expected_hz"
    else
        printf " %7s │ %5d │   %5d   │   %4d   │ ✅ OK\n" "$id" "$count" "$hz" "$expected_hz"
    fi
}

check_frame "300" 100
check_frame "310" 100
check_frame "320" 100
check_frame "330" 20
check_frame "331" 20
check_frame "340" 10
check_frame "3F0" 10

echo ""
echo "🔍 TEST 3: Sample Frame Data"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

for id in 300 310 320 330 331 340 3F0; do
    echo -n "0x$id: "
    timeout 1 candump vcan0 2>/dev/null | grep " $id " | head -1
done

echo ""
echo "📋 TEST 4: Diagnosis"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

MISSING=$(grep "MISSING" /tmp/can_counts.txt | wc -l)
LOW=$(grep "LOW" /tmp/can_counts.txt | wc -l)

if [ $(grep -c "330\|331\|340\|3F0" /tmp/can_counts.txt) -lt 4 ]; then
    echo "❌ ISSUE CONFIRMED: Slower frames (50ms/100ms) not transmitting"
    echo ""
    echo "Root Cause:"
    echo "  The TxScheduler is only transmitting fast frames (10ms)."
    echo "  Frames 0x330, 0x331, 0x340, 0x3F0 are MISSING or very low."
    echo ""
    echo "This explains why you see:"
    echo "  ✅ Vehicle State (0x300) - works"
    echo "  ✅ Motor State (0x310) - works"
    echo "  ✅ Brake State (0x320) - works"
    echo "  ❌ Position (0x330) - missing"
    echo "  ❌ Orientation (0x331) - missing"
    echo "  ❌ Drivetrain (0x340) - missing"
    echo "  ❌ Diagnostic (0x3F0) - missing"
    echo ""
    echo "Fix Required:"
    echo "  Check src/sim/*.cpp for CAN TX logic"
    echo "  Ensure tx_sched.due(now) is called EVERY loop iteration"
    echo "  Verify TxScheduler::init() included all 7 frames"
elif [ $FRAMES_PER_SEC -lt 100 ]; then
    echo "❌ ISSUE: Overall CAN TX rate too low"
    echo ""
    echo "Possible causes:"
    echo "  - CAN TX only happens in certain scenario phases"
    echo "  - Real-time pacing sleep time too long"
    echo "  - CAN TX conditional on some flag"
else
    echo "✅ CAN transmission appears healthy!"
fi

echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo "Diagnostic complete. Check results above."
echo "═══════════════════════════════════════════════════════════════════════════"