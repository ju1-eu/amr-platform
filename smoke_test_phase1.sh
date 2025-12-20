#!/bin/bash
# =============================================================================
# smoke_test_phase1.sh - Phase 1 Smoke Tests für AMR
# =============================================================================
# Verwendung: ./smoke_test_phase1.sh
# Voraussetzung: micro-ROS Agent läuft bereits
# =============================================================================

set -e

echo "=============================================="
echo "  AMR Phase 1 - Smoke Tests (Humble)"
echo "=============================================="
echo ""

# Farben
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

pass() { echo -e "${GREEN}✓ $1${NC}"; }
fail() { echo -e "${RED}✗ $1${NC}"; }
info() { echo -e "${YELLOW}→ $1${NC}"; }

# -----------------------------------------------------------------------------
# Test 1: Node-Check
# -----------------------------------------------------------------------------
echo "[1/6] Node-Check..."
if ros2 node list 2>/dev/null | grep -q "esp32_dual_core"; then
    pass "Node 'esp32_dual_core' gefunden"
else
    fail "Node nicht gefunden - Agent läuft?"
    echo "    Tipp: micro-ros-agent serial --dev /dev/ttyACM0 -b 115200 -v4"
fi

# -----------------------------------------------------------------------------
# Test 2: Topics vorhanden
# -----------------------------------------------------------------------------
echo ""
echo "[2/6] Topics prüfen..."
TOPICS=$(ros2 topic list 2>/dev/null)

for topic in "/cmd_vel" "/odom_raw" "/esp32/heartbeat" "/esp32/led_cmd"; do
    if echo "$TOPICS" | grep -q "$topic"; then
        pass "Topic $topic vorhanden"
    else
        fail "Topic $topic fehlt"
    fi
done

# -----------------------------------------------------------------------------
# Test 3: Heartbeat Rate
# -----------------------------------------------------------------------------
echo ""
echo "[3/6] Heartbeat prüfen (5 Sekunden)..."
info "Messe Frequenz..."
HB_RATE=$(timeout 5 ros2 topic hz /esp32/heartbeat 2>/dev/null | grep "average rate" | tail -1 || echo "")
if [ -n "$HB_RATE" ]; then
    pass "Heartbeat aktiv: $HB_RATE"
else
    fail "Heartbeat nicht messbar"
fi

# -----------------------------------------------------------------------------
# Test 4: Odom Rate
# -----------------------------------------------------------------------------
echo ""
echo "[4/6] Odometrie prüfen (5 Sekunden)..."
info "Messe Frequenz..."
ODOM_RATE=$(timeout 5 ros2 topic hz /odom_raw 2>/dev/null | grep "average rate" | tail -1 || echo "")
if [ -n "$ODOM_RATE" ]; then
    pass "Odometrie aktiv: $ODOM_RATE"
else
    fail "Odometrie nicht messbar"
fi

# -----------------------------------------------------------------------------
# Test 5: Odom Werte ausgeben
# -----------------------------------------------------------------------------
echo ""
echo "[5/6] Aktuelle Odometrie-Werte..."
info "Lese einen Datenpunkt..."
ros2 topic echo /odom_raw --once 2>/dev/null || fail "Keine Odom-Daten"

# -----------------------------------------------------------------------------
# Test 6: Manuelle Tests (Hinweise)
# -----------------------------------------------------------------------------
echo ""
echo "[6/6] Manuelle Tests (Motor + Failsafe)..."
echo ""
echo "  ⚠️  SICHERHEIT: Rover aufbocken oder Räder frei!"
echo ""
echo "  Vorwärts:"
echo "    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \\"
echo "      \"{linear: {x: 0.10}, angular: {z: 0.0}}\""
echo ""
echo "  Stop:"
echo "    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \\"
echo "      \"{linear: {x: 0.0}, angular: {z: 0.0}}\""
echo ""
echo "  Drehen:"
echo "    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \\"
echo "      \"{linear: {x: 0.0}, angular: {z: 0.5}}\""
echo ""
echo "  Failsafe-Test:"
echo "    1. cmd_vel senden (Motor dreht)"
echo "    2. Warten >1 Sekunde (kein neuer Befehl)"
echo "    3. Motor muss stoppen"
echo ""

echo "=============================================="
echo "  Automatische Tests abgeschlossen"
echo "=============================================="
