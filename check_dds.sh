#!/bin/bash
# check_dds.sh — ROS2/DDS diagnostics for the James robot.
#
# Run on BOTH the laptop AND inside the Docker container on the Jetson.
# Paste the combined output here to diagnose DDS connectivity issues.
#
# Usage:
#   ./check_dds.sh           # diagnostics only
#   ./check_dds.sh --fix     # also attempt to fix detected issues
#
# Lives in james_perception because that repo is mounted inside the container.

FIX=false
PUB=false
SUB=false
for arg in "$@"; do
    case "$arg" in
        --fix) FIX=true ;;
        --pub) PUB=true ;;
        --sub) SUB=true ;;
    esac
done

# ── Publisher mode ────────────────────────────────────────────────────────────
# Run this on one machine while --sub runs on the other.
if $PUB; then
    echo ""
    echo "Publishing test messages on /james/dds_check at 2 Hz for 30 seconds."
    echo "Now run on the OTHER machine:  ./check_dds.sh --sub"
    echo "Press Ctrl-C to stop early."
    echo ""
    if ! command -v ros2 &>/dev/null; then
        echo "ERROR: ros2 not found — source /opt/ros/humble/setup.bash first"
        exit 1
    fi
    ros2 topic pub /james/dds_check std_msgs/msg/String \
        "data: 'hello_from_$(hostname)'" --rate 2
    exit 0
fi

# ── Subscriber mode ───────────────────────────────────────────────────────────
if $SUB; then
    echo ""
    echo "Listening on /james/dds_check for 15 seconds..."
    echo "Make sure the OTHER machine is running:  ./check_dds.sh --pub"
    echo ""
    if ! command -v ros2 &>/dev/null; then
        echo "ERROR: ros2 not found — source /opt/ros/humble/setup.bash first"
        exit 1
    fi
    MSG=$(timeout 15 ros2 topic echo /james/dds_check --once 2>/dev/null || true)
    if [ -n "$MSG" ]; then
        echo -e "\033[0;32m✓ Data received — DDS data transfer works!\033[0m"
        echo "$MSG"
    else
        echo -e "\033[0;31m✗ No data in 15 seconds.\033[0m"
        echo "  Discovery may work but TCP data transfer is blocked."
        echo "  Check: firewall rules, FASTDDS_BUILTIN_TRANSPORTS on both sides."
    fi
    exit 0
fi

# ── Colour helpers ────────────────────────────────────────────────────────────
if [ -t 1 ]; then
    R='\033[0;31m' G='\033[0;32m' Y='\033[1;33m' B='\033[1;34m' BOLD='\033[1m' N='\033[0m'
else
    R='' G='' Y='' B='' BOLD='' N=''
fi

ok()      { echo -e "  ${G}✓${N} $*"; }
fail()    { echo -e "  ${R}✗${N} $*"; }
warn()    { echo -e "  ${Y}⚠${N} $*"; }
info()    { echo -e "  ${B}→${N} $*"; }
section() { echo -e "\n${BOLD}══════ $* ══════${N}"; }

ISSUES=()
FIXES=()
note_issue() { ISSUES+=("$1"); }
note_fix()   { FIXES+=("$1"); }

# ── 1. Context ────────────────────────────────────────────────────────────────
section "CONTEXT"
echo "  Date/time : $(date '+%Y-%m-%d %H:%M:%S')"
echo "  User      : $(whoami)"
echo "  Hostname  : $(hostname)"

if [ -f /.dockerenv ]; then
    echo -e "  Running   : ${Y}inside Docker container${N}"
    IN_DOCKER=true
else
    echo -e "  Running   : on host (laptop or Jetson outside container)"
    IN_DOCKER=false
fi

# If on Jetson host, check container status
if ! $IN_DOCKER && command -v docker &>/dev/null; then
    section "DOCKER CONTAINER"
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^robojames$"; then
        ok "Container 'robojames' is running"
        CONTAINER_FASTDDS=$(docker exec robojames bash -c \
            'echo $FASTDDS_BUILTIN_TRANSPORTS' 2>/dev/null || echo "exec failed")
        echo "  Container FASTDDS: ${CONTAINER_FASTDDS:-empty}"
        if [ "$CONTAINER_FASTDDS" = "LARGE_DATA" ]; then
            ok "Container has FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA"
        else
            fail "Container FASTDDS is '${CONTAINER_FASTDDS}' — should be LARGE_DATA"
            note_issue "Container missing FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA (re-run ./run.sh)"
        fi
    else
        fail "Container 'robojames' is NOT running"
        note_issue "Docker container not running — run ./run.sh"
    fi
fi

# ── 2. ROS2 environment ───────────────────────────────────────────────────────
section "ROS2 ENVIRONMENT"

if command -v ros2 &>/dev/null; then
    ok "ros2 found at: $(command -v ros2)"
else
    fail "ros2 not in PATH — not sourced"
    note_issue "ros2 not sourced"
    if $FIX && [ -f /opt/ros/humble/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
        note_fix "Sourced /opt/ros/humble/setup.bash (this shell only)"
        ok "Sourced /opt/ros/humble/setup.bash"
    fi
fi

echo "  ROS_DISTRO           : ${ROS_DISTRO:-not set}"

FASTDDS="${FASTDDS_BUILTIN_TRANSPORTS:-}"
echo "  FASTDDS_BUILTIN_TRANSPORTS: '${FASTDDS}'"
if [ "$FASTDDS" = "LARGE_DATA" ]; then
    ok "FASTDDS_BUILTIN_TRANSPORTS = LARGE_DATA"
elif [ -z "$FASTDDS" ]; then
    fail "FASTDDS_BUILTIN_TRANSPORTS is EMPTY — using default UDP transport"
    note_issue "FASTDDS_BUILTIN_TRANSPORTS not set (other side uses LARGE_DATA → mismatch)"
    if $FIX; then
        export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
        note_fix "Exported FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA (this shell only — add to ~/.bashrc)"
        warn "Set LARGE_DATA for this shell. Add to ~/.bashrc to make permanent."
    fi
else
    warn "FASTDDS_BUILTIN_TRANSPORTS='${FASTDDS}' — expected LARGE_DATA"
    note_issue "FASTDDS_BUILTIN_TRANSPORTS is '$FASTDDS', not LARGE_DATA"
fi

DOMAIN="${ROS_DOMAIN_ID:-}"
echo "  ROS_DOMAIN_ID        : '${DOMAIN}' (empty = 0)"
if [ -n "$DOMAIN" ] && [ "$DOMAIN" != "0" ]; then
    fail "ROS_DOMAIN_ID=$DOMAIN — must be 0 to match Jetson"
    note_issue "ROS_DOMAIN_ID=$DOMAIN (must be 0)"
fi

# ros2topic CLI tool
if $IN_DOCKER; then
    if dpkg-query -s ros-humble-ros2topic &>/dev/null 2>&1; then
        ok "ros-humble-ros2topic: installed"
    else
        fail "ros-humble-ros2topic: NOT installed — 'ros2 topic list' will fail"
        note_issue "ros-humble-ros2topic not installed"
        if $FIX; then
            sudo apt-get install -y -q ros-humble-ros2topic 2>/dev/null && \
                note_fix "Installed ros-humble-ros2topic" && \
                ok "Installed ros-humble-ros2topic"
        fi
    fi
fi

# ── 3. Network ────────────────────────────────────────────────────────────────
section "NETWORK INTERFACES"

while IFS= read -r line; do
    if [[ $line =~ ^[0-9]+:\ ([^:@]+) ]]; then
        IFACE="${BASH_REMATCH[1]}"
        IFACE="${IFACE%%@*}"
    elif [[ $line =~ inet\ ([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+/[0-9]+) ]]; then
        IP="${BASH_REMATCH[1]}"
        if [[ $IFACE =~ ^(en|eth) ]]; then TAG="[ethernet]"
        elif [[ $IFACE =~ ^(wl|wifi) ]]; then TAG="[wifi]"
        elif [[ $IFACE =~ ^lo ]]; then TAG="[loopback]"
        elif [[ $IFACE =~ ^docker ]]; then TAG="[docker-bridge]"
        else TAG=""; fi
        echo "  $IFACE  $IP  $TAG"
    fi
done < <(ip addr show 2>/dev/null)

echo ""
echo "  Default route(s):"
ip route show default 2>/dev/null | while IFS= read -r route; do
    echo "    $route"
    if [[ $route =~ metric\ ([0-9]+) ]]; then
        METRIC="${BASH_REMATCH[1]}"
    else
        METRIC=0
    fi
    if [[ $route =~ dev\ (en|eth) ]]; then
        ok "Ethernet route (metric $METRIC) — preferred for DDS"
    elif [[ $route =~ dev\ (wl|wifi) ]]; then
        if [ "${METRIC:-999}" -lt 200 ]; then
            warn "WiFi is the PRIMARY default route (metric $METRIC) — DDS unreliable for large msgs"
            note_issue "Default route via WiFi — need wired ethernet for point cloud topics"
        else
            info "WiFi backup route (metric $METRIC) — ok as fallback"
        fi
    fi
done

# ── 4. ROS2 daemon ────────────────────────────────────────────────────────────
section "ROS2 DAEMON"

if command -v ros2 &>/dev/null; then
    DAEMON_STATUS=$(ros2 daemon status 2>/dev/null || echo "unknown")
    echo "  Status: $DAEMON_STATUS"

    if echo "$DAEMON_STATUS" | grep -qi "running"; then
        ok "Daemon is running"
        if $FIX; then
            info "Restarting daemon to flush stale DDS discovery cache..."
            ros2 daemon stop 2>/dev/null; sleep 0.5
            ros2 daemon start 2>/dev/null
            note_fix "Restarted ros2 daemon"
            ok "Daemon restarted"
        fi
    else
        warn "Daemon not running — will auto-start on next ros2 command"
        if $FIX; then
            ros2 daemon start 2>/dev/null
            note_fix "Started ros2 daemon"
        fi
    fi
else
    warn "ros2 not available — cannot check daemon"
fi

# ── 5. DDS packet capture ─────────────────────────────────────────────────────
section "DDS DISCOVERY TRAFFIC (3s)"

if command -v tcpdump &>/dev/null; then
    info "Listening for DDS UDP packets on port 7400 (3 seconds) — may ask for sudo password..."
    PACKETS=$(sudo timeout 3 tcpdump -i any -n udp port 7400 2>/dev/null | head -30 || true)
    if [ -n "$PACKETS" ]; then
        ok "DDS packets seen:"
        echo "$PACKETS" | sed 's/^/    /'
        REMOTE=$(echo "$PACKETS" | grep -oP '\d+\.\d+\.\d+\.\d+' | grep -v '^127\.' | sort -u || true)
        if [ -n "$REMOTE" ]; then
            ok "Remote DDS participants detected: $(echo "$REMOTE" | tr '\n' ' ')"
        else
            warn "DDS packets found but only from localhost — no remote participants discovered yet"
        fi
    else
        fail "No DDS packets on port 7400 in 3 seconds"
        note_issue "No DDS traffic visible — firewall, wrong transport, or no ROS2 nodes running"
    fi
else
    warn "tcpdump not installed — skipping packet capture"
    info "Install: sudo apt-get install tcpdump"
fi

# ── 6. Firewall ───────────────────────────────────────────────────────────────
section "FIREWALL"

if command -v ufw &>/dev/null; then
    UFW_OUT=$(sudo ufw status 2>/dev/null || ufw status 2>/dev/null || echo "cannot read")
    if echo "$UFW_OUT" | grep -q "^Status: inactive"; then
        ok "UFW: inactive — not blocking anything"
    elif echo "$UFW_OUT" | grep -q "^Status: active"; then
        warn "UFW is ACTIVE — may block DDS ports 7400-7650"
        echo "$UFW_OUT" | head -25 | sed 's/^/    /'
        note_issue "UFW active — verify ports 7400-7650 UDP are allowed on LAN"
        if $FIX; then
            info "Adding UFW rule for ROS2 DDS (UDP 7400-7650 from local subnet)..."
            sudo ufw allow from 192.168.1.0/24 to any port 7400:7650 proto udp \
                comment "ROS2 DDS" 2>/dev/null && \
                note_fix "Added UFW rule: allow DDS UDP 7400-7650 from 192.168.1.0/24" && \
                ok "UFW rule added"
        fi
    else
        info "UFW status: $UFW_OUT"
    fi
else
    info "UFW not installed"
fi

# ── 7. ROS2 topics & nodes ────────────────────────────────────────────────────
section "ROS2 TOPICS"

if command -v ros2 &>/dev/null; then
    info "ros2 topic list (5s timeout)..."
    TOPICS=$(timeout 5 ros2 topic list 2>/dev/null || echo "(timeout/error)")
    echo "$TOPICS" | sed 's/^/  /'

    COUNT=$(echo "$TOPICS" | grep -c '^/' 2>/dev/null || echo 0)
    if [ "$COUNT" -le 2 ]; then
        fail "Only $COUNT topic(s) — likely just local /parameter_events and /rosout"
        note_issue "Only $COUNT remote topics visible — DDS discovery not reaching other machine"
    else
        ok "$COUNT topics visible — DDS discovery works"

        # Discovery (UDP) working does not guarantee data transfer (TCP) works.
        # Try to actually receive one message to verify the full data path.
        info "Testing data transfer — receiving one message from /rosout (3s)..."
        DATA=$(timeout 3 ros2 topic echo /rosout --once 2>/dev/null || true)
        if [ -n "$DATA" ]; then
            ok "Data transfer works — message received"
        else
            warn "Discovery OK but no data received in 3s — TCP may be blocked"
            note_issue "Topics visible but data does not flow — check firewall TCP rules or FASTDDS mismatch"
            info "For a clearer test: run './check_dds.sh --pub' on one machine and './check_dds.sh --sub' on the other"
        fi
    fi
else
    warn "ros2 not available"
fi

section "ROS2 NODES"

if command -v ros2 &>/dev/null; then
    NODES=$(timeout 5 ros2 node list 2>/dev/null || echo "(timeout/error)")
    if [ -z "$NODES" ]; then
        warn "No nodes visible"
    else
        echo "$NODES" | sed 's/^/  /'
    fi
fi

# ── 8. Summary ────────────────────────────────────────────────────────────────
section "SUMMARY"

if [ ${#ISSUES[@]} -eq 0 ]; then
    ok "No issues detected"
else
    echo -e "  ${R}Issues (${#ISSUES[@]}):${N}"
    for i in "${ISSUES[@]}"; do echo -e "    ${R}✗${N} $i"; done
fi

if [ ${#FIXES[@]} -gt 0 ]; then
    echo ""
    echo -e "  ${G}Fixes applied (${#FIXES[@]}):${N}"
    for f in "${FIXES[@]}"; do echo -e "    ${G}✓${N} $f"; done
fi

echo ""
if [ ${#ISSUES[@]} -gt 0 ] && ! $FIX; then
    info "Try auto-fix: ./check_dds.sh --fix"
fi

echo -e "\n${BOLD}══════ END OF REPORT ══════${N}"
