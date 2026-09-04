#!/usr/bin/env bash
# =============================================================================
# ti89_build_debug.sh — TI-89 MiSTer: Compile → Deploy → UART Debug
# =============================================================================
#
# Usage:
#   ./ti89_build_debug.sh              Deploy existing .rbf + .89u, monitor UART
#   ./ti89_build_debug.sh --compile    Compile first via Docker, then deploy+monitor
#   ./ti89_build_debug.sh --no-reboot  Deploy but skip the MiSTer reboot step
#   ./ti89_build_debug.sh --monitor    SSH into MiSTer UART monitor only (skip deploy)
#   ./ti89_build_debug.sh --kill       Kill stuck Docker/Quartus processes and clear locks
#
# Requirements:
#   - Docker Desktop installed at /Applications/Docker.app
#   - SSH key-based access to root@192.168.1.131 (MiSTer)
#   - TI89Titanium_OS.89u in the project root directory
#   - output_files/TI89.rbf must exist (or use --compile to build it)
#
# UART output format (dbg_uart.sv, ~4 Hz):
#   [TI89] PC=962226 A=000004 D=1414 RD IPL=0 INT=00AC FLW=0000 L=1 P=1 S=0 7=0 ST=4
#   Fields: PC=cpu_pc A=last_addr D=last_data RD/WR IPL=interrupt_level
#           INT=intack_count FLW=flash_writes L=lcd_on P=protect S=stopped
#           7=ai7_pending ST=boot_status(0-7)
# =============================================================================

set -euo pipefail

# ─── Configuration ────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
QSF_FILE="$PROJECT_DIR/TI89.qpf"
RBF_FILE="$PROJECT_DIR/output_files/TI89.rbf"
ROM_FILE="$PROJECT_DIR/TI89Titanium_OS.89u"

MISTER_HOST="192.168.1.131"
MISTER_USER="root"
MISTER_SSH="${MISTER_USER}@${MISTER_HOST}"
MISTER_CORE_DIR="/media/fat/_Computer"
MISTER_RBF_PATH="${MISTER_CORE_DIR}/TI89.rbf"
MISTER_ROM_PATH="${MISTER_CORE_DIR}/TI89Titanium_OS.89u"

DOCKER_BIN="/Applications/Docker.app/Contents/Resources/bin/docker"
QUARTUS_IMAGE="ryanfb/quartus-mister"
UART_DEV="/dev/ttyS1"
UART_BAUD="115200"

SSH_OPTS="-o ConnectTimeout=5 -o StrictHostKeyChecking=no -o LogLevel=ERROR -o BatchMode=yes"
SSH_RETRY_TIMEOUT=120   # seconds to wait for MiSTer to come back after reboot
SSH_RETRY_INTERVAL=5

# ─── Compile-phase lock ───────────────────────────────────────────────────────
# Only the Docker/Quartus compile phase must never double-run (concurrent
# instances corrupt TI89.qsf / incremental_db -> errors 125085 + 293007).
# Deploy-only and monitor-only invocations NEVER block; a --compile run
# waits only while another compile is actually in flight.
BUILD_LOCK="${PROJECT_DIR}/debug/.buildlock"

build_lock_wait() {
    if [ -d "$BUILD_LOCK" ]; then
        local owner
        owner=$(cat "$BUILD_LOCK/pid" 2>/dev/null || echo "?")
        if [ -n "$owner" ] && [ "$owner" != "$$" ] && kill -0 "$owner" 2>/dev/null; then
            warn "Another compile is running (pid $owner) — waiting for it to finish"
            while [ -d "$BUILD_LOCK" ] && kill -0 "$owner" 2>/dev/null; do
                sleep 10
            done
        else
            rm -rf "$BUILD_LOCK"   # stale lock from a dead instance
        fi
    fi
    # Belt and braces: an in-flight quartus compile with no lock
    # (e.g. started by an older script version) also blocks us.
    while pgrep -f "quartus_sh --flow compile" >/dev/null 2>&1; do
        warn "quartus_sh compile in flight (no lock) — waiting"
        sleep 10
    done
}

build_lock_acquire() {
    mkdir -p "$(dirname "$BUILD_LOCK")"
    if ! mkdir "$BUILD_LOCK" 2>/dev/null; then
        build_lock_wait
        mkdir "$BUILD_LOCK" 2>/dev/null || fail "cannot acquire build lock"
    fi
    echo "$$" > "$BUILD_LOCK/pid"
}

build_lock_release() {
    [ -d "$BUILD_LOCK" ] && [ "$(cat "$BUILD_LOCK/pid" 2>/dev/null)" = "$$" ] && rm -rf "$BUILD_LOCK"
}

# ─── Flags ────────────────────────────────────────────────────────────────────
DO_COMPILE=false
DO_REBOOT=true
MONITOR_ONLY=false
ROM_MISSING=false
DO_KILL=false

for arg in "$@"; do
    case "$arg" in
        --compile)      DO_COMPILE=true ;;
        --no-reboot)    DO_REBOOT=false ;;
        --monitor|--monitor-only) MONITOR_ONLY=true ;;
        --kill)         DO_KILL=true ;;
        --help|-h)
            sed -n '4,19p' "$0" | sed 's/^# //' | sed 's/^#//'
            exit 0
            ;;
        *)
            echo "Unknown flag: $arg  (try --help)" >&2
            exit 1
            ;;
    esac
done

# ─── ANSI colours ─────────────────────────────────────────────────────────────
RED=$'\e[1;31m';  GRN=$'\e[1;32m';  YLW=$'\e[1;33m'
BLU=$'\e[1;34m';  CYN=$'\e[1;36m';  RST=$'\e[0m'
BOLD=$'\e[1m'

log()  { echo "${BLU}▶${RST} ${BOLD}$*${RST}"; }
ok()   { echo "${GRN}✔${RST} $*"; }
warn() { echo "${YLW}⚠${RST}  $*"; }
fail() { echo "${RED}✘${RST} $*" >&2; exit 1; }
step() { echo; echo "${CYN}━━━ $* ━━━${RST}"; }

# ─── Spinner ──────────────────────────────────────────────────────────────────
spinner_pid=""
spinner_start() {
    local msg="$1"
    (
        local frames=("⠋" "⠙" "⠹" "⠸" "⠼" "⠴" "⠦" "⠧" "⠇" "⠏")
        local i=0
        while true; do
            printf "\r${YLW}%s${RST} %s  " "${frames[$((i % 10))]}" "$msg"
            ((i++)) || true
            sleep 0.1
        done
    ) &
    spinner_pid=$!
}
spinner_stop() {
    if [[ -n "$spinner_pid" ]]; then
        kill "$spinner_pid" 2>/dev/null || true
        wait "$spinner_pid" 2>/dev/null || true
        spinner_pid=""
        printf "\r\033[K"   # clear spinner line
    fi
}
trap spinner_stop EXIT

# =============================================================================
# PRE-PHASE: Kill stuck processes (if --kill is passed)
# =============================================================================
if $DO_KILL; then
    step "Killing stuck compile processes and clearing locks"
    
    log "Stopping lingering ryanfb/quartus-mister Docker containers..."
    if [[ -x "$DOCKER_BIN" ]]; then
        CONTAINERS=$("$DOCKER_BIN" ps -q --filter ancestor="$QUARTUS_IMAGE" 2>/dev/null || echo "")
        if [[ -n "$CONTAINERS" ]]; then
            "$DOCKER_BIN" kill $CONTAINERS >/dev/null 2>&1 || true
            ok "Containers stopped."
        else
            ok "No lingering containers found."
        fi
    fi

    log "Force-quitting background quartus_sh processes..."
    pkill -f "quartus_sh" 2>/dev/null || true
    ok "Background processes terminated."

    log "Removing stale build locks..."
    rm -rf "$BUILD_LOCK"
    ok "Build locks cleared."
    
    echo
    ok "Cleanup complete. You can now run with --compile again."
    exit 0
fi

# =============================================================================
# PHASE 0: Pre-flight checks
# =============================================================================
step "Pre-flight checks"

# Check project directory
[[ -f "$QSF_FILE" ]] || fail "Not in project root — TI89.qpf not found at $QSF_FILE"
log "Project: $PROJECT_DIR"

if ! $MONITOR_ONLY; then
    # Check .89u ROM
    if [[ ! -f "$ROM_FILE" ]]; then
        warn ".89u ROM not found at $ROM_FILE"
        warn "It will NOT be copied to MiSTer. Load it manually via OSD if needed."
        ROM_MISSING=true
    else
        ok "ROM found: $(basename "$ROM_FILE") ($(du -h "$ROM_FILE" | cut -f1))"
    fi

    # Check .rbf (only needed if not compiling)
    if ! $DO_COMPILE; then
        [[ -f "$RBF_FILE" ]] || fail ".rbf not found at $RBF_FILE — run with --compile to build it"
        ok "RBF found: output_files/TI89.rbf ($(du -h "$RBF_FILE" | cut -f1), $(date -r "$RBF_FILE" '+%Y-%m-%d %H:%M'))"
    fi

    # Check Docker (only if compiling)
    if $DO_COMPILE; then
        [[ -x "$DOCKER_BIN" ]] || fail "Docker binary not found at $DOCKER_BIN — is Docker Desktop installed?"

        log "Checking Docker Desktop..."
        if ! "$DOCKER_BIN" info &>/dev/null; then
            warn "Docker Desktop not running — attempting to start..."
            open -a Docker
            spinner_start "Waiting for Docker Desktop to start"
            deadline=$((SECONDS + 60))
            while ! "$DOCKER_BIN" info &>/dev/null; do
                if [[ $SECONDS -ge $deadline ]]; then
                    spinner_stop
                    fail "Docker Desktop did not start within 60 s — open it manually and retry"
                fi
                sleep 2
            done
            spinner_stop
        fi
        ok "Docker is running"

        # Check image available (pull if needed)
        if ! "$DOCKER_BIN" image inspect "$QUARTUS_IMAGE" &>/dev/null; then
            log "Pulling $QUARTUS_IMAGE (first-time pull, may take a while)..."
            "$DOCKER_BIN" pull "$QUARTUS_IMAGE" || fail "Failed to pull $QUARTUS_IMAGE"
        fi
        ok "Docker image: $QUARTUS_IMAGE"
    fi
fi

# Check SSH reachability
log "Checking MiSTer SSH at $MISTER_HOST..."
if ! ssh $SSH_OPTS "$MISTER_SSH" "true" 2>/dev/null; then
    if $MONITOR_ONLY; then
        fail "Cannot reach MiSTer at $MISTER_HOST — is it powered on and connected?"
    else
        warn "MiSTer not currently reachable — will retry after deploy/reboot"
    fi
else
    MISTER_KERNEL=$(ssh $SSH_OPTS "$MISTER_SSH" "uname -r" 2>/dev/null || echo "unknown")
    ok "MiSTer SSH reachable (kernel: $MISTER_KERNEL)"
fi

# =============================================================================
# PHASE 1: Compile (optional)
# =============================================================================
if $DO_COMPILE && ! $MONITOR_ONLY; then
    step "Phase 1 — Quartus Compilation (Docker)"
    log "Container : $QUARTUS_IMAGE"
    log "Project   : $QSF_FILE"
    log "This typically takes 20–60 minutes."
    echo

    COMPILE_LOG="/tmp/ti89_quartus_build.log"

    build_lock_wait
    build_lock_acquire
    trap build_lock_release EXIT INT TERM

    spinner_start "Compiling TI89 core in Docker"

    # Run Quartus full compilation inside the container.
    # The QSF edit is idempotent (never appends a duplicate line: the
    # 2026-08-31 double-run crash left duplicated assignments and
    # Quartus rewrote the file mid-compile).
    set +e
    "$DOCKER_BIN" run --rm \
        --platform linux/amd64 \
        -v "${PROJECT_DIR}:/build:rw" \
        -w /build \
        "$QUARTUS_IMAGE" \
        bash -c "export PATH=\$PATH:/opt/intelFPGA_lite/17.0/quartus/bin:/intelFPGA_lite/17.0/quartus/bin && grep -q 'NUM_PARALLEL_PROCESSORS 1' TI89.qsf || echo 'set_global_assignment -name NUM_PARALLEL_PROCESSORS 1' >> TI89.qsf; quartus_sh --flow compile TI89.qpf" \
        > "$COMPILE_LOG" 2>&1
    COMPILE_EXIT=$?
    set -e

    spinner_stop

    # Show interesting lines from log
    grep -E "^(Error|Warning.*critical|Info.*(Timing|Fitter|Assembler|Flow|Success))" "$COMPILE_LOG" || true

    if [[ $COMPILE_EXIT -ne 0 ]] || grep -q "^Error" "$COMPILE_LOG"; then
        echo
        echo "${RED}─── Compilation Errors ───${RST}"
        grep "^Error" "$COMPILE_LOG" || true
        fail "Quartus compilation failed — full log at $COMPILE_LOG"
    fi

    [[ -f "$RBF_FILE" ]] || fail "Compilation finished but $RBF_FILE was not produced"
    ok "Compiled successfully → output_files/TI89.rbf ($(du -h "$RBF_FILE" | cut -f1))"
else
    if ! $MONITOR_ONLY; then
        step "Phase 1 — Compile (skipped; using existing .rbf)"
        log "Pass --compile to rebuild from RTL sources"
    fi
fi

# =============================================================================
# PHASE 2: Deploy to MiSTer
# =============================================================================
if ! $MONITOR_ONLY; then
    step "Phase 2 — Deploy to MiSTer ($MISTER_HOST)"

    # Ensure _Computer directory exists on MiSTer
    ssh $SSH_OPTS "$MISTER_SSH" "mkdir -p ${MISTER_CORE_DIR}" \
        || fail "Cannot create $MISTER_CORE_DIR on MiSTer"

    # Copy .rbf
    log "Copying TI89.rbf → ${MISTER_SSH}:${MISTER_RBF_PATH}"
    scp -O $SSH_OPTS "$RBF_FILE" "${MISTER_SSH}:${MISTER_RBF_PATH}" \
        || fail "SCP of .rbf failed"
    ok "TI89.rbf deployed ($(du -h "$RBF_FILE" | cut -f1))"

    # Copy .89u ROM (if present)
    if [[ "$ROM_MISSING" == "false" ]]; then
        log "Copying $(basename "$ROM_FILE") → ${MISTER_SSH}:${MISTER_ROM_PATH}"
        scp -O $SSH_OPTS "$ROM_FILE" "${MISTER_SSH}:${MISTER_ROM_PATH}" \
            || fail "SCP of .89u failed"
        ok "$(basename "$ROM_FILE") deployed"
    fi

    # Verify files on MiSTer
    log "Verifying files on MiSTer..."
    ssh $SSH_OPTS "$MISTER_SSH" "ls -lh ${MISTER_CORE_DIR}/ | grep -E 'TI89'" || true

    # Sync filesystem
    ssh $SSH_OPTS "$MISTER_SSH" "sync" 2>/dev/null || true

    # Generate auto-boot MGL file
    log "Generating auto-boot MGL file..."
    ssh $SSH_OPTS "$MISTER_SSH" "cat << 'EOF' > ${MISTER_CORE_DIR}/auto_boot.mgl
<mistergamedescription>
    <rbf>_Computer/TI89</rbf>
    <file delay=\"1\" type=\"f\" index=\"0\" path=\"../../_Computer/TI89Titanium_OS.89u\"/>
</mistergamedescription>
EOF"

    # Execute the MGL to launch the core instantly (no reboot required)
    log "Launching TI-89 core and ROM..."
    ssh $SSH_OPTS "$MISTER_SSH" "echo 'load_core ${MISTER_CORE_DIR}/auto_boot.mgl' > /dev/MiSTer_cmd"
    
    # Give the core a second to initialize before starting the UART monitor
    sleep 2
fi

# =============================================================================
# PHASE 3: Monitor UART
# =============================================================================
step "Phase 3 — Live UART Debug Monitor (/dev/ttyS1 @ ${UART_BAUD})"
echo
echo "${BOLD}UART output format (dbg_uart.sv, ~4 Hz):${RST}"
echo "  ${CYN}[TI89] PC=<cpu_pc> A=<addr> D=<data> RD/WR IPL=<n> INT=<intack> FLW=<flash_wr> L=<lcd> P=<prot> S=<stop> 7=<ai7> ST=<boot_status>${RST}"
echo
echo "Healthy HOME screen signs:"
echo "  ${GRN}✔${RST} L=1   (LCD on)"
echo "  ${GRN}✔${RST} INT   incrementing each line (interrupt service healthy)"
echo "  ${GRN}✔${RST} PC    near 962226 (idle home-screen node-walk loop)"
echo "  ${GRN}✔${RST} P=1   (hwprot armed — flash init complete)"
echo "  ${GRN}✔${RST} ST=4  (boot complete)"
echo
echo "Boot in progress signs:"
echo "  ${YLW}⚠${RST}  ST=0..3  (boot FSM still running)"
echo "  ${YLW}⚠${RST}  L=0      (LCD not yet enabled)"
echo "  ${YLW}⚠${RST}  FLW      climbing rapidly (flash init pass)"
echo
echo "${YLW}Press Ctrl+C to stop monitoring.${RST}"
echo
log "SSH → ${MISTER_SSH}: stty -F ${UART_DEV} ${UART_BAUD} raw -echo && cat ${UART_DEV}"
echo "──────────────────────────────────────────────────────────────────────"

# Run UART monitor — streams until Ctrl+C
# Use -t for PTY (needed for stty), trap Ctrl+C cleanly
set +e
ssh $SSH_OPTS -t "$MISTER_SSH" \
    "stty -F ${UART_DEV} ${UART_BAUD} raw -echo && cat ${UART_DEV}"
SSH_EXIT=$?
set -e

echo
echo "──────────────────────────────────────────────────────────────────────"
if [[ $SSH_EXIT -eq 130 ]] || [[ $SSH_EXIT -eq 0 ]]; then
    ok "Monitor session ended."
else
    warn "SSH exited with code $SSH_EXIT"
fi