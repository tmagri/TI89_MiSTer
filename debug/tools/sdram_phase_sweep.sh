#!/usr/bin/env bash
# =============================================================================
# sdram_phase_sweep.sh v2 — empirical SDRAM capture-eye mapping (P3).
#
# Per phase: patch PLL -> docker compile -> deploy rbf + MGL launch ->
# poll for boot_done -> bounded RX readback (D F 0 10000) -> diff vs the
# golden synthesis -> verdict. No build-script monitor (its ssh dying
# early was desynchronizing the previous sweep flow).
#
# Usage:  sdram_phase_sweep.sh 12500 13125 14375 15000
# Results: debug/hw/sweep/summary.md (appended)
# =============================================================================
set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJ"
SWEEP_DIR="$PROJ/debug/hw/sweep"
mkdir -p "$SWEEP_DIR"

MISTER_SSH="root@192.168.1.131"
UART_DEV="/dev/ttyS1"
SSH_OPTS="-o ConnectTimeout=5 -o StrictHostKeyChecking=no -o LogLevel=ERROR -o BatchMode=yes"
DOCKER_BIN="/Applications/Docker.app/Contents/Resources/bin/docker"
QUARTUS_IMAGE="ryanfb/quartus-mister"
RBF="$PROJ/output_files/TI89.rbf"
PLL=rtl/pll/pll_0002.v

PHASES=("$@")
[ ${#PHASES[@]} -eq 0 ] && PHASES=(12500 13125 14375 15000)

SUMMARY="$SWEEP_DIR/summary.md"
touch "$SUMMARY"

# ---- helper: bounded RX readback; payload to stdout ----
rx_readback() {   # $1=hex start  $2=hex len (hex, 6 digits)
    timeout 90 ssh $SSH_OPTS "$MISTER_SSH" \
        "stty -F $UART_DEV 115200 raw -echo; \
         timeout 2 cat $UART_DEV > /dev/null; \
         (timeout 30 cat $UART_DEV > /tmp/ti89_rb) & \
         sleep 0.5; \
         printf 'D F $1 $2\r' > $UART_DEV; \
         wait; cat /tmp/ti89_rb" 2>/dev/null
}

# ---- helper: wait for boot_done via tiny readback (30 min window) ----
wait_boot() {
    for try in $(seq 1 90); do
        sleep 20
        R=$(rx_readback 000000 000010 | python3 -c "
import sys
d = sys.stdin.buffer.read()
print('YES' if b'\$D' in d and len(d) >= 30 else 'NO')")
        [ "$R" == "YES" ] && return 0
    done
    return 1
}

for PH in "${PHASES[@]}"; do
    echo "=== PHASE $PH ps  $(date '+%H:%M:%S') ==="
    sed -i '' "s/phase_shift1(\"[0-9]* ps\")/phase_shift1(\"$PH ps\")/" "$PLL"
    grep -q "phase_shift1(\"$PH ps\")" "$PLL" || { echo "patch failed"; exit 1; }

    # ---- compile (docker quartus, mirrors ti89_build_debug.sh) ----
    rm -rf debug/.buildlock
    CLOG="$SWEEP_DIR/compile_${PH}.log"
    echo "compiling (log: $CLOG)…"
    "$DOCKER_BIN" run --rm --platform linux/amd64 \
        -v "${PROJ}:/build:rw" -w /build "$QUARTUS_IMAGE" \
        bash -c "export PATH=\$PATH:/opt/intelFPGA_lite/17.0/quartus/bin:/intelFPGA_lite/17.0/quartus/bin && grep -q 'NUM_PARALLEL_PROCESSORS 1' TI89.qsf || echo 'set_global_assignment -name NUM_PARALLEL_PROCESSORS 1' >> TI89.qsf; quartus_sh --flow compile TI89.qpf" \
        > "$CLOG" 2>&1
    if [ $? -ne 0 ] || grep -aq "^Error" "$CLOG"; then
        echo "| $PH | COMPILE FAILED |" >> "$SUMMARY"
        grep -a "^Error" "$CLOG" | head -3
        continue
    fi
    [ -f "$RBF" ] || { echo "| $PH | NO RBF |" >> "$SUMMARY"; continue; }
    echo "compile done: $(ls -la "$RBF" | awk '{print $5}') bytes"

    # ---- deploy + launch ----
    timeout 60 ssh $SSH_OPTS "$MISTER_SSH" "mkdir -p /media/fat/_Computer" 2>/dev/null
    timeout 120 scp -O $SSH_OPTS "$RBF" "${MISTER_SSH}:/media/fat/_Computer/TI89.rbf" \
        || { echo "| $PH | SCP FAILED |" >> "$SUMMARY"; continue; }
    timeout 60 ssh $SSH_OPTS "$MISTER_SSH" \
        "cat > /media/fat/_Computer/auto_boot.mgl << 'EOF'
<mistergamedescription>
    <rbf>_Computer/TI89</rbf>
    <file delay=\"1\" type=\"f\" index=\"0\" path=\"../../_Computer/TI89Titanium_OS.89u\"/>
</mistergamedescription>
EOF
echo 'load_core /media/fat/_Computer/auto_boot.mgl' > /dev/MiSTer_cmd" 2>/dev/null
    echo "deployed + launched"

    # ---- wait for boot_done (tiny readback probe, 30 min window) ----
    BOOTED=0
    for try in $(seq 1 90); do
        sleep 20
        R=$(rx_readback 000000 000010 | python3 -c "
import sys
d = sys.stdin.buffer.read()
print('YES' if b'\$D' in d and len(d) >= 30 else 'NO')")
        [ "$R" == "YES" ] && { BOOTED=1; echo "boot done after ~$((try*20))s"; break; }
    done
    [ "$BOOTED" != "1" ] && { echo "| $PH | BOOT TIMEOUT |" >> "$SUMMARY"; continue; }

    # ---- bounded readback: first 64 KB of the flash window ----
    CAP="$SWEEP_DIR/rb_${PH}.bin"
    rx_readback 000000 010000 > "$CAP"

    VERDICT=$(python3 - "$CAP" "$PROJ/TI89Titanium_OS.89u" <<'PYEOF'
import sys
data = open(sys.argv[1], "rb").read()
i = data.find(b"$D")
if i < 0:
    print("VERDICT: no $D response"); sys.exit(0)
i += 4
out = bytearray()
while len(out) < 0x10000:
    if data[i:i+1] != b"@" or data[i+7:i+9] != b"\r\n":
        break
    i += 9
    take = min(8192, 0x10000 - len(out))
    out += data[i:i+take]
    i += take
sys.path.insert(0, "debug/tools")
from compare_golden import build_golden
golden, _ = build_golden(sys.argv[2])
diffs = sum(1 for a, b in zip(out, golden[:len(out)]) if a != b)
print(f"VERDICT: {diffs} mismatched bytes of {len(out)} "
      f"({100.0*diffs/max(len(out),1):.4f}%)")
PYEOF
)
    echo "$VERDICT"
    echo "$(date '+%H:%M') $PH: $VERDICT" >> "$SUMMARY"
done

# restore the reference phase in the PLL source (13750)
sed -i '' 's/\.phase_shift1("[0-9]* ps")/.phase_shift1("13750 ps")/' "$PLL"
echo "PLL source restored to 13750 ps (deployed bitstream = last swept phase)"
echo "=== done $(date '+%H:%M:%S') ==="
