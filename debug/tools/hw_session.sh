#!/usr/bin/env bash
# =============================================================================
# hw_session.sh — bounded hardware capture wrapper (Phase 0 protocol)
# TI-89 MiSTer debug plan. Every invocation:
#   * tees its output to debug/hw/<name>_<YYYYmmdd_HHMMSS>.log
#   * writes <name>_<ts>.manifest.json (sha256, git state, command, phase)
#
# Usage:
#   hw_session.sh monitor                        live UART monitor (Ctrl-C to stop)
#   hw_session.sh send '<text>'                  send one line to the core
#   hw_session.sh dump <F|R> <hexstart> <hexlen> [outfile]
#        Bounded range dump. F = flash window (chip offset), R = RAM.
#        hexstart/hexlen are hex WITHOUT $ (e.g. 4C00 1000). Requires the
#        dbg_uart RX command interface (Phase 2 RTL). Captures exactly the
#        framed response for the requested byte count — never unbounded.
#   hw_session.sh raw '<ssh remote command>'     arbitrary remote command, tee'd
#
# Conventions match ti89_build_debug.sh: root@192.168.1.131, /dev/ttyS1 @115200.
# Set PHASE env (e.g. PHASE=P2) to tag manifests.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/../.." && pwd)"
HW_DIR="$PROJ/debug/hw"
MISTER_SSH="root@192.168.1.131"
UART_DEV="/dev/ttyS1"
UART_BAUD="115200"
SSH_OPTS="-o ConnectTimeout=5 -o StrictHostKeyChecking=no -o LogLevel=ERROR -o BatchMode=yes"

mkdir -p "$HW_DIR"
TS="$(date '+%Y%m%d_%H%M%S')"
PHASE="${PHASE:-?}"

usage() { sed -n '3,22p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
[ $# -ge 1 ] || usage

write_manifest() {  # $1=name $2=logfile $3..=extra key=value
    local name="$1" log="$2"; shift 2
    local m="$HW_DIR/${name}_${TS}.manifest.json"
    {
        echo "{"
        echo "  \"date\": \"$(date -u '+%Y-%m-%dT%H:%M:%SZ')\","
        echo "  \"phase\": \"$PHASE\","
        echo "  \"command\": \"$*\","
        echo "  \"git\": \"$(git -C "$PROJ" describe --always --dirty 2>/dev/null)\","
        echo "  \"dirty_files\": [$(git -C "$PROJ" diff --name-only 2>/dev/null | sed 's/.*/"&"/' | paste -sd, -)],"
        echo "  \"log\": \"$(basename "$log")\","
        echo "  \"sha256\": \"$(shasum -a 256 "$log" | cut -d' ' -f1)\""
        echo "}"
    } > "$m"
    echo "manifest: $m" >&2
}

case "$1" in
    monitor)
        shift
        LOG="$HW_DIR/monitor_${TS}.log"
        echo "monitoring ${UART_DEV} @${UART_BAUD} -> $LOG (Ctrl-C to stop)"
        ssh $SSH_OPTS -t "$MISTER_SSH" \
            "stty -F $UART_DEV $UART_BAUD raw -echo && cat $UART_DEV" 2>&1 \
            | tee "$LOG"
        write_manifest monitor "$LOG" monitor
        ;;
    send)
        [ $# -ge 2 ] || usage
        LOG="$HW_DIR/send_${TS}.log"
        printf '%s\r' "$2" > /tmp/ti89_uart_cmd_$$
        ssh $SSH_OPTS "$MISTER_SSH" \
            "stty -F $UART_DEV $UART_BAUD raw -echo; cat > $UART_DEV" \
            < /tmp/ti89_uart_cmd_$$ 2>&1 | tee "$LOG"
        rm -f /tmp/ti89_uart_cmd_$$
        write_manifest send "$LOG" "send:$2"
        ;;
    dump)
        # dump <F|R> <hexstart> <hexlen> [outfile]
        [ $# -ge 4 ] || usage
        MEM="$2"; START="$3"; LEN="$4"
        OUT="${5:-$HW_DIR/dump_${MEM}_${START}_${LEN}_${TS}.bin}"
        LOG="${OUT%.bin}.log"
        # Framing: response = "$D\r\n" marker line, then per 4096-word block
        # a 9-byte "@xxxxxx\r\n" sync + 8192 raw bytes (same block framing as
        # the pre-boot dump). Expected = 4 + 9*ceil(words/4096) + LEN(even);
        # +16 slack covers any interleaved status characters.
        WORDS=$(( (0x$LEN + 1) / 2 ))
        BLOCKS=$(( (WORDS + 4095) / 4096 ))
        EXPECT=$(( 0x$LEN + 9 * BLOCKS + 20 ))
        S6=$(printf '%06X' $((16#$START)))
        L6=$(printf '%06X' $((16#$LEN)))
        DUR=$(( (EXPECT / 11520) + 4 ))
        echo "dump MEM=$MEM start=0x$S6 len=0x$L6 -> $OUT (expect ~$EXPECT bytes, ${DUR}s)"
        # Reader attaches FIRST: the "$D" marker comes ~350 us after the
        # command, so a reader started after `printf` loses it and shifts
        # the whole capture. Remote tmp file -> host on completion.
        ssh $SSH_OPTS "$MISTER_SSH" \
            "stty -F $UART_DEV $UART_BAUD raw -echo; \
             rm -f /tmp/ti89_cap; \
             (timeout $DUR cat $UART_DEV > /tmp/ti89_cap) & \
             sleep 0.5; \
             printf 'D $MEM $S6 $L6\r\n' > $UART_DEV; \
             wait; \
             cat /tmp/ti89_cap; rm -f /tmp/ti89_cap" > "$OUT" 2> "$LOG"
        write_manifest dump "$OUT" "dump:$MEM \$$(printf %06s $START) len \$$(printf %06s $LEN)"
        echo "captured: $OUT ($(wc -c < "$OUT") bytes)"
        ;;
    wrtest)
        # wrtest <R> <hexstart> <hexlen> : pattern-write integrity loop.
        # Sends the W command (counting-word pattern write), then a D
        # readback of the same range, then compares against the expected
        # pattern. Catches SDRAM write-path corruption on hardware.
        [ $# -ge 4 ] || usage
        MEM="$2"; START="$3"; LEN="$4"
        [ "$MEM" == "R" ] || { echo "wrtest supports R (calc RAM) only"; exit 1; }
        S6=$(printf '%06X' $((16#$START)))
        L6=$(printf '%06X' $((16#$LEN)))
        DUR=$(( (0x$LEN / 11520) + 8 ))
        CAP="$HW_DIR/wrtest_${MEM}_${START}_${LEN}_${TS}.bin"
        echo "wrtest: W R $S6 $L6 then D R $S6 $L6 (window ${DUR}s)"
        timeout $((DUR + 20)) ssh $SSH_OPTS "$MISTER_SSH" \
            "stty -F $UART_DEV $UART_BAUD raw -echo; \
             timeout 2 cat $UART_DEV > /dev/null; \
             (timeout $DUR cat $UART_DEV > /tmp/ti89_wt) & \
             sleep 0.5; \
             printf 'W $MEM $S6 $L6\r' > $UART_DEV; \
             sleep 3; \
             printf 'D $MEM $S6 $L6\r' > $UART_DEV; \
             wait; cat /tmp/ti89_wt" > "$CAP" 2> /dev/null
        write_manifest wrtest "$CAP" "wrtest:$MEM \$$(printf %06s $START) len \$$(printf %06s $LEN)"
        python3 - "$CAP" "$START" "$LEN" <<'PYEOF'
import sys
data = open(sys.argv[1], "rb").read()
i = data.find(b"$D")
if i < 0:
    print("WRTEST: no $D response captured"); sys.exit(1)
i += 4
start_w = int(sys.argv[2], 16) // 2
n_words = int(sys.argv[3], 16) // 2
out = bytearray()
while len(out) < n_words * 2:
    if data[i:i+1] != b"@" or data[i+7:i+9] != b"\r\n": break
    i += 9
    take = min(8192, n_words*2 - len(out))
    if take <= 0: break
    out += data[i:i+take]; i += take
mism = 0; first = None; bit_hist = {}
for k in range(n_words):
    exp = (start_w + k) & 0xFFFF
    got = (out[2*k] << 8) | out[2*k+1] if 2*k+1 < len(out) else None
    if got != exp:
        mism += 1
        if first is None: first = (start_w + k) * 2
        d = exp ^ (got if got is not None else 0xFFFF)
        for bit in range(16):
            if d & (1 << bit): bit_hist[bit] = bit_hist.get(bit, 0) + 1
print(f"WRTEST: {mism} of {n_words} words mismatched")
if mism:
    print(f"  first mismatch at word {first // 2} (byte ${first * 2:06X})")
    print(f"  bit histogram (bit: count): {dict(sorted(bit_hist.items(), reverse=True))}")
    sys.exit(1)
print("  RAM write+readback path CLEAN")
PYEOF
        ;;
    *)
        usage
        ;;
esac
