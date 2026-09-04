#!/usr/bin/env bash
# session_status.sh — append a one-line status snapshot every 60 s so
# background progress is auditable without babysitting.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$SCRIPT_DIR/../.."

LOG="$PROJ_DIR/debug/session_status.log"

while true; do
    {
        echo "--- $(date '+%H:%M:%S') ---"
        # Quartus build / deploy pipeline
        if pgrep -qf quartus_sh; then
            echo "quartus: RUNNING ($(grep -ac 'Info.*Flow' /tmp/ti89_quartus_build.log 2>/dev/null || echo 0) flow lines)"
        elif pgrep -qf ti89_build_debug.sh; then
            echo "quartus: done/monitoring (script alive)"
        else
            echo "quartus: idle"
        fi
        ls -l "$PROJ_DIR/output_files/TI89.rbf" 2>/dev/null | awk '{print "rbf:", $6, $7, $8, "size", $5}'
        
        # UART capture growth (build/monitor transcripts in readback/)
        for f in "$PROJ_DIR"/debug/readback/*.log; do
            [ -f "$f" ] && echo "$(basename "$f"): $(grep -ac '\[TI89\]' "$f" 2>/dev/null) lines, last: $(grep -a '\[TI89\]' "$f" 2>/dev/null | tail -1 | cut -c1-100)"
        done
        
        # Sims (historical logs now live in ti89_sim/archive/)
        pgrep -qf Vtb_hwvid && echo "sim hwvid: RUNNING" || echo "sim hwvid: stopped"
        
        [ -f "$PROJ_DIR/debug/ti89_sim/archive/sim_hwvid.log" ] && \
            echo "sim hwvid last: $(tail -1 "$PROJ_DIR/debug/ti89_sim/archive/sim_hwvid.log" | cut -c1-120)"
            
        [ -f "$PROJ_DIR/debug/ti89_sim/archive/boot_events.log" ] && \
            echo "ai7 hits in log: $(grep -ac 'AI7 HIT' "$PROJ_DIR/debug/ti89_sim/archive/boot_events.log" 2>/dev/null)"
    } >> "$LOG" 2>&1
    sleep 60
done