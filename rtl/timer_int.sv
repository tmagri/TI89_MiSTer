//
// timer_int.sv — Timer and interrupt controller
// TI-89 MiSTer Core
//
// Implements the TI-89 HW2+ auto-interrupt sources, following the
// reference simulator's timer model (v12.js timer_interrupts):
//
//   AI1: fixed rate — every 2048 OSC2 units (64 base ticks, ~256 Hz)
//        gated by OSC2 enable ($600015 bit 1) and master disable (bit 7)
//   AI2: keyboard scan (set by keyboard controller, acked by writing $60001B)
//   AI3: every 524288 OSC2 units (16384 base ticks, 1 Hz)
//        gated by bit 2, master disable and (on HW3) the OSC2 enable bit
//   AI4: link port (not implemented — no link cable)
//   AI5: programmable timer ($600017) — counts UP on prescaled ticks;
//        a tick with current value 0 reloads $600017, otherwise the value
//        increments; AI5 is raised whenever the value is 0 just after the
//        increment stage — i.e. on the 255->0 wrap, and on every tick when
//        the reload value is 0 (matches TiEmu hw_update exactly)
//   AI6: ON key press (set by keyboard controller, acked by writing $60001A)
//   AI7: protection violation (not implemented)
//
// Timing: OSC2 = 2^19 Hz. The base tick is OSC2/2^5 = 16384 Hz, derived
// from the 64 MHz master clock (64e6 / 3906 = 16385 Hz, 0.006% fast).
// One base tick corresponds to 32 OSC2 counter units, so:
//   AI1 fires when base counter [5:0] == 0   (every 64 ticks)
//   AI3 fires when base counter [13:0] == 0  (every 16384 ticks)
// and the programmable timer prescaler ($600015 [5:4]) divides the base
// tick by 1 / 16 / 128 / 8192.
//
// AI1/3/5 pending flags are cleared by the CPU's interrupt acknowledge
// (autovector IACK cycle); AI2/AI6 are cleared by the register writes
// above (and also by IACK, harmlessly).
//

module timer_int (
    input         clk,
    input         reset,

    // Timer control from I/O ports
    input   [7:0] timer_ctrl,    // $600015
    input   [7:0] timer_init,    // $600017 written value (reload value)
    output reg [7:0] timer_value, // Current timer value (read by CPU)

    // Keyboard interrupt sources
    input         kbd_int,       // Key state change detected (sets AI2)
    input         on_key_press,  // ON key pressed edge (sets AI6)

    // Interrupt acknowledgements from I/O port writes
    input         ack_ai2,       // Write to $60001B
    input         ack_ai6,       // Write to $60001A

    // CPU interrupt acknowledge (IACK bus cycle, from TI89.sv)
    input         intack,

    // Interrupt output to CPU (active-high level, 0 = none)
    output reg [2:0] ipl,

    // Individual pending flags (bit N = auto-interrupt N pending).
    // Used by the top level for the $600005 STOP wake-up logic.
    output [7:0] int_pend
);

    // =========================================================================
    // Base tick generator: 64 MHz / 3906 = 16385 Hz (nominal 16384 Hz)
    // =========================================================================

    localparam [15:0] BASE_DIVISOR = 16'd3906;

    reg [15:0] base_counter;
    reg        base_tick;

    always @(posedge clk) begin
        if (reset) begin
            base_counter <= 16'd0;
            base_tick    <= 1'b0;
        end else begin
            base_tick <= 1'b0;
            if (base_counter >= BASE_DIVISOR - 16'd1) begin
                base_counter <= 16'd0;
                base_tick    <= 1'b1;
            end else begin
                base_counter <= base_counter + 16'd1;
            end
        end
    end

    // =========================================================================
    // Control bits ($600015)
    // =========================================================================

    wire master_en = ~timer_ctrl[7];  // bit 7: master disable (0 = enabled)
    wire timer_en  = timer_ctrl[3];   // programmable timer enable
    wire ai3_en    = timer_ctrl[2];   // AI3 enable
    wire osc2_en   = timer_ctrl[1];   // OSC2 enable (gates AI1 and AI5)

    // Prescaler divide ratio (in base ticks): 1 / 16 / 128 / 8192
    reg [12:0] prescaler_mask;
    always @(*) begin
        case (timer_ctrl[5:4])
            2'b00:    prescaler_mask = 13'd0;     // every base tick
            2'b01:    prescaler_mask = 13'd15;    // every 16 base ticks
            2'b10:    prescaler_mask = 13'd127;   // every 128 base ticks
            default:  prescaler_mask = 13'd8191;  // every 8192 base ticks
        endcase
    end

    // =========================================================================
    // Free-running OSC2-derived counter (counts base ticks, wraps at 2^19)
    // =========================================================================

    reg [18:0] timer;

    wire prescale_tick = base_tick && ((timer[12:0] & prescaler_mask) == 13'd0);

    // =========================================================================
    // Interrupt pending flags
    // =========================================================================

    reg ai1_pending, ai2_pending, ai3_pending;
    reg ai5_pending, ai6_pending;

    assign int_pend = {1'b0, ai6_pending, ai5_pending, 1'b0,
                       ai3_pending, ai2_pending, ai1_pending, 1'b0};

    always @(posedge clk) begin
        if (reset) begin
            timer        <= 19'd0;
            timer_value  <= 8'd0;
            ai1_pending  <= 1'b0;
            ai2_pending  <= 1'b0;
            ai3_pending  <= 1'b0;
            ai5_pending  <= 1'b0;
            ai6_pending  <= 1'b0;
        end else begin

            // -----------------------------------------------------------------
            // Flag setting
            // -----------------------------------------------------------------

            // ON key (AI6) and keyboard (AI2) — not gated by master disable
            if (on_key_press) ai6_pending <= 1'b1;
            if (kbd_int)      ai2_pending <= 1'b1;

            if (base_tick) begin
                timer <= timer + 19'd1;

                if (master_en) begin
                    // AI1: every 64 base ticks, only while OSC2 is enabled
                    if (osc2_en && (timer[5:0] == 6'd0))
                        ai1_pending <= 1'b1;

                    // AI3: every 16384 base ticks. The reference gates it
                    // on OSC2 enable everywhere except HW2; this core is
                    // Titanium (HW3), so require osc2_en.
                    if (ai3_en && osc2_en && (timer[13:0] == 14'd0))
                        ai3_pending <= 1'b1;
                end
            end

            // Programmable timer (AI5) — counts UP on prescaled ticks.
            // Reference (TiEmu hw_update): the increment stage runs first
            // (0 -> reload from $600017, else ++, wrapping FF -> 00), then
            // AI5 is raised whenever the value is 0 at that moment. That
            // means AI5 fires on the FF->00 wrap AND every tick when the
            // reload value itself is 0.
            if (prescale_tick && master_en && osc2_en && timer_en) begin
                if (timer_value == 8'd0) begin
                    timer_value <= timer_init;          // reload
                    if (timer_init == 8'd0)
                        ai5_pending <= 1'b1;            // value stays 0
                end else if (timer_value == 8'hFF) begin
                    timer_value <= 8'd0;                // overflow
                    ai5_pending <= 1'b1;
                end else begin
                    timer_value <= timer_value + 8'd1;
                end
            end

            // -----------------------------------------------------------------
            // Flag clearing
            // -----------------------------------------------------------------

            // Register-write acknowledges
            if (ack_ai2) ai2_pending <= 1'b0;
            if (ack_ai6) ai6_pending <= 1'b0;

            // CPU interrupt acknowledge: clear whichever flag is driving the
            // current IPL (the CPU has accepted that level).
            if (intack) begin
                case (ipl)
                    3'd1: ai1_pending <= 1'b0;
                    3'd2: ai2_pending <= 1'b0;
                    3'd3: ai3_pending <= 1'b0;
                    3'd5: ai5_pending <= 1'b0;
                    3'd6: ai6_pending <= 1'b0;
                    default: ;
                endcase
            end
        end
    end

    // =========================================================================
    // Priority encoder: highest pending interrupt -> IPL output
    // =========================================================================
    // 68000 priority: 7 > 6 > 5 > 4 > 3 > 2 > 1 > 0 (none)

    always @(*) begin
        if      (ai6_pending) ipl = 3'd6;
        else if (ai5_pending) ipl = 3'd5;
        // AI4 (link port) not implemented
        else if (ai3_pending) ipl = 3'd3;
        else if (ai2_pending) ipl = 3'd2;
        else if (ai1_pending) ipl = 3'd1;
        else                  ipl = 3'd0;
    end

endmodule
