//
// sdram.sv — SDRAM controller for the TI-89 core
// TI-89 MiSTer Core
//
// Backs the calculator's memories with the DE10-Nano's 32MB SDRAM
// (IS42S16160G, 16-bit, 4 banks x 8192 rows x 512 columns). Clock: 64 MHz.
//
// Memory layout (set by mem_ctrl, which owns port A):
//   byte $000000-$3FFFFF : OS image / flash window (4 MB)
//   byte $400000-$43FFFF : calculator RAM (256 KB)
//
// Two request ports:
//
//   Port A — CPU-side access (from mem_ctrl's arbiter, which multiplexes
//     flash_ctrl and the RAM clients), latched request:
//     a_rd / a_wr strobes latch address/data/byte-lanes; the controller
//     services one request at a time and pulses a_ready on completion
//     (a_rdata valid for reads). Requesters must wait for a_ready
//     before strobing again.
//
//   Port B — OS image download writes (from rom_loader), fire-and-forget:
//     writes are buffered in a small FIFO; b_wait goes high when the
//     FIFO nears full (wired to hps_io's ioctl_wait in TI89.sv).
//
// Port A has priority; port B traffic only exists while the CPU is held
// in reset, so in practice the ports never contend.
//
// Address mapping (word address w = byte_addr[24:1], full 32 MB):
//   bank = w[23:22], row = w[21:9] (13 bits), column = w[8:0] (9 bits)
//
// Every access is a full row cycle: ACTIVATE -> READ/WRITE -> explicit
// PRECHARGE -> tRP, so back-to-back accesses never violate precharge or
// write-recovery timing. Refresh runs every ~14 us (well inside the
// 15.6 us per-row budget), deferred behind an access in progress.
//
// DQM is held high (masked) during init and idle; reads use DQM = 0,
// writes use DQM = byte-lane enables so the flash WSM's per-byte
// programming works.
//

module sdram (
    input         clk,         // 64 MHz master clock
    input         reset,

    // SDRAM physical interface
    output        SDRAM_CLK,
    output        SDRAM_CKE,
    output [12:0] SDRAM_A,
    output  [1:0] SDRAM_BA,
    // DQ split into in/out/OE so a sim chip model and the top level can
    // share the bidirectional line with a single driver each.
    input  [15:0] SDRAM_DQ_IN,
    output [15:0] SDRAM_DQ_OUT,
    output        SDRAM_DQ_OE,
    output        SDRAM_DQML,
    output        SDRAM_DQMH,
    output        SDRAM_nCS,
    output        SDRAM_nCAS,
    output        SDRAM_nRAS,
    output        SDRAM_nWE,

    // Port A: CPU / flash controller access (byte address, word aligned)
    input  [24:0] a_addr,
    input  [15:0] a_wdata,
    input         a_rd,        // One-cycle read strobe
    input         a_wr,        // One-cycle write strobe
    input         a_uds_n,     // Byte lanes (writes)
    input         a_lds_n,
    output reg [15:0] a_rdata,
    output reg    a_ready,     // One-cycle completion pulse

    // Port B: OS loader writes (word address)
    input  [20:0] b_addr,
    input  [15:0] b_wdata,
    input         b_wr,
    output        b_wait,      // FIFO almost full

    output reg    init_done    // SDRAM initialized, accesses allowed
);

    // =========================================================================
    // Timing (64 MHz, one cycle = 15.6 ns; IS42S16320D-7TL)
    // =========================================================================

    localparam [16:0] PWRUP_WAIT = 17'd20000; // ~310 us power stabilization
    localparam [15:0] REF_PERIOD = 16'd480;   // ~7.5 us between refreshes
                                              // (8192 rows must refresh in 64 ms)
    localparam  [3:0] INIT_REFS  = 4'd8;      // refreshes during init

    // SDRAM commands {nCS, nRAS, nCAS, nWE}
    localparam [3:0] CMD_NOP  = 4'b0111;
    localparam [3:0] CMD_ACT  = 4'b0011;
    localparam [3:0] CMD_RD   = 4'b0101;
    localparam [3:0] CMD_WR   = 4'b0100;
    localparam [3:0] CMD_PRE  = 4'b0010;
    localparam [3:0] CMD_REF  = 4'b0001;
    localparam [3:0] CMD_LMR  = 4'b0000;

    // Mode register: burst length 1, sequential, CAS latency 2
    localparam [12:0] MODE_REG = 13'b000_0_00_010_0_000;

    // =========================================================================
    // FSM states
    // =========================================================================

    localparam [4:0] S_RESET  = 5'd0;  // Power-up wait
    localparam [4:0] S_IPRE   = 5'd1;  // Init: precharge all banks
    localparam [4:0] S_IPREW  = 5'd2;  // Init: tRP wait
    localparam [4:0] S_IREF   = 5'd3;  // Init: issue refresh
    localparam [4:0] S_REFW   = 5'd4;  // Init: refresh tRC wait
    localparam [4:0] S_ILMR   = 5'd5;  // Init: load mode register
    localparam [4:0] S_ILMRW  = 5'd6;  // Init: tMRD wait
    localparam [4:0] S_IDLE   = 5'd7;
    localparam [4:0] S_REF    = 5'd8;  // Runtime refresh wait
    localparam [4:0] S_ACT    = 5'd9;  // tRCD wait
    localparam [4:0] S_RCAS   = 5'd10; // Read: CAS latency wait
    localparam [4:0] S_RCAP   = 5'd11; // Read: capture data, precharge
    localparam [4:0] S_WDLY   = 5'd12; // Write: data hold (tWR)
    localparam [4:0] S_WPRE   = 5'd13; // Write: precharge
    localparam [4:0] S_TRP    = 5'd14; // Precharge tRP wait / access done

    reg [4:0]  state;
    reg [3:0]  cmd;
    reg [12:0] s_addr;
    reg  [1:0] s_ba;
    reg [15:0] s_dout;
    reg        s_dqml, s_dqmh;
    reg        dq_oe;       // Drive SDRAM_DQ (write window)
    reg  [3:0] timer;        // Wait counter for the timed states
    reg  [3:0] init_refs;
    reg [16:0] pwrup;

    // =========================================================================
    // Port A request latch
    // =========================================================================

    reg        pa_valid;
    reg        pa_wr;
    reg [23:0] pa_waddr;   // Word address
    reg [15:0] pa_wdata;
    reg        pa_uds_n, pa_lds_n;

    // =========================================================================
    // Port B write FIFO (8 entries)
    // =========================================================================

    reg [20:0] fifo_addr [0:7];
    reg [15:0] fifo_data [0:7];
    reg  [3:0] fifo_wptr, fifo_rptr; // Extra bit = full/empty flag
    wire [3:0] fifo_count = fifo_wptr - fifo_rptr;
    wire       fifo_empty = (fifo_count == 4'd0);

    assign b_wait = (fifo_count >= 4'd6);

    always @(posedge clk) begin
        if (reset) begin
            fifo_wptr <= 4'd0;
        end else if (b_wr && !b_wait) begin
            fifo_addr[fifo_wptr[2:0]] <= b_addr;
            fifo_data[fifo_wptr[2:0]] <= b_wdata;
            fifo_wptr <= fifo_wptr + 4'd1;
        end
    end

    // =========================================================================
    // Refresh timer (runs once init_done; requests are queued in ref_pend)
    // =========================================================================

    reg [15:0] ref_timer;
    reg  [1:0] ref_pend;
    wire       ref_due = (ref_timer == REF_PERIOD - 16'd1);

    // ref_take pulses for the one cycle the main FSM spends in S_IDLE
    // issuing a queued refresh. All ref_pend updates live in this single
    // block so the net has exactly one driver (Quartus error 10028
    // otherwise).
    wire       ref_take = (state == S_IDLE) && (ref_pend != 2'd0);

    always @(posedge clk) begin
        if (reset) begin
            ref_timer <= 16'd0;
            ref_pend  <= 2'd0;
        end else if (init_done) begin
            if (ref_due)
                ref_timer <= 16'd0;
            else
                ref_timer <= ref_timer + 16'd1;

            case ({ref_due && (ref_pend != 2'd3), ref_take})
                2'b01:   ref_pend <= ref_pend - 2'd1; // FSM issued one
                2'b10:   ref_pend <= ref_pend + 2'd1; // one came due
                2'b11:   ;                           // both: net zero
                default: ;                           // neither: hold
            endcase
        end
    end

    // =========================================================================
    // Selected request source (Port A has priority)
    // =========================================================================

    wire        src_valid = pa_valid || !fifo_empty;
    wire        src_wr    = pa_valid ? pa_wr  : 1'b1;
    wire [23:0] src_addr  = pa_valid ? pa_waddr : {3'b000, fifo_addr[fifo_rptr[2:0]]};
    wire [15:0] src_data  = pa_valid ? pa_wdata : fifo_data[fifo_rptr[2:0]];
    wire        src_uds_n = pa_valid ? pa_uds_n : 1'b0;
    wire        src_lds_n = pa_valid ? pa_lds_n : 1'b0;

    // Request details captured for the access in flight
    reg         cur_wr;
    reg         cur_port_a;
    reg         cur_uds_n, cur_lds_n;
    reg   [1:0] cur_bank;
    reg   [8:0] cur_col;
    reg  [15:0] cur_data;

    // =========================================================================
    // Main FSM
    // =========================================================================

    always @(posedge clk) begin
        if (reset) begin
            state      <= S_RESET;
            cmd        <= CMD_NOP;
            s_addr     <= 13'd0;
            s_ba       <= 2'd0;
            s_dout     <= 16'd0;
            s_dqml     <= 1'b1;
            s_dqmh     <= 1'b1;
            dq_oe      <= 1'b0;
            timer      <= 4'd0;
            init_refs  <= 4'd0;
            pwrup      <= 17'd0;
            init_done  <= 1'b0;
            a_rdata    <= 16'd0;
            a_ready    <= 1'b0;
            pa_valid   <= 1'b0;
            pa_wr      <= 1'b0;
            pa_waddr   <= 24'd0;
            pa_wdata   <= 16'd0;
            pa_uds_n   <= 1'b1;
            pa_lds_n   <= 1'b1;
            fifo_rptr  <= 4'd0;
            cur_wr     <= 1'b0;
            cur_port_a <= 1'b0;
            cur_uds_n  <= 1'b1;
            cur_lds_n  <= 1'b1;
            cur_bank   <= 2'd0;
            cur_col    <= 9'd0;
            cur_data   <= 16'd0;
        end else begin
            cmd     <= CMD_NOP;
            dq_oe   <= 1'b0;
            s_dqml  <= 1'b1;
            s_dqmh  <= 1'b1;
            a_ready <= 1'b0;

            // Latch new port A requests (one outstanding at a time)
            if ((a_rd || a_wr) && !pa_valid) begin
                pa_valid <= 1'b1;
                pa_wr    <= a_wr;
                pa_waddr <= a_addr[24:1];
                pa_wdata <= a_wdata;
                pa_uds_n <= a_uds_n;
                pa_lds_n <= a_lds_n;
            end

            case (state)
                // -----------------------------------------------------
                // Power-up: hold NOP, DQM masked
                S_RESET: begin
                    if (pwrup == PWRUP_WAIT)
                        state <= S_IPRE;
                    else
                        pwrup <= pwrup + 17'd1;
                end

                // -----------------------------------------------------
                // Initialization sequence
                // -----------------------------------------------------
                S_IPRE: begin
                    cmd       <= CMD_PRE;
                    s_addr    <= 13'b0010000000000; // A10=1: all banks
                    s_ba      <= 2'b00;
                    timer     <= 4'd2;
                    init_refs <= INIT_REFS;
                    state     <= S_IPREW;
                end

                S_IPREW: begin
                    if (timer != 4'd0)
                        timer <= timer - 4'd1;
                    else
                        state <= S_IREF;
                end

                S_IREF: begin
                    cmd       <= CMD_REF;
                    timer     <= 4'd4;              // tRC
                    init_refs <= init_refs - 4'd1;
                    state     <= S_REFW;
                end

                S_REFW: begin
                    if (timer != 4'd0)
                        timer <= timer - 4'd1;
                    else if (init_refs != 4'd0)
                        state <= S_IREF;
                    else
                        state <= S_ILMR;
                end

                S_ILMR: begin
                    cmd    <= CMD_LMR;
                    s_ba   <= 2'b00;
                    s_addr <= MODE_REG;
                    timer  <= 4'd3;                 // tMRD
                    state  <= S_ILMRW;
                end

                S_ILMRW: begin
                    if (timer != 4'd0)
                        timer <= timer - 4'd1;
                    else begin
                        init_done <= 1'b1;
                        state     <= S_IDLE;
                    end
                end

                // -----------------------------------------------------
                // Idle: service refreshes, then requests
                // -----------------------------------------------------
                S_IDLE: begin
                    if (ref_take) begin
                        cmd      <= CMD_REF;
                        timer    <= 4'd4;
                        state    <= S_REF;
                    end else if (src_valid) begin
                        // Capture the request and activate its row
                        cur_wr     <= src_wr;
                        cur_port_a <= pa_valid;
                        cur_uds_n  <= src_uds_n;
                        cur_lds_n  <= src_lds_n;
                        cur_data   <= src_data;
                        cur_bank   <= src_addr[23:22];
                        cur_col    <= src_addr[8:0];

                        cmd    <= CMD_ACT;
                        s_ba   <= src_addr[23:22];
                        s_addr <= src_addr[21:9];   // row
                        timer  <= 4'd2;             // tRCD
                        state  <= S_ACT;

                        // Consume the source
                        if (pa_valid)
                            pa_valid <= 1'b0;
                        else
                            fifo_rptr <= fifo_rptr + 4'd1;
                    end
                end

                // Runtime refresh wait
                S_REF: begin
                    if (timer != 4'd0)
                        timer <= timer - 4'd1;
                    else
                        state <= S_IDLE;
                end

                // -----------------------------------------------------
                // Access: tRCD wait, then READ or WRITE
                // -----------------------------------------------------
                S_ACT: begin
                    if (timer != 4'd0) begin
                        timer <= timer - 4'd1;
                    end else if (cur_wr) begin
                        cmd    <= CMD_WR;
                        s_ba   <= cur_bank;
                        s_addr <= {3'd0, 1'b0, cur_col}; // A10=0, col A[8:0]
                        s_dout <= cur_data;
                        dq_oe  <= 1'b1;
                        s_dqml <= cur_lds_n;       // 0 = byte enabled
                        s_dqmh <= cur_uds_n;
                        state  <= S_WDLY;
                    end else begin
                        cmd    <= CMD_RD;
                        s_ba   <= cur_bank;
                        s_addr <= {3'd0, 1'b0, cur_col}; // A10=0, col A[8:0]
                        s_dqml <= 1'b0;
                        s_dqmh <= 1'b0;
                        timer  <= 4'd2;            // CAS latency
                        state  <= S_RCAS;
                    end
                end

                // Read: CAS latency wait.
                //
                // Capture happens HERE, in the timer==0 branch, NOT in
                // S_RCAP: with CL=2 and BL=1 the chip drives DQ for
                // exactly one SDRAM clock window (it releases the bus
                // immediately after), and that window ends one half
                // master cycle before S_RCAP would run. Capturing in
                // S_RCAP sampled the lines after the chip had let go —
                // invisible in simulation (the model never tri-states
                // DQ) but intermittent bit errors on real silicon
                // (garbled glyphs, hung boot). The rising edge here is
                // half a cycle after the data edge: inside the window.
                S_RCAS: begin
                    s_dqml <= 1'b0;
                    s_dqmh <= 1'b0;
                    if (timer != 4'd0)
                        timer <= timer - 4'd1;
                    else begin
                        a_rdata <= SDRAM_DQ_IN;
                        state   <= S_RCAP;
                    end
                end

                // Read: close the row
                S_RCAP: begin
                    s_dqml  <= 1'b0;
                    s_dqmh  <= 1'b0;
                    cmd     <= CMD_PRE;
                    s_ba    <= cur_bank;
                    s_addr  <= 13'd0;
                    timer   <= 4'd2;               // tRP
                    state   <= S_TRP;
                end

                // Write: keep DQ valid one extra cycle (tWR)
                S_WDLY: begin
                    dq_oe  <= 1'b1;
                    s_dout <= cur_data;
                    s_dqml <= cur_lds_n;
                    s_dqmh <= cur_uds_n;
                    state  <= S_WPRE;
                end

                // Write: close the row
                S_WPRE: begin
                    cmd   <= CMD_PRE;
                    s_ba  <= cur_bank;
                    s_addr <= 13'd0;
                    timer <= 4'd2;                 // tRP
                    state <= S_TRP;
                end

                // Precharge wait; then the access is done
                S_TRP: begin
                    if (timer != 4'd0)
                        timer <= timer - 4'd1;
                    else begin
                        if (cur_port_a)
                            a_ready <= 1'b1;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // SDRAM pin drivers
    // =========================================================================

    assign SDRAM_CLK  = ~clk;
    assign SDRAM_CKE  = 1'b1;
    assign SDRAM_A    = s_addr;
    assign SDRAM_BA   = s_ba;
    assign SDRAM_DQ_OUT = s_dout;
    assign SDRAM_DQ_OE  = dq_oe;
    assign SDRAM_DQML = s_dqml;
    assign SDRAM_DQMH = s_dqmh;
    assign SDRAM_nCS  = cmd[3];
    assign SDRAM_nRAS = cmd[2];
    assign SDRAM_nCAS = cmd[1];
    assign SDRAM_nWE  = cmd[0];

endmodule
