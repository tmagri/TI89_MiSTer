//
// mem_ctrl.sv — TI-89 Titanium address decoder, RAM and bus controller
// TI-89 MiSTer Core
//
// Memory map (TI-89 Titanium / HW3, from TiEmu mem89tm.c):
//   $000000-$03FFFF : RAM  (256 KB, mirrored) — on-chip BRAM
//   $200000-$23FFFF : RAM mirror
//   $400000-$43FFFF : RAM mirror
//   $600000-$6FFFFF : I/O Bank 1 (32 bytes, mirrored)
//   $700000-$7000FF : I/O Bank 2 (64 bytes, HW2+)
//   $710000-$7100FF : I/O Bank 3 (256 bytes, HW3+)
//   $800000-$BFFFFF : FLASH (4 MB) — SDRAM behind the flash_ctrl WSM
//   All other       : Unmapped → reads return $1414
//
// Boot: a .89u OS upgrade has no boot block. After the image is loaded
// into SDRAM (at byte offset 0), this controller performs the same steps
// TiEmu does in reset_calculator():
//   1. clear all RAM,
//   2. copy 128 words from image offset $12088 (the OS header, which
//      begins with the initial SSP and PC long words) to RAM $000000,
//   3. release the CPU. The 68000 reset then fetches SSP from $000000
//      and PC from $000004 (both in RAM); PC points into the FLASH
//      window and the OS starts.
//
// While boot_done is low the CPU must be held in reset externally.
//

module mem_ctrl (
    input         clk,
    input         reset,

    // Boot control
    input         rom_loaded,  // OS image present in SDRAM (from rom_loader)
    output reg    boot_done,   // boot complete — CPU may leave reset

    // CPU bus interface
    input  [23:1] cpu_addr,
    input  [15:0] cpu_dout,
    output reg [15:0] cpu_din,
    input         cpu_as_n,
    input         cpu_uds_n,
    input         cpu_lds_n,
    input         cpu_rw_n,      // 1=read, 0=write
    output reg    cpu_dtack_n,

    // LCD DMA port (port B of the dual-port RAM)
    // Synchronous read: address this cycle, lcd_ram_data valid next cycle.
    input      [17:0] lcd_ram_addr,  // Byte address within 256KB RAM
    output reg [15:0] lcd_ram_data,

    // FLASH controller interface (sits between this module and the SDRAM)
    output reg [21:0] flash_addr,   // Byte address within 4MB flash window
    output reg [15:0] flash_wdata,
    input      [15:0] flash_rdata,
    output reg        flash_rd,     // One-cycle strobe
    output reg        flash_wr,     // One-cycle strobe
    output reg        flash_uds_n,  // Byte lanes (writes)
    output reg        flash_lds_n,
    input             flash_ready,  // Request complete / rdata valid

    // I/O port controller interface
    output reg  [7:0] io_addr,     // I/O register address
    output reg [15:0] io_wdata,    // Write data to I/O
    input      [15:0] io_rdata,    // Read data from I/O (combinational)
    output reg        io_rd,       // One-cycle read strobe
    output reg        io_wr,       // One-cycle write strobe
    output reg  [1:0] io_bank,     // 0=bank1($6xxxxx), 1=bank2, 2=bank3
    output        io_uds_n,    // Byte lanes of the current I/O request
    output        io_lds_n
);

    // =========================================================================
    // RAM — 256 KB on-chip (M10K blocks), true dual-port
    //   Port A: CPU access (read/write, byte enables) + boot FSM writes
    //   Port B: LCD DMA read
    // =========================================================================

    reg [15:0] ram [0:131071]; // 131072 x 16-bit = 256 KB

    // Port B: LCD DMA synchronous read
    always @(posedge clk) begin
        lcd_ram_data <= ram[lcd_ram_addr[17:1]];
    end

    // =========================================================================
    // Bus cycle state machine
    // =========================================================================
    // Two requesters share the FSM:
    //   - the boot FSM (while boot_done is low; CPU is held in reset then)
    //   - the CPU
    // The request is latched in S_IDLE and processed from registers.

    localparam [2:0] S_IDLE   = 3'd0;
    localparam [2:0] S_ACCESS = 3'd1;
    localparam [2:0] S_IO     = 3'd2;
    localparam [2:0] S_WAIT   = 3'd3;
    localparam [2:0] S_DONE   = 3'd4;

    reg [2:0]  state;

    // Latched request
    reg [23:0] req_addr;   // Full byte address
    reg        req_rw;     // 1=read, 0=write
    reg        req_uds_n;
    reg        req_lds_n;
    reg [15:0] req_wdata;
    reg        req_boot;   // Request came from the boot FSM

    // Byte lanes of the latched request (stable while the I/O strobes fire)
    assign io_uds_n = req_uds_n;
    assign io_lds_n = req_lds_n;

    // Address decoding (from the latched address)
    wire sel_ram   = (req_addr[23:18] == 6'b000000) ||   // $000000-$03FFFF
                     (req_addr[23:18] == 6'b001000) ||   // $200000-$23FFFF
                     (req_addr[23:18] == 6'b010000);     // $400000-$43FFFF
    wire sel_io1   = (req_addr[23:20] == 4'h6);          // $600000-$6FFFFF
    wire sel_io2   = (req_addr[23:16] == 8'h70) && (req_addr[15:8] == 8'h00);
    wire sel_io3   = (req_addr[23:16] == 8'h71) && (req_addr[15:8] == 8'h00);
    wire sel_io    = sel_io1 || sel_io2 || sel_io3;
    wire sel_flash = (req_addr[23:22] == 2'b10);         // $800000-$BFFFFF

    wire [16:0] ram_word_addr = req_addr[17:1];

    // Boot FSM state (declared here so the bus FSM above can reference it)
    localparam [2:0] B_WAIT  = 3'd0;
    localparam [2:0] B_CLEAR = 3'd1;
    localparam [2:0] B_COPY  = 3'd2;
    localparam [2:0] B_CWAIT = 3'd3;
    localparam [2:0] B_DONE  = 3'd4;

    reg [2:0]  boot_state;
    reg [16:0] clr_idx;    // RAM clear index  (0..131071)
    reg [6:0]  boot_idx;   // Header word index (0..127)

    // Boot FSM handshake: one flash-window read per header word
    wire boot_req = (boot_state == B_COPY);
    reg  boot_ack;      // Boot read data consumed this cycle

    always @(posedge clk) begin
        if (reset) begin
            state       <= S_IDLE;
            cpu_dtack_n <= 1'b1;
            cpu_din     <= 16'h1414;
            flash_rd    <= 1'b0;
            flash_wr    <= 1'b0;
            flash_addr  <= 22'd0;
            flash_wdata <= 16'd0;
            flash_uds_n <= 1'b1;
            flash_lds_n <= 1'b1;
            io_rd       <= 1'b0;
            io_wr       <= 1'b0;
            io_addr     <= 8'd0;
            io_bank     <= 2'd0;
            io_wdata    <= 16'd0;
            boot_ack    <= 1'b0;
        end else begin
            // Default: deassert one-cycle strobes
            flash_rd <= 1'b0;
            flash_wr <= 1'b0;
            io_rd    <= 1'b0;
            io_wr    <= 1'b0;
            boot_ack <= 1'b0;

            case (state)
                S_IDLE: begin
                    cpu_dtack_n <= 1'b1;
                    // RAM clear pass (port A write, one word per cycle).
                    // The boot FSM advances clr_idx in lockstep.
                    if (boot_state == B_CLEAR)
                        ram[clr_idx] <= 16'd0;
                    if (boot_req) begin
                        // Boot header copy: read flash window $812088..
                        req_addr  <= 24'h812088 + {16'd0, boot_idx, 1'b0};
                        req_rw    <= 1'b1;
                        req_uds_n <= 1'b0;
                        req_lds_n <= 1'b0;
                        req_wdata <= 16'd0;
                        req_boot  <= 1'b1;
                        state     <= S_ACCESS;
                    end else if (!cpu_as_n &&
                                 (!cpu_uds_n || !cpu_lds_n)) begin
                        // A bus cycle is qualified by its data strobes, not
                        // by AS alone. This matters for WRITE cycles: on the
                        // 68000 bus AS asserts about one phase before UDS/LDS
                        // (fx68k drives AS at bus phase S0 and the data
                        // strobes at S2). Latching on AS alone would capture
                        // writes while both strobes are still negated, the
                        // byte-lane logic would see "no lanes active", and
                        // every RAM/IO/FLASH write would be silently dropped.
                        // Address and write data are already stable when the
                        // strobes assert, so sampling here is safe.
                        //
                        // The only cycles that hold AS low with both strobes
                        // negated are interrupt acknowledge cycles (answered
                        // through VPA/autovector, no DTACK expected) and
                        // address-error accesses (aborted internally by the
                        // CPU), so ignoring those is the correct behavior.
                        req_addr  <= {cpu_addr, 1'b0};
                        req_rw    <= cpu_rw_n;
                        req_uds_n <= cpu_uds_n;
                        req_lds_n <= cpu_lds_n;
                        req_wdata <= cpu_dout;
                        req_boot  <= 1'b0;
                        state     <= S_ACCESS;
                    end
                end

                S_ACCESS: begin
                    if (sel_ram && !req_boot) begin
                        // RAM — single-cycle access
                        if (req_rw) begin
                            cpu_din <= ram[ram_word_addr];
                        end else begin
                            if (!req_uds_n)
                                ram[ram_word_addr][15:8] <= req_wdata[15:8];
                            if (!req_lds_n)
                                ram[ram_word_addr][7:0]  <= req_wdata[7:0];
                        end
                        cpu_dtack_n <= 1'b0;
                        state <= S_DONE;

                    end else if (sel_flash) begin
                        // FLASH via flash_ctrl (WSM + SDRAM)
                        flash_addr  <= req_addr[21:0];
                        flash_wdata <= req_wdata;
                        flash_uds_n <= req_uds_n;
                        flash_lds_n <= req_lds_n;
                        if (req_rw)
                            flash_rd <= 1'b1;
                        else
                            flash_wr <= 1'b1;
                        state <= S_WAIT;

                    end else if (sel_io && !req_boot) begin
                        // I/O — register address/strobes for the next cycle
                        if (sel_io1) begin
                            io_addr <= req_addr[4:0];
                            io_bank <= 2'd0;
                        end else if (sel_io2) begin
                            io_addr <= req_addr[7:0];
                            io_bank <= 2'd1;
                        end else begin
                            io_addr <= req_addr[7:0];
                            io_bank <= 2'd2;
                        end
                        io_wdata <= req_wdata;
                        state <= S_IO;

                    end else begin
                        // Unmapped (or boot request to a non-flash address,
                        // which cannot happen) — return bus idle value
                        cpu_din     <= 16'h1414;
                        cpu_dtack_n <= 1'b0;
                        state       <= S_DONE;
                    end
                end

                S_IO: begin
                    // io_addr/io_bank are valid now; io_rdata is combinational
                    io_rd <= req_rw;
                    io_wr <= !req_rw;
                    if (req_rw)
                        cpu_din <= io_rdata;
                    cpu_dtack_n <= 1'b0;
                    state <= S_DONE;
                end

                S_WAIT: begin
                    // Waiting for the flash controller
                    if (flash_ready) begin
                        if (req_rw) begin
                            if (req_boot) begin
                                // Header word -> RAM (port A write)
                                ram[boot_idx]      <= flash_rdata;
                                boot_ack           <= 1'b1;
                            end else begin
                                cpu_din <= flash_rdata;
                            end
                        end
                        cpu_dtack_n <= 1'b0;
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    // Wait for the requester to end the cycle
                    if (req_boot || cpu_as_n) begin
                        cpu_dtack_n <= 1'b1;
                        state <= S_IDLE;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Boot FSM
    // =========================================================================
    // B_WAIT  : wait until an OS image has been loaded into SDRAM
    // B_CLEAR : clear all RAM (TiEmu erases RAM upon reset); the actual RAM
    //           writes are performed by the bus FSM above (single port-A
    //           writer) using clr_idx
    // B_COPY  : request a flash read of header word boot_idx
    // B_CWAIT : wait until the bus FSM delivers it (boot_ack)
    // B_DONE  : hold boot_done high until a new image is loaded

    always @(posedge clk) begin
        if (reset || !rom_loaded) begin
            boot_state <= B_WAIT;
            boot_done  <= 1'b0;
            clr_idx    <= 17'd0;
            boot_idx   <= 7'd0;
        end else begin
            case (boot_state)
                B_WAIT: begin
                    boot_state <= B_CLEAR;
                end

                B_CLEAR: begin
                    // The bus FSM writes ram[clr_idx] <= 0 while we advance
                    if (clr_idx == 17'd131071) begin
                        clr_idx    <= 17'd0;
                        boot_state <= B_COPY;
                    end else begin
                        clr_idx <= clr_idx + 17'd1;
                    end
                end

                B_COPY: begin
                    // Wait for the bus FSM to pick up the request
                    if (state != S_IDLE)
                        boot_state <= B_CWAIT;
                end

                B_CWAIT: begin
                    if (boot_ack) begin
                        if (boot_idx == 7'd127) begin
                            boot_state <= B_DONE;
                        end else begin
                            boot_idx   <= boot_idx + 7'd1;
                            boot_state <= B_COPY;
                        end
                    end
                end

                B_DONE: begin
                    boot_done <= 1'b1;
                end
            endcase
        end
    end

endmodule
