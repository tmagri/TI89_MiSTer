//
// mem_ctrl.sv — TI-89 Titanium address decoder, bus controller and
//               SDRAM arbiter
// TI-89 MiSTer Core
//
// Memory map (TI-89 Titanium / HW3, from TiEmu mem89tm.c):
//   $000000-$03FFFF : RAM (256 KB, mirrored) — stored in SDRAM
//   $200000-$23FFFF : RAM mirror
//   $400000-$43FFFF : RAM mirror
//   $600000-$6FFFFF : I/O Bank 1 (32 bytes, mirrored)
//   $700000-$7000FF : I/O Bank 2 (64 bytes, HW2+)
//   $710000-$7100FF : I/O Bank 3 (256 bytes, HW3+)
//   $800000-$BFFFFF : FLASH (4 MB) — SDRAM behind the flash_ctrl WSM
//   All other       : Unmapped → reads return $1414. The reference
//                     emulator (n-89 / TiEmu, mem.c + mem89tm.c) models
//                     every unused address as a 0x14-filled byte, so a
//                     word reads $1414 / long $14141414; the OS relies
//                     on this value.
//
// The 256 KB calculator RAM lives in the MiSTer SDRAM chip, NOT in
// on-chip M10K blocks: the Cyclone V has only 553 M10Ks (~5.5 Mbit) and
// the ascal scaler, OSD buffers and fx68k micro-Roms already consume
// most of them. SDRAM layout (byte addresses):
//   $000000-$3FFFFF : OS image / flash window (4 MB)
//   $400000-$43FFFF : calculator RAM (256 KB)
//
// This module owns SDRAM port A and arbitrates it between two clients:
//   - FLASH : flash_ctrl's sd_* pins (its WSM issues its own SDRAM
//             reads/writes for program read-modify-write and erase fill)
//   - RAM   : a single request slot fed, in priority order, by the
//             LCD DMA, the CPU bus FSM and the boot FSM
// Each SDRAM transaction is a strobe followed ~8 cycles later by the
// sd_ready pulse. The 68000 bus is slow enough that the added latency
// is invisible; the LCD DMA fetches a whole row in well under one
// horizontal blanking period.
//
// Boot: a .89u OS upgrade has no boot block. After the image is loaded
// into SDRAM (at byte offset 0), this controller performs the same steps
// TiEmu does in reset_calculator():
//   0. (diagnostic build) dump the whole 4MB flash image back out over
//      the dbg_uart link — DUMP_PASSES times — so the host can verify,
//      byte for byte, both what the loader wrote and what the SDRAM
//      read path returns (a deterministic mismatch indicts the load
//      path; a pass-to-pass varying mismatch indicts read timing),
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
    input         init_done,   // SDRAM controller initialized
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

    // LCD DMA port (request/acknowledge handshake)
    // lcd_ram_req is held high until lcd_ram_ack pulses; lcd_ram_addr
    // must stay stable while req is high. lcd_ram_data is valid in the
    // same cycle as lcd_ram_ack.
    input      [17:0] lcd_ram_addr,  // Byte address within 256KB RAM
    output reg [15:0] lcd_ram_data,
    input             lcd_ram_req,
    output reg        lcd_ram_ack,

    // FLASH controller interface, command side (mem_ctrl -> flash_ctrl)
    output reg [21:0] flash_addr,   // Byte address within 4MB flash window
    output reg [15:0] flash_wdata,
    input      [15:0] flash_rdata,
    output reg        flash_rd,     // One-cycle strobe
    output reg        flash_wr,     // One-cycle strobe
    output reg        flash_uds_n,  // Byte lanes (writes)
    output reg        flash_lds_n,
    input             flash_ready,  // Request complete / rdata valid

    // FLASH controller interface, SDRAM side (flash_ctrl -> arbiter).
    // flash_ctrl's sd_* pins connect here; the arbiter multiplexes them
    // with the RAM clients onto the SDRAM port A pins below.
    input  [21:0] fl_sd_addr,       // Byte address within the flash window
    input  [15:0] fl_sd_wdata,
    input         fl_sd_rd,         // One-cycle strobe
    input         fl_sd_wr,         // One-cycle strobe
    input         fl_sd_uds_n,
    input         fl_sd_lds_n,
    output [15:0] fl_sd_rdata,      // Combinational pass-through of sd_rdata
    output reg    fl_sd_ready,      // One-cycle completion pulse

    // SDRAM port A (arbitrated output, strobe / sd_ready handshake)
    output reg [24:0] sd_addr,      // Byte address within 32MB
    output reg [15:0] sd_wdata,
    output reg        sd_rd,        // One-cycle read strobe
    output reg        sd_wr,        // One-cycle write strobe
    output reg        sd_uds_n,     // Byte lanes (writes)
    output reg        sd_lds_n,
    input      [15:0] sd_rdata,
    input             sd_ready,     // One-cycle completion pulse

    // I/O port controller interface
    output reg  [7:0] io_addr,     // I/O register address
    output reg [15:0] io_wdata,    // Write data to I/O
    input      [15:0] io_rdata,    // Read data from I/O (combinational)
    output reg        io_rd,       // One-cycle read strobe
    output reg        io_wr,       // One-cycle write strobe
    output reg  [1:0] io_bank,     // 0=bank1($6xxxxx), 1=bank2, 2=bank3
    output        io_uds_n,    // Byte lanes of the current I/O request
    output        io_lds_n,

    // Flash protection state (TiEmu hwprot.c). io_ports gates writes to
    // io2[$00-$0F], io2[$12] and io2[$1F] with this.
    output reg    protect,

    // AI7 "vector table write protection & stack overflow" (mem.c
    // put_long/put_word/put_byte): prot_arm = $600001 bit 2 from
    // io_ports; ai7_hit pulses for one cycle when a CPU RAM write lands
    // below $000120 while armed. The write still completes — the level-7
    // interrupt is taken after the current instruction, exactly like the
    // reference.
    input         prot_arm,
    output reg    ai7_hit,

    // ---- Pre-boot SDRAM image dump (diagnostic) ----
    // Streams the 4MB flash image to dbg_uart, DUMP_PASSES times, while
    // the CPU is still held in reset. Handshake: mem_ctrl pulses
    // dump_stb (dump_word valid) only in a cycle where dump_rdy is high;
    // dbg_uart drops dump_rdy until it has shifted both bytes out.
    // dump_pass_stb marks the start of each pass (dbg_uart prints a
    // "$P<n>" line); dump_active covers the whole dump so dbg_uart can
    // suppress its periodic status lines.
    input             dump_rdy,
    output            dump_stb,     // combinational: valid exactly when dump_rdy
    output     [15:0] dump_word,
    output reg        dump_pass_stb,
    output            dump_active,

    // ---- Host command dumps (post-boot, bounded; from dbg_uart RX) ----
    // Streams an arbitrary bounded range to dbg_uart with the same
    // framing (marker "$D" via dump_cmd_mode). cmd_req is only honored
    // in B_DONE (CPU released, no boot dump in flight) and re-latches
    // start/len from the (already validated/clamped) command inputs.
    input             cmd_req,
    input             cmd_mem,      // 0 = flash image, 1 = calculator RAM
    input      [23:0] cmd_start,    // byte offset (even)
    input      [23:0] cmd_len,      // byte length (even, nonzero)
    output reg        dump_cmd_mode
);

    // =========================================================================
    // Calculator RAM location inside the SDRAM
    // =========================================================================
    localparam [24:0] RAM_BASE = 25'h0400000; // after the 4MB image area

    // =========================================================================
    // SDRAM port A arbiter
    // =========================================================================
    localparam [1:0] G_NONE  = 2'd0;
    localparam [1:0] G_FLASH = 2'd1;
    localparam [1:0] G_RAM   = 2'd2;

    localparam [1:0] SRC_LCD  = 2'd0;
    localparam [1:0] SRC_CPU  = 2'd1;
    localparam [1:0] SRC_BOOT = 2'd2;

    reg [1:0] grant;

    // ---- FLASH client: latch flash_ctrl's strobes until granted ----
    reg        fl_pend;
    reg        fl_p_wr;
    reg [21:0] fl_p_addr;
    reg [15:0] fl_p_wdata;
    reg        fl_p_uds_n, fl_p_lds_n;

    always @(posedge clk) begin
        if (reset) begin
            fl_pend    <= 1'b0;
            fl_p_wr    <= 1'b0;
            fl_p_addr  <= 22'd0;
            fl_p_wdata <= 16'd0;
            fl_p_uds_n <= 1'b1;
            fl_p_lds_n <= 1'b1;
        end else begin
            if (grant == G_FLASH && sd_ready)
                fl_pend <= 1'b0;
            // flash_ctrl only strobes after receiving fl_sd_ready, so a
            // new strobe cannot collide with an outstanding request.
            if (fl_sd_rd || fl_sd_wr) begin
                fl_pend    <= 1'b1;
                fl_p_wr    <= fl_sd_wr;
                fl_p_addr  <= fl_sd_addr;
                fl_p_wdata <= fl_sd_wdata;
                fl_p_uds_n <= fl_sd_uds_n;
                fl_p_lds_n <= fl_sd_lds_n;
            end
        end
    end

    // ---- RAM clients ----

    // CPU / boot request lines (set by their FSMs below, cleared when
    // the request is taken into the slot)
    reg cpu_ram_want;
    reg boot_ram_want;
    reg boot_ram_flying;

    // Boot FSM RAM-write parameters (held until taken into the slot)
    reg [24:0] boot_ram_addr;
    reg [15:0] boot_ram_wdata;
    reg        boot_ram_we;   // 0 = boot RAM-slot request is a READ (dump)

    // ---- Pre-boot dump registers (see port comment) ----
    reg  [15:0] dump_rd_data;  // word latched by the grant FSM (reads)
    reg  [20:0] dump_idx;      // SDRAM word index within the 4MB image
    reg  [1:0]  dump_pass;     // pass counter 0..DUMP_PASSES-1
    reg         dwait_rd;      // read completed, waiting for dump_rdy

    assign dump_word  = dump_rd_data;

    // Bus FSM latched request registers (declared here because the RAM
    // slot samples them combinationally when it loads a CPU request)
    reg [23:0] req_addr;   // Full byte address
    reg        req_rw;     // 1=read, 0=write
    reg        req_uds_n;
    reg        req_lds_n;
    reg [15:0] req_wdata;
    reg        req_boot;   // Request came from the boot FSM

    // LCD DMA: edge-detect the held request into lcd_pend
    reg        lcd_req_q;
    reg        lcd_pend;
    reg [17:0] lcd_p_addr;
    wire       lcd_shot = lcd_ram_req && !lcd_req_q;

    // ---- The single RAM request slot ----
    reg        ram_valid;
    reg        ram_wr;
    reg [24:0] ram_addr;
    reg [15:0] ram_wdata;
    reg        ram_uds_n, ram_lds_n;
    reg  [1:0] ram_src;

    wire ram_load_lcd  = !ram_valid && lcd_pend;
    wire ram_load_cpu  = !ram_valid && !lcd_pend && cpu_ram_want;
    wire ram_load_boot = !ram_valid && !lcd_pend && !cpu_ram_want &&
                         boot_ram_want;

    always @(posedge clk) begin
        if (reset) begin
            lcd_req_q  <= 1'b0;
            lcd_pend   <= 1'b0;
            lcd_p_addr <= 18'd0;
        end else begin
            lcd_req_q <= lcd_ram_req;
            if (lcd_shot) begin
                lcd_pend   <= 1'b1;
                lcd_p_addr <= lcd_ram_addr;
            end else if (ram_load_lcd) begin
                lcd_pend   <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            ram_valid <= 1'b0;
            ram_wr    <= 1'b0;
            ram_addr  <= 25'd0;
            ram_wdata <= 16'd0;
            ram_uds_n <= 1'b1;
            ram_lds_n <= 1'b1;
            ram_src   <= SRC_LCD;
        end else begin
            if (grant == G_RAM && sd_ready) begin
                ram_valid <= 1'b0;
            end else if (!ram_valid) begin
                // Priority: LCD DMA (fixed raster timing) > CPU > boot
                if (lcd_pend) begin
                    ram_valid <= 1'b1;
                    ram_src   <= SRC_LCD;
                    ram_wr    <= 1'b0;
                    ram_addr  <= RAM_BASE + {6'd0, lcd_p_addr[17:1], 1'b0};
                    ram_wdata <= 16'd0;
                    ram_uds_n <= 1'b0;
                    ram_lds_n <= 1'b0;
                end else if (cpu_ram_want) begin
                    ram_valid <= 1'b1;
                    ram_src   <= SRC_CPU;
                    ram_wr    <= !req_rw;
                    ram_addr  <= RAM_BASE + {6'd0, req_addr[17:1], 1'b0};
                    ram_wdata <= req_wdata;
                    ram_uds_n <= req_uds_n;
                    ram_lds_n <= req_lds_n;
                end else if (boot_ram_want) begin
                    ram_valid <= 1'b1;
                    ram_src   <= SRC_BOOT;
                    ram_wr    <= boot_ram_we;   // 0 for dump reads
                    ram_addr  <= boot_ram_addr;
                    ram_wdata <= boot_ram_wdata;
                    ram_uds_n <= 1'b0;
                    ram_lds_n <= 1'b0;
                end
            end
        end
    end

    // ---- Grant / completion FSM ----
    // sd_rdata is consumed by the client that owns the completed
    // transaction; the bus FSM reads it directly for CPU cycles.
    wire ram_is_lcd = ram_valid && (ram_src == SRC_LCD);
    reg  cpu_ram_done;
    reg  boot_ram_done;

    always @(posedge clk) begin
        if (reset) begin
            grant        <= G_NONE;
            sd_addr      <= 25'd0;
            sd_wdata     <= 16'd0;
            sd_rd        <= 1'b0;
            sd_wr        <= 1'b0;
            sd_uds_n     <= 1'b1;
            sd_lds_n     <= 1'b1;
            fl_sd_ready  <= 1'b0;
            lcd_ram_ack  <= 1'b0;
            lcd_ram_data <= 16'd0;
            cpu_ram_done <= 1'b0;
            boot_ram_done<= 1'b0;
            dump_rd_data <= 16'd0;
        end else begin
            sd_rd         <= 1'b0;
            sd_wr         <= 1'b0;
            fl_sd_ready   <= 1'b0;
            lcd_ram_ack   <= 1'b0;
            cpu_ram_done  <= 1'b0;
            boot_ram_done <= 1'b0;

            case (grant)
                G_NONE: begin
                    // LCD-in-RAM-slot has top priority; otherwise flash
                    // traffic goes before CPU/boot RAM traffic.
                    if (ram_is_lcd || (ram_valid && !fl_pend)) begin
                        grant    <= G_RAM;
                        sd_addr  <= ram_addr;
                        sd_wdata <= ram_wdata;
                        sd_uds_n <= ram_uds_n;
                        sd_lds_n <= ram_lds_n;
                        if (ram_wr) sd_wr <= 1'b1;
                        else        sd_rd <= 1'b1;
                    end else if (fl_pend) begin
                        grant    <= G_FLASH;
                        sd_addr  <= {3'b000, fl_p_addr};
                        sd_wdata <= fl_p_wdata;
                        sd_uds_n <= fl_p_uds_n;
                        sd_lds_n <= fl_p_lds_n;
                        if (fl_p_wr) sd_wr <= 1'b1;
                        else         sd_rd <= 1'b1;
                    end
                end

                G_FLASH: begin
                    if (sd_ready) begin
                        fl_sd_ready <= 1'b1;
                        grant       <= G_NONE;
                    end
                end

                G_RAM: begin
                    if (sd_ready) begin
                        case (ram_src)
                            SRC_LCD: begin
                                lcd_ram_data <= sd_rdata;
                                lcd_ram_ack  <= 1'b1;
                            end
                            SRC_CPU:  cpu_ram_done  <= 1'b1;
                            SRC_BOOT: begin
                                // Dump reads latch their word here; the
                                // boot FSM's done pulse covers writes too.
                                if (!ram_wr)
                                    dump_rd_data <= sd_rdata;
                                boot_ram_done <= 1'b1;
                            end
                        endcase
                        grant <= G_NONE;
                    end
                end

                default: grant <= G_NONE;
            endcase
        end
    end

    assign fl_sd_rdata = sd_rdata;

    // =========================================================================
    // Bus cycle state machine
    // =========================================================================
    // Two requesters share the FSM:
    //   - the boot FSM (while boot_done is low; CPU is held in reset then)
    //   - the CPU
    // The request is latched in S_IDLE and processed from registers.

    localparam [2:0] S_IDLE    = 3'd0;
    localparam [2:0] S_ACCESS  = 3'd1;
    localparam [2:0] S_IO      = 3'd2;
    localparam [2:0] S_WAIT    = 3'd3; // flash access in flight
    localparam [2:0] S_WAITRAM = 3'd4; // RAM access in flight (SDRAM)
    localparam [2:0] S_DONE    = 3'd5;

    reg [2:0]  state;

    // (req_addr / req_rw / req_uds_n / req_lds_n / req_wdata / req_boot
    // are declared near the RAM slot above; the slot samples them
    // combinationally when it loads a CPU request.)

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

    // =========================================================================
    // Boot FSM state (declared here, before the hwprot/bus-FSM sections,
    // because cyc_start below references boot_req)
    // =========================================================================
    localparam [3:0] B_WAIT   = 4'd0;
    localparam [3:0] B_CLEAR  = 4'd1;
    localparam [3:0] B_COPY   = 4'd2;
    localparam [3:0] B_CWAIT  = 4'd3;
    localparam [3:0] B_RWRITE = 4'd4;
    localparam [3:0] B_DONE   = 4'd5;
    localparam [3:0] B_DPASS  = 4'd6; // dump: emit pass marker
    localparam [3:0] B_DREQ   = 4'd7; // dump: launch SDRAM read
    localparam [3:0] B_DWAIT  = 4'd8; // dump: read done -> UART, advance
    localparam [3:0] B_CPASS  = 4'd9; // command dump: emit "$D" marker
    localparam [3:0] B_CREQ   = 4'd10;// command dump: launch SDRAM read
    localparam [3:0] B_CWAITW = 4'd11;// command dump: read done -> UART
    localparam [3:0] B_CEND   = 4'd12;// command dump: complete

    // Number of full-image read-back passes before the boot proceeds
    // (1: the 4MB image was verified bit-exact on hardware 2026-09-01;
    // kept as a load-integrity canary rather than full verification)
    localparam [1:0] DUMP_PASSES = 2'd1;

    reg [3:0]  boot_state;
    reg [16:0] clr_idx;    // RAM clear index  (0..131071)
    reg [6:0]  boot_idx;   // Header word index (0..127)

    // Command dump (host 'D' command): range + progress
    reg        cmd_mem_r;      // 0 = flash image, 1 = calc RAM
    reg [23:0] cmd_start_r;    // first byte offset (even)
    reg [23:0] cmd_left_r;     // bytes remaining (even)
    reg [22:0] cmd_total_w;    // total words in the command range
    wire [22:0] cmd_cur_w = cmd_total_w - cmd_left_r[22:1]; // current word index

    assign dump_active = (boot_state == B_DPASS) || (boot_state == B_DREQ) ||
                         (boot_state == B_DWAIT) || (boot_state == B_CPASS) ||
                         (boot_state == B_CREQ) || (boot_state == B_CWAITW);

    // Combinational strobe: asserted in the SAME cycle dump_rdy is high,
    // so dbg_uart's (dump_stb && du_state == DU_IDLE) acceptance can never
    // miss. (The previous registered version pulsed one cycle later and
    // lost words whenever the producer left IDLE in that gap — visible as
    // the missing word 0 of dump passes 1/2 on 2026-09-01.)
    assign dump_stb   = ((boot_state == B_DWAIT) || (boot_state == B_CWAITW))
                        && dwait_rd && dump_rdy;

    // Boot FSM handshake: one flash-window read per header word
    wire boot_req = (boot_state == B_COPY);
    reg  boot_ack;      // Boot read data consumed this cycle

    // (boot_ram_addr / boot_ram_wdata are declared near the RAM slot
    // above; the boot FSM drives them, the slot consumes them.)

    // =========================================================================
    // Flash protection — TiEmu hwprot.c (HW2+/HW3 paths)
    // =========================================================================
    // The protection logic is stealth address decode: it watches EVERY CPU
    // bus cycle — instruction fetches included; the reference routes
    // fetches through the same hwp_get_word() path (cpu_prefetch.h ->
    // get_word -> hw_get_word) — and tracks consecutive accesses to the
    // phantom windows below. Seven consecutive qualifying accesses with
    // nothing else in between arm the sequence; an access to the enable
    // window $1C0000-$1FFFFF then turns protection ON (read) or OFF
    // (write). Any access outside the phantom windows resets the counter.
    //
    // While protected:
    //   - every flash write is dropped before reaching the WSM (flash.c
    //     FlashWriteByte/Word return on tihw.protect),
    //   - certificate reads return $14/$1414 instead of flash contents
    //     (hwp_get returns before touching the flash array),
    //   - io_ports ignores writes to io2[$00-$0F], io2[$12], io2[$1F].
    // Writes to the boot block $800000-$80FFFF are ALWAYS dropped
    // (hwp_put_byte returns before mem_put), protected or not.
    //
    // ba = rom_base - $200000 = $600000; the IN_BOUNDS2 windows below are
    // ba-relative in the reference and absolute here.

    reg  [4:0] access2; // only the >= 7 comparison matters; saturate at 7

    // One event per CPU bus cycle. Boot FSM cycles are excluded: the
    // reference performs the boot header copy outside the CPU memory map
    // (direct ROM access), so those reads never reach hwp.
    wire cyc_start = (state == S_IDLE) && !boot_req &&
                     !cpu_as_n && (!cpu_uds_n || !cpu_lds_n);

    wire        hwp_odd  = cpu_uds_n && !cpu_lds_n;  // odd byte: A0 = 1
    wire        hwp_word = !cpu_uds_n && !cpu_lds_n;
    wire [23:0] hwp_addr = {cpu_addr, hwp_odd};      // true byte address

    // Phantom windows (no counter reset) and active windows
    wire hwp_arc  = (hwp_addr >= 24'h040000) && (hwp_addr <= 24'h0FFFFF);
    wire hwp_scr  = (hwp_addr >= 24'h180000) && (hwp_addr <= 24'h1BFFFF);
    wire hwp_en   = (hwp_addr >= 24'h1C0000) && (hwp_addr <= 24'h1FFFFF);
    wire hwp_auth = ((hwp_addr >= 24'h800000) && (hwp_addr <= 24'h80FFFF)) ||
                    ((hwp_addr >= 24'h812000) && (hwp_addr <= 24'h817FFF)) ||
                    ((hwp_addr >= 24'h81A000) && (hwp_addr <= 24'h81FFFF));
    wire hwp_cert = ((hwp_addr >= 24'h810000) && (hwp_addr <= 24'h811FFF)) ||
                    ((hwp_addr >= 24'h818000) && (hwp_addr <= 24'h819FFF));

    // Latched-address versions used in S_ACCESS (A0 irrelevant: all of
    // these windows are >= 4KB aligned)
    wire req_boot_blk = (req_addr[23:16] == 8'h80);    // $800000-$80FFFF
    wire req_cert     = ((req_addr >= 24'h810000) && (req_addr <= 24'h811FFF)) ||
                        ((req_addr >= 24'h818000) && (req_addr <= 24'h819FFF));

    reg [4:0] a_nxt;
    reg       p_nxt;

    always @(posedge clk) begin
        if (reset) begin
            protect <= 1'b0;
            access2 <= 5'd0;
        end else if (cyc_start) begin
            if (hwp_en) begin
                // Enable window: counts per byte (odd addresses included);
                // a word access processes as two consecutive byte events,
                // each able to cross the threshold and reset the counter.
                // Reads enable protection, writes disable it.
                a_nxt = access2 + 5'd1;
                p_nxt = protect;
                if (a_nxt >= 5'd7) begin p_nxt = cpu_rw_n; a_nxt = 5'd0; end
                if (hwp_word) begin
                    a_nxt = a_nxt + 5'd1;
                    if (a_nxt >= 5'd7) begin p_nxt = cpu_rw_n; a_nxt = 5'd0; end
                end
                access2 <= a_nxt;
                protect <= p_nxt;
            end else if (hwp_auth) begin
                // Authorization windows: even byte and word accesses count
                // (one increment per word; the odd byte of a word doesn't).
                if (hwp_word || !hwp_odd)
                    access2 <= (access2 >= 5'd7) ? 5'd7 : access2 + 5'd1;
            end else if (!hwp_arc && !hwp_scr && !hwp_cert) begin
                // Any other access breaks the consecutive sequence
                access2 <= 5'd0;
            end
        end
    end

    // =========================================================================
    // Bus cycle state machine
    // =========================================================================
    // (S_* states and 'state' declared above, before the hwprot section.)
    always @(posedge clk) begin
        if (reset) begin
            state        <= S_IDLE;
            cpu_dtack_n  <= 1'b1;
            cpu_din      <= 16'h1414;   // unmapped/unused value (see S_ACCESS)
            flash_rd     <= 1'b0;
            flash_wr     <= 1'b0;
            flash_addr   <= 22'd0;
            flash_wdata  <= 16'd0;
            flash_uds_n  <= 1'b1;
            flash_lds_n  <= 1'b1;
            io_rd        <= 1'b0;
            io_wr        <= 1'b0;
            io_addr      <= 8'd0;
            io_bank      <= 2'd0;
            io_wdata     <= 16'd0;
            ai7_hit      <= 1'b0;
            boot_ack     <= 1'b0;
            cpu_ram_want <= 1'b0;
            req_addr     <= 24'd0;
            req_rw       <= 1'b1;
            req_uds_n    <= 1'b1;
            req_lds_n    <= 1'b1;
            req_wdata    <= 16'd0;
            req_boot     <= 1'b0;
        end else begin
            // Default: deassert one-cycle strobes
            flash_rd <= 1'b0;
            flash_wr <= 1'b0;
            io_rd    <= 1'b0;
            io_wr    <= 1'b0;
            ai7_hit  <= 1'b0;
            boot_ack <= 1'b0;
            if (ram_load_cpu)
                cpu_ram_want <= 1'b0;

            case (state)
                S_IDLE: begin
                    cpu_dtack_n <= 1'b1;
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
                    if (!req_rw && !req_boot && prot_arm &&
                        (req_addr < 24'h000120)) begin
                        // AI7 "vector table write protection": a CPU write
                        // below $000120 while $600001 bit 2 is armed raises
                        // the level-7 autovector. A/B EXPERIMENT (P3):
                        // perform the write AND raise AI7. The OS's
                        // soft-reboot choreography at $824184 writes its
                        // reboot magic to $000002/$4/$6 through this exact
                        // path; blocking the write (previous behavior)
                        // leaves the NMI handler reading stale context and
                        // the boot derails at the same FLW/INT marks on
                        // every build. v12 and real hardware let the write
                        // land.
                        ai7_hit     <= 1'b1;
                        cpu_dtack_n <= 1'b0;
                        // fall through to the normal write path below by
                        // NOT consuming req_valid: the write proceeds via
                        // sel_ram/sel_flash handling in the same cycle.
                        if (sel_ram && !req_boot) begin
                            cpu_ram_want <= 1'b1;
                            state        <= S_WAITRAM;
                        end else if (sel_flash) begin
                            flash_addr  <= req_addr[21:0];
                            flash_wdata <= req_wdata;
                            flash_uds_n <= req_uds_n;
                            flash_lds_n <= req_lds_n;
                            flash_wr    <= 1'b1;
                            state       <= S_WAIT;
                        end else begin
                            cpu_dtack_n <= 1'b0;
                            state       <= S_DONE;
                        end
                    end else if (sel_ram && !req_boot) begin
                        // RAM — lives in SDRAM; queue the access and wait
                        // for the arbiter's completion pulse.
                        cpu_ram_want <= 1'b1;
                        state        <= S_WAITRAM;

                    end else if (sel_flash) begin
                        // FLASH via flash_ctrl (WSM + SDRAM), subject to
                        // the protection rules above.
                        if (!req_rw && (protect || req_boot_blk)) begin
                            // Dropped write: protection active (flash.c
                            // returns before the WSM sees the command) or
                            // boot block (hwp_put_byte returns before
                            // mem_put). Complete with no controller action.
                            cpu_dtack_n <= 1'b0;
                            state       <= S_DONE;
                        end else if (req_rw && protect && req_cert) begin
                            // Protected certificate / read-protected window:
                            // $14 per byte, flash array never consulted.
                            cpu_din     <= 16'h1414;
                            cpu_dtack_n <= 1'b0;
                            state       <= S_DONE;
                        end else begin
                            flash_addr  <= req_addr[21:0];
                            flash_wdata <= req_wdata;
                            flash_uds_n <= req_uds_n;
                            flash_lds_n <= req_lds_n;
                            if (req_rw)
                                flash_rd <= 1'b1;
                            else
                                flash_wr <= 1'b1;
                            state <= S_WAIT;
                        end

                    end else if (sel_io && !req_boot) begin
                        // I/O — register address/strobes for the next cycle.
                        // io_ports.sv reconstructs the even byte address as
                        // {addr, 1'b0}, so addr must be a WORD index
                        // (byte_offset / 2), not a raw byte offset.
                        // Passing req_addr[4:0] (byte offset, always even)
                        // would double every address: $600002 → addr=2 →
                        // {2,0}=4 → io1[4] instead of io1[2].  Worse, the
                        // lower byte of a word write to $600002 would land
                        // at io1[5] ($600005 = STOP), halting the CPU before
                        // any timer is configured.
                        if (sel_io1) begin
                            io_addr <= {3'd0, req_addr[5:1]};  // word idx (0-15) in 32-byte space
                            io_bank <= 2'd0;
                        end else if (sel_io2) begin
                            io_addr <= {2'd0, req_addr[7:1]};  // word idx (0-31) in 64-byte space
                            io_bank <= 2'd1;
                        end else begin
                            io_addr <= req_addr[8:1];           // word idx (0-127) in 256-byte space
                            io_bank <= 2'd2;
                        end
                        io_wdata <= req_wdata;
                        state <= S_IO;

                    end else begin
                        // Unmapped: the reference emulator (n-89 / TiEmu,
                        // mem89tm.c + mem.c) models every unused address as
                        // a byte filled with 0x14 — a word read returns
                        // $1414, a long $14141414. The OS's control flow
                        // depends on this value, so match it exactly.
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
                                // Header word -> boot FSM writes it to RAM
                                boot_ack <= 1'b1;
                            end else begin
                                cpu_din <= flash_rdata;
                            end
                        end
                        cpu_dtack_n <= 1'b0;
                        state <= S_DONE;
                    end
                end

                S_WAITRAM: begin
                    // Waiting for the SDRAM RAM access
                    if (cpu_ram_done) begin
                        if (req_rw)
                            cpu_din <= sd_rdata; // held by the controller
                        cpu_dtack_n <= 1'b0;
                        state       <= S_DONE;
                    end
                end

                S_DONE: begin
                    // Wait for the requester to end the cycle
                    if (req_boot || cpu_as_n) begin
                        cpu_dtack_n <= 1'b1;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Boot FSM
    // =========================================================================
    // B_WAIT   : wait until an OS image has been loaded into SDRAM and the
    //            SDRAM controller is initialized
    // B_DPASS  : request the "$P<n>" pass marker from dbg_uart
    // B_DREQ   : queue an SDRAM read of image word dump_idx (RAM slot)
    // B_DWAIT  : on completion wait for dbg_uart readiness, pulse dump_stb,
    //            advance (last word -> next pass or B_CLEAR)
    // B_CLEAR  : clear all RAM (TiEmu erases RAM upon reset) — one SDRAM
    //            write per word through the arbiter
    // B_COPY   : request a flash read of header word boot_idx (bus FSM)
    // B_CWAIT  : wait until the bus FSM delivers it (boot_ack)
    // B_RWRITE : write the header word into RAM, wait for completion
    // B_DONE   : hold boot_done high until a new image is loaded

    always @(posedge clk) begin
        if (reset || !rom_loaded) begin
            boot_state      <= B_WAIT;
            boot_done       <= 1'b0;
            clr_idx         <= 17'd0;
            boot_idx        <= 7'd0;
            boot_ram_want   <= 1'b0;
            boot_ram_flying <= 1'b0;
            boot_ram_addr   <= 25'd0;
            boot_ram_wdata  <= 16'd0;
            boot_ram_we     <= 1'b1;
            dump_pass_stb   <= 1'b0;
            dump_idx        <= 21'd0;
            dump_pass       <= 2'd0;
            dwait_rd        <= 1'b0;
            cmd_mem_r       <= 1'b0;
            cmd_start_r     <= 24'd0;
            cmd_left_r      <= 24'd0;
            cmd_total_w     <= 23'd0;
            dump_cmd_mode   <= 1'b0;
        end else begin
            // Default: deassert the one-cycle dump strobe
            dump_pass_stb <= 1'b0;

            if (ram_load_boot) begin
                boot_ram_want   <= 1'b0;
                boot_ram_flying <= 1'b1;
            end
            if (boot_ram_done)
                boot_ram_flying <= 1'b0;

            case (boot_state)
                B_WAIT: begin
                    if (init_done)
                        boot_state <= B_DPASS;
                end

                B_DPASS: begin
                    // Marker first; B_DREQ/B_DWAIT then wait for dump_rdy
                    // (low while dbg_uart shifts the "$P<n>" line out), so
                    // no word can overtake its own pass header.
                    if (dump_rdy) begin
                        dump_pass_stb <= 1'b1;
                        boot_state    <= B_DREQ;
                    end
                end

                B_DREQ: begin
                    if (!boot_ram_want && !boot_ram_flying) begin
                        boot_ram_want  <= 1'b1;
                        boot_ram_we    <= 1'b0;   // read
                        boot_ram_addr  <= {3'd0, dump_idx, 1'b0};
                        boot_ram_wdata <= 16'd0;
                    end
                    if (boot_ram_want || boot_ram_flying)
                        boot_state <= B_DWAIT;
                end

                B_DWAIT: begin
                    // boot_ram_done and dump_rd_data land together; give
                    // dwait_rd one cycle to register before the strobe.
                    if (boot_ram_done)
                        dwait_rd <= 1'b1;
                    if (dwait_rd && dump_rdy) begin
                        // dump_stb is combinational on this very
                        // condition, so the word is handed over in this
                        // cycle: dump_word = dump_rd_data.
                        dwait_rd <= 1'b0;
                        if (dump_idx == 21'h1FFFFF) begin
                            dump_idx <= 21'd0;
                            if (dump_pass == DUMP_PASSES - 2'd1)
                                boot_state <= B_CLEAR;
                            else begin
                                dump_pass  <= dump_pass + 2'd1;
                                boot_state <= B_DPASS;
                            end
                        end else begin
                            dump_idx  <= dump_idx + 21'd1;
                            boot_state <= B_DREQ;
                        end
                    end
                end

                B_CLEAR: begin
                    if (!boot_ram_want && !boot_ram_flying) begin
                        boot_ram_want  <= 1'b1;
                        boot_ram_we    <= 1'b1;   // write
                        boot_ram_addr  <= RAM_BASE + {7'd0, clr_idx, 1'b0};
                        boot_ram_wdata <= 16'd0;
                    end
                    if (boot_ram_done) begin
                        if (clr_idx == 17'd131071) begin
                            boot_state <= B_COPY;
                        end else begin
                            clr_idx <= clr_idx + 17'd1;
                        end
                    end
                end

                B_COPY: begin
                    // Wait for the bus FSM to pick up the request
                    if (state != S_IDLE)
                        boot_state <= B_CWAIT;
                end

                B_CWAIT: begin
                    if (boot_ack)
                        boot_state <= B_RWRITE;
                end

                B_RWRITE: begin
                    if (!boot_ram_want && !boot_ram_flying) begin
                        boot_ram_want  <= 1'b1;
                        boot_ram_we    <= 1'b1;   // write
                        boot_ram_addr  <= RAM_BASE + {17'd0, boot_idx, 1'b0};
                        boot_ram_wdata <= flash_rdata; // held by flash_ctrl
                    end
                    if (boot_ram_done) begin
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
                    // Host command dump: only accepted here (CPU released,
                    // no boot dump in flight). cmd_req arrives already
                    // validated/clamped by dbg_uart's parser.
                    if (cmd_req) begin
                        cmd_mem_r      <= cmd_mem;
                        cmd_start_r    <= cmd_start;
                        cmd_left_r     <= cmd_len;
                        cmd_total_w    <= cmd_len[23:1];
                        dump_cmd_mode  <= cmd_mem;
                        boot_state     <= B_CPASS;
                    end
                end

                // -----------------------------------------------------
                // Command dump (marker "$D"): same word/stream handshake
                // as the pre-boot dump, but over an arbitrary bounded
                // range and with the CPU running (dump reads take the
                // lowest-priority RAM slot; flash-image reads are plain
                // SDRAM reads of the 4 MB image area).
                B_CPASS: begin
                    if (dump_rdy) begin
                        dump_pass_stb <= 1'b1;
                        boot_state    <= B_CREQ;
                    end
                end

                B_CREQ: begin
                    if (!boot_ram_want && !boot_ram_flying) begin
                        boot_ram_want  <= 1'b1;
                        boot_ram_we    <= 1'b0;    // read
                        // lowest-priority RAM slot; CPU keeps running.
                        // dbg_uart clamps start+len to the region, so both
                        // sums below stay within their field widths.
                        if (cmd_mem_r)
                            boot_ram_addr <= RAM_BASE +
                                {7'd0, (cmd_start_r[17:1] + cmd_cur_w[16:0]), 1'b0};
                        else
                            boot_ram_addr <= {3'd0,
                                (cmd_start_r[21:1] + cmd_cur_w[20:0]), 1'b0};
                        boot_ram_wdata <= 16'd0;
                    end
                    if (boot_ram_want || boot_ram_flying)
                        boot_state <= B_CWAITW;
                end

                B_CWAITW: begin
                    // mirrors B_DWAIT: dwait_rd gives the grant FSM one cycle
                    // to latch dump_rd_data, then the combinational dump_stb
                    // hands the word over exactly when dbg_uart is idle.
                    if (boot_ram_done)
                        dwait_rd <= 1'b1;
                    if (dwait_rd && dump_rdy) begin
                        dwait_rd <= 1'b0;
                        if (cmd_left_r == 24'd2) begin
                            boot_state <= B_CEND;
                        end else begin
                            cmd_left_r <= cmd_left_r - 24'd2;
                            boot_state <= B_CREQ;
                        end
                    end
                end

                B_CEND: begin
                    boot_state <= B_DONE;
                end

                default: boot_state <= B_WAIT;
            endcase
        end
    end

endmodule
