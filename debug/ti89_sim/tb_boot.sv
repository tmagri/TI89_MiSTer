`timescale 1ns / 1ps
//
// tb_boot.sv — full-boot simulation of the TI-89 core
//
// Mirrors TI89.sv wiring (cpu_wrapper + mem_ctrl + flash_ctrl + io_ports
// + timer_int + keyboard + lcd_ctrl) but replaces:
//   - hps_io / rom_loader : flash SDRAM is preloaded from flash.hex
//                           (the real .89u converted by convert89u.py,
//                           same algorithm as references/n-89 convert.rs)
//   - sdram controller    : ideal ~3-cycle port-A model
//
// SDRAM layout (matches mem_ctrl):
//   byte $000000-$3FFFFF : flash window (word 0x000000-0x1FFFFF)
//   byte $400000-$43FFFF : calculator RAM (word 0x200000-0x21FFFF)
// Purpose: find where (if anywhere) the real OS boot stalls.
//
module tb_boot;

    // =========================================================================
    // Clock / reset
    // =========================================================================
    reg clk = 0;
    always #8 clk = ~clk;  // 62.5 MHz

    reg reset      = 1;
    reg rom_loaded = 0;

    // =========================================================================
    // Nets (same names as TI89.sv)
    // =========================================================================
    wire        boot_done;
    wire        cpu_reset = reset | ~boot_done;

    wire [23:1] cpu_addr;
    wire [15:0] cpu_dout;
    wire [15:0] cpu_din;
    wire        cpu_as_n, cpu_uds_n, cpu_lds_n, cpu_rw_n;
    wire        cpu_dtack_n;
    wire  [2:0] cpu_fc;

    wire [21:0] flash_addr;
    wire [15:0] flash_wdata;
    wire        flash_rd, flash_wr, flash_uds_n, flash_lds_n;
    wire [15:0] flash_rdata;
    wire        flash_ready;

    // SDRAM port A (mem_ctrl arbiter -> SDRAM)
    wire [24:0] sd_addr;
    wire [15:0] sd_wdata;
    wire        sd_rd, sd_wr, sd_uds_n, sd_lds_n;
    wire [15:0] sd_rdata;
    wire        sd_ready;

    // flash_ctrl's SDRAM client (-> mem_ctrl arbiter)
    wire [21:0] fl_addr;
    wire [15:0] fl_wdata;
    wire        fl_rd, fl_wr, fl_uds_n, fl_lds_n;
    wire [15:0] fl_rdata;
    wire        fl_ready;

    wire [17:0] lcd_ram_addr;
    wire [15:0] lcd_ram_data;
    wire        lcd_ram_req;
    wire        lcd_ram_ack;

    reg         sim_init_done = 0; // SDRAM "initialized" for the boot FSM

    wire  [7:0] io_addr;
    wire [15:0] io_wdata;
    wire [15:0] io_rdata;
    wire        io_rd, io_wr;
    wire  [1:0] io_bank;
    wire        io_uds_n, io_lds_n;
    wire        protect;
    wire        prot_arm;
    wire        ai7_hit;

    // =========================================================================
    // REAL SDRAM controller + REAL chip model (P3: reproduces the
    // hardware-only read defect). The behavioral sdmem model cannot.
    // Chip samples commands on SDRAM_CLK = ~clk (half a period after the
    // controller's launch edge), exactly like the -3000 ps phase on hw.
    // Flash image at chip word 0 (sdram_chip preloads flash.hex),
    // calculator RAM at word 0x200000.
    // =========================================================================
    wire        clk_sdram = ~clk;
    wire        SDRAM_CLK, SDRAM_CKE, SDRAM_DQ_OE, SDRAM_DQML, SDRAM_DQMH;
    wire        SDRAM_nCS, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nWE;
    wire [12:0] SDRAM_A;
    wire [1:0]  SDRAM_BA;
    wire [15:0] SDRAM_DQ_OUT, chip_dq_out;
    wire        chip_dq_oe;
    wire [15:0] SDRAM_DQ = SDRAM_DQ_OE ? SDRAM_DQ_OUT :
                          (chip_dq_oe ? chip_dq_out : 16'hZZZZ);
    wire        sdram_init_done;

    sdram u_sdram (
        .clk(clk), .clk_sdram(clk_sdram), .reset(reset),
        .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA),
        .SDRAM_DQ_IN(SDRAM_DQ), .SDRAM_DQ_OUT(SDRAM_DQ_OUT),
        .SDRAM_DQ_OE(SDRAM_DQ_OE), .SDRAM_DQML(SDRAM_DQML),
        .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_nCS(SDRAM_nCS),
        .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nWE(SDRAM_nWE),
        .a_addr(sd_addr), .a_wdata(sd_wdata), .a_rd(sd_rd), .a_wr(sd_wr),
        .a_uds_n(sd_uds_n), .a_lds_n(sd_lds_n), .a_rdata(sd_rdata),
        .a_ready(sd_ready),
        .b_addr(21'd0), .b_wdata(16'd0), .b_wr(1'b0), .b_wait(),
        .init_done(sdram_init_done)
    );

    sdram_chip u_chip (
        .CLK(SDRAM_CLK), .CKE(SDRAM_CKE), .A(SDRAM_A), .BA(SDRAM_BA),
        .DQ_IN(SDRAM_DQ_OUT), .DQ_OUT(chip_dq_out), .DQ_OE(chip_dq_oe),
        .DQML(SDRAM_DQML), .DQMH(SDRAM_DQMH), .nCS(SDRAM_nCS),
        .nRAS(SDRAM_nRAS), .nCAS(SDRAM_nCAS), .nWE(SDRAM_nWE)
    );

    // =========================================================================
    // DUT instances (wiring copied from TI89.sv)
    // =========================================================================

    mem_ctrl u_mem (
        .clk(clk),
        .reset(reset),
        .rom_loaded(rom_loaded),
        .init_done(sim_init_done && sdram_init_done),
        .boot_done(boot_done),
        .cpu_addr(cpu_addr),
        .cpu_dout(cpu_dout),
        .cpu_din(cpu_din),
        .cpu_as_n(cpu_as_n),
        .cpu_uds_n(cpu_uds_n),
        .cpu_lds_n(cpu_lds_n),
        .cpu_rw_n(cpu_rw_n),
        .cpu_dtack_n(cpu_dtack_n),
        .lcd_ram_addr(lcd_ram_addr),
        .lcd_ram_data(lcd_ram_data),
        .lcd_ram_req(lcd_ram_req),
        .lcd_ram_ack(lcd_ram_ack),
        .flash_addr(flash_addr),
        .flash_wdata(flash_wdata),
        .flash_rdata(flash_rdata),
        .flash_rd(flash_rd),
        .flash_wr(flash_wr),
        .flash_uds_n(flash_uds_n),
        .flash_lds_n(flash_lds_n),
        .flash_ready(flash_ready),
        .fl_sd_addr(fl_addr),
        .fl_sd_wdata(fl_wdata),
        .fl_sd_rd(fl_rd),
        .fl_sd_wr(fl_wr),
        .fl_sd_uds_n(fl_uds_n),
        .fl_sd_lds_n(fl_lds_n),
        .fl_sd_rdata(fl_rdata),
        .fl_sd_ready(fl_ready),
        .sd_addr(sd_addr),
        .sd_wdata(sd_wdata),
        .sd_rd(sd_rd),
        .sd_wr(sd_wr),
        .sd_uds_n(sd_uds_n),
        .sd_lds_n(sd_lds_n),
        .sd_rdata(sd_rdata),
        .sd_ready(sd_ready),
        .io_addr(io_addr),
        .io_wdata(io_wdata),
        .io_rdata(io_rdata),
        .io_rd(io_rd),
        .io_wr(io_wr),
        .io_bank(io_bank),
        .io_uds_n(io_uds_n),
        .io_lds_n(io_lds_n),
        .protect(protect),
        .prot_arm(prot_arm),
        .ai7_hit(ai7_hit),
        // P3: dump/command ports (the TB has no dbg_uart; the pre-boot
        // dump must see an always-ready producer or the boot never starts)
        .dump_rdy(1'b1),
        .dump_stb(),
        .dump_word(),
        .dump_pass_stb(),
        .dump_active(),
        .cmd_req(1'b0),
        .cmd_mem(1'b0),
        .cmd_start(24'd0),
        .cmd_len(24'd0),
        .dump_cmd_mode()
    );

    flash_ctrl u_flash (
        .clk(clk),
        .reset(reset),
        .flash_addr(flash_addr),
        .flash_wdata(flash_wdata),
        .flash_rd(flash_rd),
        .flash_wr(flash_wr),
        .flash_uds_n(flash_uds_n),
        .flash_lds_n(flash_lds_n),
        .flash_rdata(flash_rdata),
        .flash_ready(flash_ready),
        .sd_addr(fl_addr),
        .sd_wdata(fl_wdata),
        .sd_rd(fl_rd),
        .sd_wr(fl_wr),
        .sd_uds_n(fl_uds_n),
        .sd_lds_n(fl_lds_n),
        .sd_rdata(fl_rdata),
        .sd_ready(fl_ready)
    );

    // ---- I/O ports / timer / keyboard ----
    wire  [9:0] kbd_row_mask;
    wire   [7:0] kbd_col_data;
    wire         on_key;
    wire   [7:0] timer_ctrl;
    wire   [7:0] timer_init;
    wire   [7:0] timer_value;
    wire         timer_load;
    wire  [15:0] lcd_base_addr;
    wire   [7:0] lcd_log_w;
    wire   [7:0] lcd_log_h;
    wire   [3:0] lcd_contrast;
    wire         lcd_on;
    wire         lcd_vsync;
    wire         cpu_stop;
    wire   [4:0] stop_mask;
    wire         ack_ai2;
    wire         ack_ai6;

    io_ports u_io (
        .clk(clk),
        .reset(cpu_reset),
        .addr(io_addr),
        .wdata(io_wdata),
        .rdata(io_rdata),
        .rd(io_rd),
        .wr(io_wr),
        .bank(io_bank),
        .uds_n(io_uds_n),
        .lds_n(io_lds_n),
        .protect(protect),
        .kbd_row_mask(kbd_row_mask),
        .kbd_col_data(kbd_col_data),
        .on_key(on_key),
        .timer_ctrl(timer_ctrl),
        .timer_init(timer_init),
        .timer_value(timer_value),
        .lcd_addr(lcd_base_addr),
        .lcd_log_w(lcd_log_w),
        .lcd_log_h(lcd_log_h),
        .lcd_contrast(lcd_contrast),
        .lcd_on(lcd_on),
        .lcd_vsync(lcd_vsync),
        .cpu_stop(cpu_stop),
        .stop_mask(stop_mask),
        .ack_ai2(ack_ai2),
        .ack_ai6(ack_ai6),
        .timer_load(timer_load),
        .prot_arm(prot_arm)
    );

    wire [2:0] ipl;
    wire [7:0] int_pend;
    wire       kbd_int;
    wire       on_key_press;
    wire       intack_edge;

    timer_int u_tmr (
        .clk(clk),
        .reset(cpu_reset),
        .timer_ctrl(timer_ctrl),
        .timer_init(timer_init),
        .timer_load(timer_load),
        .timer_value(timer_value),
        .kbd_int(kbd_int),
        .on_key_press(on_key_press),
        .ai7_set(ai7_hit),
        .ack_ai2(ack_ai2),
        .ack_ai6(ack_ai6),
        .intack(intack_edge),
        .ipl(ipl),
        .int_pend(int_pend)
    );

    keyboard u_kbd (
        .clk(clk),
        .reset(cpu_reset),
        .ps2_key(11'd0),
        .row_mask(kbd_row_mask),
        .col_data(kbd_col_data),
        .on_key(on_key),
        .on_key_press(on_key_press),
        .kbd_int(kbd_int)
    );

    // ---- Interrupt acknowledge + STOP logic (from TI89.sv) ----
    // VPA must stay asserted for the WHOLE IACK cycle: the pending-flag
    // clear in timer_int drops ipl to 0 mid-cycle; without the latch the
    // combinational gate deasserts VPA and fx68k reads a garbage vector
    // off the bus instead of autovectoring (the crash that caused the
    // "AI5 storm" — AI1 ack sampled vector 20, [$50]=0, PC ran away).
    wire intack_raw = (cpu_fc == 3'b111) && !cpu_as_n;
    reg intack_latch = 0;
    always @(posedge clk) begin
        if (cpu_as_n)
            intack_latch <= 1'b0;
        else if (intack_raw && (ipl != 3'd0))
            intack_latch <= 1'b1;
    end
    wire intack = intack_raw && (ipl != 3'd0 || intack_latch);
    wire vpa_n  = ~intack;

    // Run-14: PC tracker — remember the last two program-space fetches so
    // each logged I/O access can be attributed to the instruction (and its
    // extension word) that caused it. FC: 010=user prog, 110=super prog.
    reg cpu_as_q = 1'b1;
    always @(posedge clk) cpu_as_q <= cpu_as_n;
    wire cyc_start = cpu_as_q && !cpu_as_n;
    reg [23:0] last_pc1 = 0, last_pc2 = 0;
    always @(posedge clk) begin
        if (cyc_start && cpu_fc[1] && !cpu_fc[0]) begin
            last_pc2 <= last_pc1;
            last_pc1 <= {cpu_addr, 1'b0};
        end
    end

    reg intack_q = 0;
    always @(posedge clk) intack_q <= intack;
    assign intack_edge = intack && !intack_q;

    reg  stopped = 0;
    wire wake = int_pend[7] | int_pend[6] | (|(int_pend[5:1] & stop_mask));

    always @(posedge clk) begin
        if (reset || !boot_done)
            stopped <= 1'b0;
        else if (cpu_stop)
            stopped <= 1'b1;
        else if (wake)
            stopped <= 1'b0;
    end

    wire cpu_halt = stopped && cpu_as_n;

    cpu_wrapper u_cpu (
        .clk(clk),
        .reset(cpu_reset),
        .cpu_en(1'b1),
        .halt(cpu_halt),
        .cpu_addr(cpu_addr),
        .cpu_dout(cpu_dout),
        .cpu_din(cpu_din),
        .cpu_as_n(cpu_as_n),
        .cpu_uds_n(cpu_uds_n),
        .cpu_lds_n(cpu_lds_n),
        .cpu_rw_n(cpu_rw_n),
        .cpu_dtack_n(cpu_dtack_n),
        .cpu_fc(cpu_fc),
        .cpu_E(),
        .cpu_vma_n(),
        .ipl(ipl),
        .vpa_n(vpa_n),
        .cpu_reset_out_n(),
        .cpu_halted_n()
    );

    wire pixel_out;
    wire pixel_valid;

    lcd_ctrl u_lcd (
        .clk(clk),
        .reset(reset),
        .lcd_base_addr(lcd_base_addr),
        .lcd_log_w(lcd_log_w),
        .lcd_log_h(lcd_log_h),
        .lcd_contrast(lcd_contrast),
        .lcd_on(lcd_on),
        .ram_addr(lcd_ram_addr),
        .ram_data(lcd_ram_data),
        .ram_req(lcd_ram_req),
        .ram_ack(lcd_ram_ack),
        .pixel_out(pixel_out),
        .pixel_valid(pixel_valid),
        .hsync(),
        .vsync(lcd_vsync),
        .hblank(),
        .vblank(),
        .pixel_x(),
        .pixel_y()
    );

    // =========================================================================
    // Boot sequence
    // =========================================================================
    initial begin
        repeat (100) @(posedge clk);
        @(negedge clk); reset = 0;
        repeat (10) @(negedge clk);
        sim_init_done = 1; // SDRAM "initialized" -> boot FSM may clear RAM
        repeat (5) @(negedge clk);
        rom_loaded = 1;    // "image already loaded" -> boot FSM runs
    end

    // =========================================================================
    // Bus monitor: ring buffer of the last 8192 completed transfers
    // =========================================================================
    reg [23:0] ev_addr [0:8191];
    reg [15:0] ev_data [0:8191];
    reg  [2:0] ev_ctl  [0:8191]; // {rw, uds_n, lds_n}
    reg  [2:0] ev_fc   [0:8191];
    reg [31:0] ev_cyc  [0:8191];
    reg [12:0] ev_idx = 0;

    reg [31:0] cyc = 0;
    reg [31:0] n_xfers = 0;
    reg [31:0] last_xfer_cyc = 0;

    always @(posedge clk) cyc <= cyc + 1;

    reg        as_prev = 1;
    reg [23:0] cap_addr = 0;
    reg [15:0] cap_data = 0;
    reg  [2:0] cap_ctl = 0;
    reg  [2:0] cap_fc = 0;
    reg        cap_v = 0;

    wire xfer = !cpu_as_n && !cpu_dtack_n;

    // Detailed trace of the first transfers (for post-mortem analysis)
    integer tf;
    initial tf = $fopen("xfer_trace.log", "w");

    // Run-8: post-trigger dense trace (armed by the vector-table watchpoint)
    integer tg;
    initial tg = $fopen("trig2_trace.log", "w");
    reg        armed = 0;      // dense trace enabled after a late vec write
    reg [31:0] n_trig2 = 0;

    always @(posedge clk) begin
        as_prev <= cpu_as_n;
        if (xfer) begin
            cap_addr <= {cpu_addr, 1'b0};
            cap_data <= cpu_rw_n ? cpu_din : cpu_dout;
            cap_ctl  <= {cpu_rw_n, cpu_uds_n, cpu_lds_n};
            cap_fc   <= cpu_fc;
            cap_v    <= 1'b1;
        end
        if (cpu_as_n && !as_prev && cap_v) begin
            ev_addr[ev_idx] <= cap_addr;
            ev_data[ev_idx] <= cap_data;
            ev_ctl[ev_idx]  <= cap_ctl;
            ev_fc[ev_idx]   <= cap_fc;
            ev_cyc[ev_idx]  <= cyc;
            ev_idx          <= ev_idx + 13'd1;
            n_xfers         <= n_xfers + 1;
            last_xfer_cyc   <= cyc;
            cap_v           <= 1'b0;
            if (n_xfers < 32'd1600000)
                $fwrite(tf, "%0d %s %06x fc=%03b uds=%b lds=%b data=%04x\n",
                        cyc, cap_ctl[2] ? "RD" : "WR", cap_addr, cap_fc,
                        cap_ctl[1], cap_ctl[0], cap_data);
            if (armed && n_trig2 < 32'd2000000) begin
                n_trig2 <= n_trig2 + 1;
                $fwrite(tg, "%0d %s %06x fc=%03b uds=%b lds=%b data=%04x\n",
                        cyc, cap_ctl[2] ? "RD" : "WR", cap_addr, cap_fc,
                        cap_ctl[1], cap_ctl[0], cap_data);
            end
        end
    end

    // Dump the ring buffer (last 8192 transfers) to a file.
    task dump_ring_file;
        input [8*24-1:0] fname;
        integer rf, k;
        reg [12:0] base;
        begin
            rf = $fopen(fname, "w");
            base = ev_idx; // oldest = current index (wrapped)
            for (k = 0; k < 8192; k = k + 1) begin : dmp
                reg [12:0] i;
                i = base + k;
                if (ev_cyc[i] != 0)
                    $fwrite(rf, "%0d %s %06x fc=%03b uds=%b lds=%b data=%04x\n",
                            ev_cyc[i], ev_ctl[i][2] ? "RD" : "WR",
                            ev_addr[i], ev_fc[i],
                            ev_ctl[i][1], ev_ctl[i][0], ev_data[i]);
            end
            $fclose(rf);
            $display("cyc=%0d: ring dump -> %0s", cyc, fname);
        end
    endtask

    task dump_ring;
        integer k;
        reg [12:0] base;
        begin
            base = ev_idx; // oldest = current index (wrapped)
            for (k = 0; k < 8192; k = k + 1) begin : dmp2
                reg [12:0] i;
                i = base + k;
                if (ev_cyc[i] != 0 && k >= 8192-64)
                    $display("  %08d cyc  %s %06x  uds=%b lds=%b  data=%04x",
                             ev_cyc[i], ev_ctl[i][2] ? "RD" : "WR",
                             ev_addr[i], ev_ctl[i][1], ev_ctl[i][0],
                             ev_data[i]);
            end
        end
    endtask

    // =========================================================================
    // Event log file (IO accesses, LCD status)
    // =========================================================================
    integer lf;
    initial lf = $fopen("boot_events.log", "w");

    always @(posedge clk) begin
        if (u_mem.io_wr)
            $fwrite(lf, "%0t cyc=%0d IO WR bank=%0d addr=$%02x data=$%04x uds=%b lds=%b pc=%06x/%06x\n",
                    $time, cyc, u_mem.io_bank, u_mem.io_addr, u_mem.io_wdata,
                    u_mem.io_uds_n, u_mem.io_lds_n, last_pc1, last_pc2);
        if (u_mem.io_rd)
            $fwrite(lf, "%0t cyc=%0d IO RD bank=%0d addr=$%02x -> $%04x pc=%06x/%06x\n",
                    $time, cyc, u_mem.io_bank, u_mem.io_addr, u_mem.io_rdata,
                    last_pc1, last_pc2);
    end

    reg lcd_on_prev = 0;
    always @(posedge clk) begin
        lcd_on_prev <= lcd_on;
        if (lcd_on !== lcd_on_prev) begin
            $display("cyc=%0d: lcd_on -> %b (base=$%04x w=%0d h=%0d contrast=%0d)",
                     cyc, lcd_on, lcd_base_addr, lcd_log_w, lcd_log_h, lcd_contrast);
            $fwrite(lf, "%0t cyc=%0d lcd_on -> %b\n", $time, cyc, lcd_on);
        end
    end

    // Progress every 64K transfers
    reg [31:0] xfer_mile = 0;
    always @(posedge clk) begin
        if (n_xfers >= xfer_mile + 32'h10000) begin
            xfer_mile <= n_xfers;
            $display("cyc=%0d: %0d bus transfers, last addr $%06x",
                     cyc, n_xfers, cap_addr);
        end
    end

    // =========================================================================
    // Instrumentation counters
    // =========================================================================
    reg [31:0] n_intack   = 0;  // interrupt acknowledges delivered
    reg [31:0] n_stop     = 0;  // entries into the STOP state
    reg [31:0] n_flash_wr = 0;  // flash-window write cycles
    reg [31:0] n_fb_wr    = 0;  // RAM writes into the $4000-$7FFF window
    reg [31:0] n_ai7      = 0;  // AI7 (low-RAM write protection) fires
    reg [23:0] last_flash_addr = 0;  // most recent flash-window write addr

    // RAM writes now appear on the SDRAM port. The $4000-$7FFF RAM window
    // (framebuffer region) maps to SDRAM bytes $404000-$407FFF.
    wire fb_wr = u_mem.sd_wr &&
                 (u_mem.sd_addr >= 25'h404000) &&
                 (u_mem.sd_addr <= 25'h407FFF);

    reg stopped_prev = 0;
    always @(posedge clk) begin
        if (intack_edge) begin
            n_intack <= n_intack + 1;
            $fwrite(lf, "%0t cyc=%0d INTACK #%0d ipl=%b int_pend=%02x\n",
                    $time, cyc, n_intack, ipl, int_pend);
        end
        stopped_prev <= stopped;
        if (stopped && !stopped_prev) n_stop <= n_stop + 1;
        if (fb_wr) n_fb_wr <= n_fb_wr + 1;
        if (ai7_hit) begin
            n_ai7 <= n_ai7 + 1;
            if (n_ai7 < 32'd2048)
                $fwrite(lf, "%0t cyc=%0d AI7 HIT #%0d (low-RAM write, prot armed)\n",
                        $time, cyc, n_ai7 + 1);
        end
        if (u_mem.flash_wr) begin
            n_flash_wr <= n_flash_wr + 1;
            last_flash_addr <= u_mem.flash_addr;
            // Log the first 256 writes plus a 4-write sample every 64k,
            // so the long descending flash pass stays visible.
            if (n_flash_wr < 32'd256 || (n_flash_wr[15:0] < 16'd4))
                $fwrite(lf, "%0t cyc=%0d FLASH WR addr=$%06x data=$%04x uds=%b lds=%b\n",
                        $time, cyc, u_mem.flash_addr, u_mem.flash_wdata,
                        u_mem.flash_uds_n, u_mem.flash_lds_n);
        end
    end

    // =========================================================================
    // Run-8: vector-table watchpoint. Every SDRAM write into calculator-RAM
    // bytes $00-$7F (the 68000 vector table, VBR=0) is logged; a write after
    // 30M cycles (i.e. after boot-time setup) dumps the 8192-transfer ring
    // and arms the dense post-trigger trace.
    // =========================================================================
    integer vf;
    initial vf = $fopen("vec_watch.log", "w");

    wire vec_wr = u_mem.sd_wr &&
                  (u_mem.sd_addr >= 25'h400000) &&
                  (u_mem.sd_addr <= 25'h40007F);
    wire low_wr = u_mem.sd_wr &&
                  (u_mem.sd_addr >= 25'h400080) &&
                  (u_mem.sd_addr <= 25'h4000FF) &&
                  (cyc > 32'd100000000);

    reg vec_tripped = 0;

    // Run-10: watch the OS stack-pointer variable at RAM $80B0 (SDRAM
    // byte $4080B0) and its neighbors. Every write is logged with cycle
    // and data. The FIRST non-zero write (= the stack-top sizing commit)
    // dumps the ring buffer and arms the dense trace, so the sizing
    // context is captured.
    wire spvar_wr = u_mem.sd_wr &&
                    ((u_mem.sd_addr == 25'h4080B0) ||
                     (u_mem.sd_addr == 25'h4080AC) ||
                     (u_mem.sd_addr == 25'h4080B2) ||
                     (u_mem.sd_addr == 25'h4080AE));

    reg spvar_tripped = 0;

    always @(posedge clk) begin
        if (spvar_wr) begin
            $fwrite(vf, "%0t cyc=%0d SPVAR WR addr=$%06x data=$%04x uds=%b lds=%b cpu_as_n=%b\n",
                    $time, cyc, u_mem.sd_addr - 25'h400000, u_mem.sd_wdata,
                    u_mem.sd_uds_n, u_mem.sd_lds_n, cpu_as_n);
            if (!spvar_tripped && (u_mem.sd_wdata != 16'd0)) begin
                spvar_tripped <= 1'b1;
                armed         <= 1'b1;
                $display("cyc=%0d: *** FIRST NON-ZERO SPVAR WRITE: addr=$%06x data=$%04x ***",
                         cyc, u_mem.sd_addr - 25'h400000, u_mem.sd_wdata);
                dump_ring_file("spvar_ring.log");
            end
        end
    end

    always @(posedge clk) begin
        if (vec_wr) begin
            $fwrite(vf, "%0t cyc=%0d VEC WR byte=$%03x data=$%04x uds=%b lds=%b cpu_as_n=%b intack=%b\n",
                    $time, cyc, u_mem.sd_addr - 25'h400000, u_mem.sd_wdata,
                    u_mem.sd_uds_n, u_mem.sd_lds_n, cpu_as_n, intack);
            if (cyc > 32'd30000000 && !vec_tripped) begin
                vec_tripped <= 1'b1;
                armed       <= 1'b1;
                $display("cyc=%0d: *** VECTOR TABLE WRITE after boot: byte=$%03x data=$%04x ***",
                         cyc, u_mem.sd_addr - 25'h400000, u_mem.sd_wdata);
                dump_ring_file("vec_ring.log");
            end
        end
        if (low_wr)
            $fwrite(vf, "%0t cyc=%0d LOW WR byte=$%03x data=$%04x uds=%b lds=%b cpu_as_n=%b\n",
                    $time, cyc, u_mem.sd_addr - 25'h400000, u_mem.sd_wdata,
                    u_mem.sd_uds_n, u_mem.sd_lds_n, cpu_as_n);
    end

    // =========================================================================
    // hwprot observation (stealth windows + protect transitions)
    // =========================================================================
    reg        protect_q = 0;
    reg [31:0] n_prot_chg  = 0; // protect flag transitions
    reg [31:0] n_en_rd     = 0; // reads  of $1C0000-$1FFFFF (enable window)
    reg [31:0] n_en_wr     = 0; // writes of $1C0000-$1FFFFF (enable window)
    reg [31:0] n_auth      = 0; // accesses to the authorization windows
    reg [31:0] n_cert_rd   = 0; // reads of certificate windows
    reg [31:0] n_fl_wr_drop= 0; // flash writes dropped (protect / boot block)

    // Flash write dropped in S_ACCESS by protection or the boot block
    wire fl_wr_drop = (u_mem.state == 3'd1) && u_mem.sel_flash &&
                      !u_mem.req_rw && (u_mem.protect || u_mem.req_boot_blk);

    always @(posedge clk) begin
        protect_q <= u_mem.protect;
        if (fl_wr_drop) n_fl_wr_drop <= n_fl_wr_drop + 1;

        if (u_mem.protect !== protect_q) begin
            n_prot_chg <= n_prot_chg + 1;
            $display("cyc=%0d: PROTECT -> %b", cyc, u_mem.protect);
            if (n_prot_chg < 32'd512)
                $fwrite(lf, "%0t cyc=%0d PROTECT -> %b\n",
                        $time, cyc, u_mem.protect);
        end
        if (u_mem.cyc_start && u_mem.hwp_en) begin
            if (u_mem.cpu_rw_n) n_en_rd <= n_en_rd + 1;
            else                n_en_wr <= n_en_wr + 1;
            if (n_en_rd + n_en_wr < 32'd8192)
                $fwrite(lf, "%0t cyc=%0d HWP-EN %s $%06x (a2=%0d)\n",
                        $time, cyc, u_mem.cpu_rw_n ? "RD" : "WR",
                        u_mem.hwp_addr, u_mem.access2);
        end
        if (u_mem.cyc_start && u_mem.hwp_auth) begin
            n_auth <= n_auth + 1;
            if (n_auth < 32'd512)
                $fwrite(lf, "%0t cyc=%0d HWP-AUTH %s $%06x (a2=%0d)\n",
                        $time, cyc, u_mem.cpu_rw_n ? "RD" : "WR",
                        u_mem.hwp_addr, u_mem.access2);
        end
        if (u_mem.cyc_start && u_mem.hwp_cert && u_mem.cpu_rw_n) begin
            n_cert_rd <= n_cert_rd + 1;
            if (n_cert_rd < 32'd512)
                $fwrite(lf, "%0t cyc=%0d HWP-CERT RD $%06x protect=%b\n",
                        $time, cyc, u_mem.hwp_addr, u_mem.protect);
        end
    end

    // Progress report every 20M cycles
    reg [31:0] prog_cyc = 0;
    always @(posedge clk) begin
        if (cyc >= prog_cyc + 32'd20000000) begin
            prog_cyc <= cyc;
            $display("cyc=%0d: xfers=%0d last=$%06x flash_wr=%0d lastfl=$%06x fb_wr=%0d intack=%0d stops=%0d stopped=%b lcd_on=%b protect=%b",
                     cyc, n_xfers, cap_addr, n_flash_wr, last_flash_addr,
                     n_fb_wr, n_intack, n_stop, stopped, lcd_on, u_mem.protect);
        end
    end

    // Fine-grained progress (1M) across the draw-collapse / scan / sweep
    // decision window so transition points can be pinned to +-1M cycles.
    reg [31:0] prog_fine = 0;
    always @(posedge clk) begin
        if (cyc >= 32'd70000000 && cyc < 32'd175000000 &&
            cyc >= prog_fine + 32'd1000000) begin
            prog_fine <= cyc;
            $display("fine cyc=%0d: xfers=%0d last=$%06x fb_wr=%0d intack=%0d protect=%b",
                     cyc, n_xfers, cap_addr, n_fb_wr, n_intack, u_mem.protect);
        end
    end

    // =========================================================================
    // Watchdogs
    // =========================================================================
    initial begin
        wait (boot_done === 1'b1);
        $display("cyc=%0d: boot_done, CPU released", cyc);
        $fwrite(lf, "%0t cyc=%0d boot_done\n", $time, cyc);
    end

    reg [31:0] lcd_on_cyc = 0;
    always @(posedge clk) begin
        if (lcd_on && lcd_on_cyc == 0)
            lcd_on_cyc <= cyc;
    end

    // Dump all 256KB of RAM for offline framebuffer analysis.
    // RAM now lives in SDRAM at byte $400000 (word 0x200000).
    task dump_ram;
        input [8*24-1:0] fname;
        integer df, i;
        begin
            df = $fopen(fname, "w");
            for (i = 0; i < 131072; i = i + 1)
                $fwrite(df, "%04x\n", u_chip.mem[32'h200000 + i]);
            $fclose(df);
            $display("cyc=%0d: RAM dump -> %0s (256KB)", cyc, fname);
        end
    endtask

    // Run-11: deep-stack watchpoint. The boot SSP is $4C00; every CPU
    // write below $4C00 after 10M cycles (boot clear/copy excluded) is a
    // low-RAM variable or a stack push. Logged until the cap so the
    // descent of SP from $4C00 into the vector table can be followed.
    integer sf;
    initial sf = $fopen("stack_watch.log", "w");
    reg [31:0] n_stk = 0;

    wire deep_wr = u_mem.sd_wr && !cpu_as_n &&
                   (u_mem.sd_addr >= 25'h400100) &&
                   (u_mem.sd_addr <  25'h404C00) &&
                   (cyc > 32'd10000000);

    always @(posedge clk) begin
        if (deep_wr && n_stk < 32'd200000) begin
            n_stk <= n_stk + 1;
            $fwrite(sf, "cyc=%0d DEEP WR addr=$%06x data=$%04x uds=%b lds=%b\n",
                    cyc, u_mem.sd_addr - 25'h400000, u_mem.sd_wdata,
                    u_mem.sd_uds_n, u_mem.sd_lds_n);
        end
    end

    // Checkpoint RAM snapshots before the AI7 storm (boot SSP = $4C00):
    // 40M and 60M.
    reg [2:0] cp_flags = 0;
    always @(posedge clk) begin
        if (cyc >= 32'd40000000 && cp_flags[0] == 0) begin
            cp_flags[0] <= 1'b1;
            dump_ram("ram_cp40.hex");
        end
        if (cyc >= 32'd60000000 && cp_flags[1] == 0) begin
            cp_flags[1] <= 1'b1;
            dump_ram("ram_cp60.hex");
        end
        // Run-13: second snapshot ~5.5 LCD frames before END to test the
        // home-screen speckle: if the pattern DIFFERS from ram_dump.hex
        // it is flicker grayscale (benign, matches real HW); if IDENTICAL
        // it is a static corrupted draw.
        if (cyc >= 32'd599802000 && cp_flags[2] == 0) begin
            cp_flags[2] <= 1'b1;
            dump_ram("ram_dump_prev.hex");
        end
    end

    task finish_dump;
        input [8*40-1:0] why;
        begin
            $display("%0s at cyc=%0d (%0d transfers)", why, cyc, n_xfers);
            $display("stopped=%b lcd_on=%b ipl=%b int_pend=%02x base=$%04x w=%0d h=%0d",
                     stopped, lcd_on, ipl, int_pend, lcd_base_addr, lcd_log_w,
                     lcd_log_h);
            $display("counters: intack=%0d stops=%0d flash_wr=%0d fb_wr=%0d ai7=%0d",
                     n_intack, n_stop, n_flash_wr, n_fb_wr, n_ai7);
            $display("hwprot: protect=%b chg=%0d en_rd=%0d en_wr=%0d auth=%0d cert_rd=%0d fl_drop=%0d",
                     u_mem.protect, n_prot_chg, n_en_rd, n_en_wr, n_auth,
                     n_cert_rd, n_fl_wr_drop);
            $display("Last bus activity:");
            dump_ring();
            dump_ram("ram_dump.hex");
            $fclose(lf); $fclose(tf); $fclose(tg); $fclose(vf); $fclose(sf);
            $finish;
        end
    endtask

    // =========================================================================
    // P3 derail trigger: the hardware derails into the $1414 sweep with PC
    // in unmapped $1xxxxx. Catch the FIRST program fetch (fc=010/110) that
    // lands outside every valid map (RAM $0-$3FFFF, mirrors
    // $200000-$23FFFF / $400000-$43FFFF, flash $800000-$BFFFFF), dump the
    // full 8192-entry bus ring + RAM, and stop.
    // =========================================================================
    wire [23:0] fetch_b = {cpu_addr, 1'b0};
    wire derail_fetch = boot_done && cyc_start && cpu_fc[1] && !cpu_fc[0] &&
        (fetch_b >= 24'h100000) && (fetch_b < 24'h200000);

    reg derail_tripped = 0;
    always @(posedge clk) begin
        if (derail_fetch && !derail_tripped) begin
            derail_tripped <= 1'b1;
            $display("*** DERAIL at cyc=%0d: program fetch $%06x (last_pc1=$%06x last_pc2=$%06x) intack=%0d flash_wr=%0d ***",
                     cyc, fetch_b, last_pc1, last_pc2, n_intack, n_flash_wr);
            dump_ring_file("derail_ring.log");
            dump_ram("derail_ram.hex");
            $fwrite(lf, "%0t cyc=%0d DERAIL fetch $%06x\n", $time, cyc, fetch_b);
            $finish;
        end
    end

    always @(posedge clk) begin
        // Stall: no CPU bus activity for a very long time (100M cycles =
        // 1.6 s — well above any legitimate timer period, including the
        // once-per-second RTC/AI3 wake).
        if (boot_done && !reset && (n_xfers > 0) &&
            (cyc - last_xfer_cyc > 32'd100000000))
            finish_dump("STALL (100M idle cycles)");
        // (The AI7 abort was removed for P3: the hardware fix now lets the
        // AI7 write land, and we must run PAST the soft-reboot to catch the
        // derail. The derail trigger above stops the sim at the fault.)
        else if (cyc > 32'd4000000000)
            finish_dump("END (2.5G cycle budget)");
    end

endmodule
