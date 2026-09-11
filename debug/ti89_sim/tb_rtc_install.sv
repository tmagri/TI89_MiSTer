`timescale 1ns / 1ps

module tb_rtc_install;
    reg clk = 0;
    always #8 clk = ~clk; // ~60 MHz

    reg cpu_reset = 1;
    reg [32:0] timestamp = 33'd0;

    reg  [7:0]  io_addr = 8'd0;
    reg  [15:0] io_wdata = 16'd0;
    wire [15:0] io_rdata;
    reg         io_rd = 0;
    reg         io_wr = 0;
    reg  [1:0]  io_bank = 2'd0;
    reg         io_uds_n = 1;
    reg         io_lds_n = 1;

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
        .protect(1'b0),
        .kbd_row_mask(),
        .kbd_col_data(8'hFF),
        .on_key(1'b0),
        .timer_ctrl(),
        .timer_init(),
        .timer_value(8'd0),
        .lcd_addr(),
        .lcd_log_w(),
        .lcd_log_h(),
        .lcd_contrast(),
        .lcd_on(),
        .lcd_vsync(1'b0),
        .cpu_stop(),
        .stop_mask(),
        .ack_ai2(),
        .ack_ai6(),
        .timer_load(),
        .prot_arm(),
        .timestamp(timestamp)
    );

    // rom_loader instantiation
    reg         rl_reset = 1;
    reg         ioctl_download = 0;
    reg  [15:0] ioctl_index = 0;
    reg         ioctl_wr = 0;
    reg  [26:0] ioctl_addr = 0;
    reg  [15:0] ioctl_dout = 0;
    reg         sdram_wait = 0;

    wire        rom_loaded;
    wire        load_failed;
    wire        loading;
    wire        sdram_wr;
    wire [20:0] sdram_addr;
    wire [15:0] sdram_dout;

    rom_loader u_rl (
        .clk(clk),
        .reset(rl_reset),
        .ioctl_download(ioctl_download),
        .ioctl_index(ioctl_index),
        .ioctl_wr(ioctl_wr),
        .ioctl_addr(ioctl_addr),
        .ioctl_dout(ioctl_dout),
        .sdram_wait(sdram_wait),
        .rom_loaded(rom_loaded),
        .load_failed(load_failed),
        .loading(loading),
        .sdram_wr(sdram_wr),
        .sdram_addr(sdram_addr),
        .sdram_dout(sdram_dout)
    );

    // Track writes to cert block (0x008000..0x008027) and archive markers
    reg [15:0] captured_cert [0:39];
    reg [39:0] cert_written = 40'd0;
    reg [38:0] arc_written = 39'd0;

    always @(posedge clk) begin
        if (sdram_wr) begin
            if (sdram_addr >= 21'h008000 && sdram_addr <= 21'h008027) begin
                captured_cert[sdram_addr - 21'h008000] <= sdram_dout;
                cert_written[sdram_addr - 21'h008000] <= 1'b1;
            end
            if (sdram_addr >= 21'h0C8000 && sdram_dout == 16'hFFFE) begin
                if ((sdram_addr[14:0] == 15'd0) && ((sdram_addr - 21'h0C8000) >> 15 <= 38)) begin
                    arc_written[(sdram_addr - 21'h0C8000) >> 15] <= 1'b1;
                end
            end
        end
    end

    // Write helper: note that a is word address (byte_addr >> 1)
    task write_io(input [1:0] b, input [7:0] a, input [15:0] d, input uds, input lds);
    begin
        @(posedge clk);
        io_bank  <= b;
        io_addr  <= a;
        io_wdata <= d;
        io_wr    <= 1'b1;
        io_uds_n <= ~uds;
        io_lds_n <= ~lds;
        @(posedge clk);
        io_wr    <= 1'b0;
        io_uds_n <= 1'b1;
        io_lds_n <= 1'b1;
    end
    endtask

    task send_word(input [15:0] w);
    begin
        @(posedge clk);
        ioctl_dout <= {w[7:0], w[15:8]}; // little-endian in ioctl_dout
        ioctl_wr   <= 1'b1;
        ioctl_addr <= ioctl_addr + 27'd2;
        @(posedge clk);
        ioctl_wr   <= 1'b0;
    end
    endtask

    integer i;

    initial begin
        $display("=== STARTING RTC AND INSTALL TESTBENCH ===");
        #100;
        @(posedge clk);
        cpu_reset <= 1'b0;
        rl_reset  <= 1'b0;
        #50;

        // 1. Verify io3[0x5F] default is 0x81 (HW3 RTC present and enabled)
        @(posedge clk);
        io_bank <= 2'd2;
        io_addr <= 8'h2F;
        io_rd   <= 1'b1;
        io_uds_n <= 1'b0;
        io_lds_n <= 1'b0;
        @(posedge clk);
        io_rd <= 1'b0;
        #1;
        if (io_rdata[7:0] !== 8'h81) begin
            $display("FAILED: io3[0x5F] expected 0x81, got 0x%02X", io_rdata[7:0]);
            $finish;
        end else begin
            $display("PASS: io3[0x5F] defaults to 0x%02X (HW3 RTC present & enabled)", io_rdata[7:0]);
        end

        // 2. Feed MiSTer HPS timestamp for 2026-09-11 (Unix time: 1,780,000,000)
        // TI-89 epoch difference: 852,076,800
        // Expected TI time: 927,923,200 (0x374EFC00)
        @(posedge clk);
        timestamp <= {1'b1, 32'd1780000000};
        #100;

        // Read back RTC seconds ($710046..$710049)
        @(posedge clk);
        io_bank <= 2'd2;
        io_addr <= 8'h23;
        io_uds_n <= 1'b0;
        io_lds_n <= 1'b0;
        io_rd   <= 1'b1;
        @(posedge clk);
        io_rd <= 1'b0;
        #1;
        if (io_rdata !== 16'h374E) begin
            $display("FAILED: RTC seconds[31:16] expected 0x374E, got 0x%04X", io_rdata);
            $finish;
        end else begin
            $display("PASS: RTC seconds[31:16] = 0x%04X matches expected HPS timestamp", io_rdata);
        end

        @(posedge clk);
        io_bank <= 2'd2;
        io_addr <= 8'h24;
        io_uds_n <= 1'b0;
        io_lds_n <= 1'b0;
        io_rd   <= 1'b1;
        @(posedge clk);
        io_rd <= 1'b0;
        #1;
        if (io_rdata !== 16'hFC00) begin
            $display("FAILED: RTC seconds[15:0] expected 0xFC00, got 0x%04X", io_rdata);
            $finish;
        end else begin
            $display("PASS: RTC seconds[15:0] = 0x%04X matches expected HPS timestamp", io_rdata);
        end

        // 3. Test Persistence across CPU Reset
        $display("Asserting cpu_reset to test RTC persistence...");
        @(posedge clk);
        cpu_reset <= 1'b1;
        #200;
        @(posedge clk);
        cpu_reset <= 1'b0;
        #100;

        @(posedge clk);
        io_bank <= 2'd2;
        io_addr <= 8'h23;
        io_uds_n <= 1'b0;
        io_lds_n <= 1'b0;
        io_rd   <= 1'b1;
        @(posedge clk);
        io_rd <= 1'b0;
        #1;
        if (io_rdata !== 16'h374E) begin
            $display("FAILED: RTC reset on cpu_reset! Expected 0x374E, got 0x%04X", io_rdata);
            $finish;
        end else begin
            $display("PASS: RTC persisted across cpu_reset (seconds[31:16] = 0x%04X)", io_rdata);
        end

        // 4. Test AMS Cold-Boot Protection
        $display("Simulating AMS cold-boot wipe: writing 0 to RTC reload registers...");
        write_io(2'd2, 8'h20, 16'h0000, 1, 1);
        write_io(2'd2, 8'h21, 16'h0000, 1, 1);
        write_io(2'd2, 8'h22, 16'h0000, 1, 0); // 0x44 = 0
        write_io(2'd2, 8'h2F, 16'h0001, 0, 1); // 0x5F = 0x01 (reload strobe)

        #100;
        @(posedge clk);
        io_bank <= 2'd2;
        io_addr <= 8'h23;
        io_uds_n <= 1'b0;
        io_lds_n <= 1'b0;
        io_rd   <= 1'b1;
        @(posedge clk);
        io_rd <= 1'b0;
        #1;
        if (io_rdata !== 16'h374E) begin
            $display("FAILED: AMS 0-write wiped RTC! Got 0x%04X", io_rdata);
            $finish;
        end else begin
            $display("PASS: AMS 0-write successfully guarded! RTC retained hps_ti_time (0x%04X)", io_rdata);
        end

        // 5. Test manual user write of non-zero time (e.g. 0x12345678)
        $display("Simulating user manual time setting (0x12345678)...");
        write_io(2'd2, 8'h20, 16'h1234, 1, 1);
        write_io(2'd2, 8'h21, 16'h5678, 1, 1);
        write_io(2'd2, 8'h22, 16'h0000, 1, 0);
        write_io(2'd2, 8'h2F, 16'h0001, 0, 1); // strobe reload

        #100;
        @(posedge clk);
        io_bank <= 2'd2;
        io_addr <= 8'h23;
        io_uds_n <= 1'b0;
        io_lds_n <= 1'b0;
        io_rd   <= 1'b1;
        @(posedge clk);
        io_rd <= 1'b0;
        #1;
        if (io_rdata !== 16'h1234) begin
            $display("FAILED: User time write failed! Expected 0x1234, got 0x%04X", io_rdata);
            $finish;
        end else begin
            $display("PASS: User non-zero time write accepted (0x%04X)", io_rdata);
        end

        $display("=== ALL RTC TESTS PASSED! ===");

        // 6. Test rom_loader S_PCERT and S_PARC
        $display("=== TESTING ROM_LOADER CERT & ARCHIVE PRE-FILL ===");
        @(posedge clk);
        ioctl_download <= 1'b1;
        ioctl_index    <= 16'd0;
        ioctl_addr     <= 27'd0;
        @(posedge clk);

        // Send signature: "**TIFL**"
        send_word(16'h2A2A); // "**"
        send_word(16'h5449); // "TI"
        send_word(16'h464C); // "FL"
        send_word(16'h2A2A); // "**"

        // Send "basecode"
        send_word(16'h6261); // "ba"
        send_word(16'h7365); // "se"
        send_word(16'h636F); // "co"
        send_word(16'h6465); // "de"

        // 53 skip bytes
        for (i = 0; i < 27; i = i + 1) begin
            send_word(16'h0000);
        end

        // A few payload words
        for (i = 0; i < 100; i = i + 1) begin
            send_word(16'h1234 + i);
        end

        // Finish download
        @(posedge clk);
        ioctl_download <= 1'b0;

        // Wait for rom_loader to reach S_DONE
        $display("Waiting for rom_loader to complete S_FILL1, S_FILL2, S_PCERT, S_PARC...");
        while (!rom_loaded && !load_failed) begin
            @(posedge clk);
        end

        if (load_failed) begin
            $display("FAILED: rom_loader reported load_failed!");
            $finish;
        end

        $display("rom_loaded asserted!");

        // Verify all 40 cert words were written!
        if (cert_written !== 40'hFFFFFFFFFF) begin
            $display("FAILED: Not all 40 cert words were written! Mask = 0x%010X", cert_written);
            $finish;
        end else begin
            $display("PASS: All 40 cert words written to SDRAM!");
        end

        if (captured_cert[0] !== 16'hFFF8 || captured_cert[1] !== 16'h0000 || captured_cert[39] !== 16'h4FFF) begin
            $display("FAILED: Cert words mismatch! [0]=0x%04X, [1]=0x%04X, [39]=0x%04X",
                     captured_cert[0], captured_cert[1], captured_cert[39]);
            $finish;
        end else begin
            $display("PASS: Cert words matched S3 gospel values ([0]=0x%04X, [1]=0x%04X, [39]=0x%04X)",
                     captured_cert[0], captured_cert[1], captured_cert[39]);
        end

        // Verify all 39 archive markers were written!
        if (arc_written !== 39'h7FFFFFFFFF) begin
            $display("FAILED: Not all 39 archive markers were written! Mask = 0x%010X", arc_written);
            $finish;
        end else begin
            $display("PASS: All 39 archive markers written to SDRAM!");
        end

        $display("=== ALL ROM_LOADER TESTS PASSED! ===");
        $finish;
    end
endmodule
