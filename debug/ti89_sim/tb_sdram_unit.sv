`timescale 1ns / 1ps

module tb_sdram_unit;
    reg clk = 0;
    always #8 clk = ~clk; // 62.5 MHz

    // PLL outclk_1 equivalent (-3000 ps): chip edges 3 ns after clk
    reg clk_sdram = 0;
    initial begin #13; forever #8 clk_sdram = ~clk_sdram; end  // leading ~3ns

    reg reset = 1;
    wire SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH;
    wire [12:0] SDRAM_A;
    wire [1:0]  SDRAM_BA;
    wire SDRAM_nCS, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nWE;
    wire [15:0] sdram_dq_out;
    wire        sdram_dq_oe;
    wire [15:0] chip_dq_out;
    wire        chip_dq_oe;
    wire [15:0] SDRAM_DQ = sdram_dq_oe ? sdram_dq_out :
                           (chip_dq_oe ? chip_dq_out : 16'h0000);

    reg  [24:0] a_addr = 0;
    reg  [15:0] a_wdata = 0;
    reg         a_rd = 0;
    reg         a_wr = 0;
    reg         a_uds_n = 0;
    reg         a_lds_n = 0;
    wire [15:0] a_rdata;
    wire        a_ready;
    wire        init_done;

    sdram u_sdram (
        .clk(clk),
        .clk_sdram(clk_sdram),
        .reset(reset),
        .SDRAM_CLK(SDRAM_CLK),
        .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_A(SDRAM_A),
        .SDRAM_BA(SDRAM_BA),
        .SDRAM_DQ_IN(SDRAM_DQ),
        .SDRAM_DQ_OUT(sdram_dq_out),
        .SDRAM_DQ_OE(sdram_dq_oe),
        .SDRAM_DQML(SDRAM_DQML),
        .SDRAM_DQMH(SDRAM_DQMH),
        .SDRAM_nCS(SDRAM_nCS),
        .SDRAM_nCAS(SDRAM_nCAS),
        .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nWE(SDRAM_nWE),
        .a_addr(a_addr),
        .a_wdata(a_wdata),
        .a_rd(a_rd),
        .a_wr(a_wr),
        .a_uds_n(a_uds_n),
        .a_lds_n(a_lds_n),
        .a_rdata(a_rdata),
        .a_ready(a_ready),
        .b_addr(21'd0),
        .b_wdata(16'd0),
        .b_wr(1'b0),
        .b_wait(),
        .init_done(init_done)
    );

    sdram_chip u_chip (
        .CLK(SDRAM_CLK),
        .CKE(SDRAM_CKE),
        .A(SDRAM_A),
        .BA(SDRAM_BA),
        .DQ_IN(SDRAM_DQ),
        .DQ_OUT(chip_dq_out),
        .DQ_OE(chip_dq_oe),
        .DQML(SDRAM_DQML),
        .DQMH(SDRAM_DQMH),
        .nCS(SDRAM_nCS),
        .nRAS(SDRAM_nRAS),
        .nCAS(SDRAM_nCAS),
        .nWE(SDRAM_nWE)
    );

    reg mon_en = 0;

    always @(posedge SDRAM_CLK) if (mon_en && !SDRAM_nCS)
        $display("%0t CHIP-CMD nRAS=%b nCAS=%b nWE=%b A=%h BA=%b DQM=%b%b dq_oe=%b dq_out=%h",
                 $time, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE, SDRAM_A, SDRAM_BA,
                 SDRAM_DQMH, SDRAM_DQML, chip_dq_oe, chip_dq_out);

    always @(negedge clk) if (mon_en)
        $display("%0t CAPTURE dq=%h chip_oe=%b", $time, SDRAM_DQ, chip_dq_oe);

    always @(posedge clk) if (mon_en && u_sdram.a_ready)
        $display("%0t READY rdata=%h captured=%h", $time, a_rdata, u_sdram.sdram_dq_captured);

    initial begin
        $display("Starting SDRAM unit test...");
        #100;
        reset = 0;
        // Wait for init_done
        while (!init_done) @(posedge clk);
        $display("SDRAM init done at %0t ns", $time);
        mon_en = 1;

        // Test 1: Write to RAM address $400000 (word 0x200000)
        @(posedge clk);
        a_addr  <= 25'h0400000;
        a_wdata <= 16'h1234;
        a_wr    <= 1'b1;
        a_uds_n <= 1'b0;
        a_lds_n <= 1'b0;
        @(posedge clk);
        a_wr    <= 1'b0;

        while (!a_ready) @(posedge clk);
        $display("Write 1 completed at %0t ns", $time);

        // Test 2: Read back from RAM address $400000
        @(posedge clk);
        a_addr <= 25'h0400000;
        a_rd   <= 1'b1;
        @(posedge clk);
        a_rd   <= 1'b0;

        while (!a_ready) @(posedge clk);
        $display("Read 1 completed at %0t ns, a_rdata = %04x (expected 1234)", $time, a_rdata);
        if (a_rdata !== 16'h1234) begin
            $display("ERROR: Read data mismatch! Got %04x, expected 1234", a_rdata);
            $fatal(1);
        end

        // Test 3: Write multiple words and read them back
        for (integer i = 0; i < 16; i = i + 1) begin
            @(posedge clk);
            a_addr  <= 25'h0400000 + (i * 2);
            a_wdata <= 16'hA000 + i;
            a_wr    <= 1'b1;
            @(posedge clk);
            a_wr    <= 1'b0;
            while (!a_ready) @(posedge clk);
        end

        for (integer i = 0; i < 16; i = i + 1) begin
            @(posedge clk);
            a_addr <= 25'h0400000 + (i * 2);
            a_rd   <= 1'b1;
            @(posedge clk);
            a_rd   <= 1'b0;
            while (!a_ready) @(posedge clk);
            if (a_rdata !== (16'hA000 + i)) begin
                $display("ERROR: Word %0d mismatch! Got %04x, expected %04x", i, a_rdata, 16'hA000 + i);
                $fatal(1);
            end
        end

        $display("All SDRAM unit tests PASSED!");
        $finish;
    end
endmodule
