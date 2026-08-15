//
// cpu_wrapper.sv — fx68k clock enable generation, bus bridge, and IRQ encoder
// TI-89 MiSTer Core
//

module cpu_wrapper (
    input         clk,          // Master clock (64 MHz)
    input         reset,        // Active-high synchronous reset
    input         cpu_en,       // CPU enable (deasserted during ROM load)
    input         halt,         // Freeze the CPU in place (no reset); used
                                // for the $600005 low-power STOP register

    // CPU bus interface (directly exposed to memory controller)
    output [23:1] cpu_addr,     // 68000 address bus (word-aligned)
    output [15:0] cpu_dout,     // Data from CPU
    input  [15:0] cpu_din,      // Data to CPU
    output        cpu_as_n,     // Address Strobe (active low)
    output        cpu_uds_n,    // Upper Data Strobe (active low)
    output        cpu_lds_n,    // Lower Data Strobe (active low)
    output        cpu_rw_n,     // Read/Write (1=read, 0=write)
    input         cpu_dtack_n,  // Data Transfer Acknowledge (active low)
    output  [2:0] cpu_fc,       // Function Code

    output        cpu_E,        // E clock output (active)
    output        cpu_vma_n,    // Valid Memory Address

    // Interrupt interface
    input   [2:0] ipl,          // Interrupt Priority Level (active-high encoded, 0=none, 7=NMI)
    input         vpa_n,        // Valid Peripheral Address. The top level
                                // drives this low during IACK cycles
                                // (fc=111, as_n=0) to grant autovector
                                // interrupts

    // Reset output from CPU
    output        cpu_reset_out_n,
    output        cpu_halted_n
);

    // =========================================================================
    // Clock enable generation for fx68k
    // =========================================================================
    //
    // fx68k requires enPhi1 and enPhi2 as single-cycle pulses at the master
    // clock rate, alternating to create the effective 68000 clock.
    //
    // For TI-89 Titanium (HW3): effective CPU clock ~12 MHz
    // With 64 MHz master: divide by ~5.33, we use divide-by-6 (10.67 MHz)
    // which is close enough. Each half-period is 3 master clocks.
    //
    // Phase timing: enPhi1 fires 1 cycle before the rising edge,
    //               enPhi2 fires 1 cycle before the falling edge.
    //

    localparam CLK_DIV = 3'd5;  // Divide master by (CLK_DIV+1) = 6 → ~10.67 MHz

    reg [2:0] clk_counter;
    reg       en_phi1;
    reg       en_phi2;

    always @(posedge clk) begin
        if (reset || !cpu_en || halt) begin
            clk_counter <= 3'd0;
            en_phi1     <= 1'b0;
            en_phi2     <= 1'b0;
        end else begin
            en_phi1 <= 1'b0;
            en_phi2 <= 1'b0;

            if (clk_counter == CLK_DIV) begin
                clk_counter <= 3'd0;
            end else begin
                clk_counter <= clk_counter + 3'd1;
            end

            // enPhi1: assert one cycle before rising edge (at count 0)
            if (clk_counter == CLK_DIV)
                en_phi1 <= 1'b1;

            // enPhi2: assert one cycle before falling edge (at half-period)
            if (clk_counter == (CLK_DIV >> 1))
                en_phi2 <= 1'b1;
        end
    end

    // =========================================================================
    // IPL encoding (active-low for fx68k)
    // =========================================================================
    // ipl input: 3-bit priority, 0=no interrupt, 7=highest (NMI)
    // fx68k expects IPL0n, IPL1n, IPL2n active-low (inverted)

    wire ipl0n = ~ipl[0];
    wire ipl1n = ~ipl[1];
    wire ipl2n = ~ipl[2];

    // =========================================================================
    // fx68k instantiation
    // =========================================================================

    wire        fx_erw_n;
    wire        fx_as_n;
    wire        fx_lds_n;
    wire        fx_uds_n;
    wire        fx_E;
    wire        fx_vma_n;
    wire  [2:0] fx_fc;
    wire        fx_bg_n;
    wire        fx_reset_n;
    wire        fx_halted_n;
    wire [15:0] fx_dout;
    wire [23:1] fx_addr;

    fx68k cpu (
        .clk        (clk),
        .extReset   (reset || !cpu_en),
        .pwrUp      (reset),
        .enPhi1     (en_phi1),
        .enPhi2     (en_phi2),

        .eRWn       (fx_erw_n),
        .ASn        (fx_as_n),
        .LDSn       (fx_lds_n),
        .UDSn       (fx_uds_n),
        .E          (fx_E),
        .VMAn       (fx_vma_n),
        .FC0        (fx_fc[0]),
        .FC1        (fx_fc[1]),
        .FC2        (fx_fc[2]),
        .BGn        (fx_bg_n),
        .oRESETn    (fx_reset_n),
        .oHALTEDn   (fx_halted_n),

        .DTACKn     (cpu_dtack_n),
        .VPAn       (vpa_n),        // Low during IACK -> autovector
        .BERRn      (1'b1),         // No bus errors
        .BRn        (1'b1),         // No bus request
        .BGACKn     (1'b1),         // No bus grant acknowledge

        .IPL0n      (ipl0n),
        .IPL1n      (ipl1n),
        .IPL2n      (ipl2n),

        .iEdb       (cpu_din),
        .oEdb       (fx_dout),
        .eab        (fx_addr)
    );

    // =========================================================================
    // Output assignments
    // =========================================================================

    assign cpu_addr       = fx_addr;
    assign cpu_dout       = fx_dout;
    assign cpu_as_n       = fx_as_n;
    assign cpu_uds_n      = fx_uds_n;
    assign cpu_lds_n      = fx_lds_n;
    assign cpu_rw_n       = fx_erw_n;
    assign cpu_fc         = fx_fc;
    assign cpu_E          = fx_E;
    assign cpu_vma_n      = fx_vma_n;
    assign cpu_reset_out_n = fx_reset_n;
    assign cpu_halted_n   = fx_halted_n;

endmodule
