//
// pll.v — thin wrapper around the wizard-generated pll_0002 megafunction
// TI-89 MiSTer Core
//
// Why this wrapper exists:
//   The sys framework's timing file (sys/sys_top.sdc) defines exclusive
//   clock groups using a hierarchy pattern that expects the PLL to be
//   instantiated as  *|pll|pll_inst|altera_pll_i|... :
//       set_clock_groups -name "emu_pll_clk" \
//           -group [get_clocks {*|pll|pll_inst|altera_pll_i|*}] ...
//   Minimig achieves this by wrapping its PLL in a module named `pll` that
//   instantiates the megafunction as `pll_inst`.  If we instantiate
//   pll_0002 directly (as instance `pll`), our clocks land at
//   emu|pll|altera_pll_i|... which does NOT match the pattern, so Quartus
//   leaves every cross-domain path between clk_sys and the sys/ascal/HDMI
//   domains unconstrained — flooding the timing report with phantom
//   violations (and making real ones impossible to spot).
//
//   This wrapper makes our hierarchy identical to Minimig's:
//       emu|pll|pll_inst|altera_pll_i|...
//
// Clock plan (integer-N PLL, 50 MHz reference):
//   M = m_cnt_hi_div + m_cnt_lo_div = 30 + 30 = 60
//   N = n_cnt_hi_div + n_cnt_lo_div =  3 +  2 =  5
//   VCO = 50 MHz x 60/5 = 600 MHz
//   outclk_0 = VCO / (c_cnt_hi_div0 + c_cnt_lo_div0) = 600/10 = 60.000 MHz
//

`timescale 1 ps / 1 ps
module pll (
		input  wire        refclk,            //            refclk.clk
		input  wire        rst,               //             reset.reset
		output wire        outclk_0,          //           outclk0.clk
		output wire        outclk_1,          //           outclk1.clk
		output wire        locked,            //            locked.export
		input  wire [63:0] reconfig_to_pll,   //   reconfig_to_pll.reconfig_to_pll
		output wire [63:0] reconfig_from_pll  // reconfig_from_pll.reconfig_from_pll
	);

	pll_0002 pll_inst (
		.refclk            (refclk),            //            refclk.clk
		.rst               (rst),               //             reset.reset
		.outclk_0          (outclk_0),          //           outclk0.clk
		.outclk_1          (outclk_1),          //           outclk1.clk
		.locked            (locked),            //            locked.export
		.reconfig_to_pll   (reconfig_to_pll),   //   reconfig_to_pll.reconfig_to_pll
		.reconfig_from_pll (reconfig_from_pll)  // reconfig_from_pll.reconfig_from_pll
	);

endmodule
