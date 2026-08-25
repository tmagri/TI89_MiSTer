# TI-89 MiSTer Core — Timing Constraints

# Primary clocks are defined in sys_top.sdc (included by the sys framework)

# -------------------------------------------------------------------------
# fx68k multicycle paths (see rtl/fx68k/fx68k.txt & Minimig reference)
# -------------------------------------------------------------------------
# Microcode/nanocode access and ALU condition code paths are multi-cycle.
# Using broad wildcards so both instance paths (*|cpu|*) and entity-qualified
# paths (*fx68k:*) match regardless of Quartus keeper naming format.

set_multicycle_path -start -setup -from [get_keepers {*|Ir[*]}]             -to [get_keepers {*|microAddr[*]}] 2
set_multicycle_path -start -hold  -from [get_keepers {*|Ir[*]}]             -to [get_keepers {*|microAddr[*]}] 1
set_multicycle_path -start -setup -from [get_keepers {*|Ir[*]}]             -to [get_keepers {*|nanoAddr[*]}] 2
set_multicycle_path -start -hold  -from [get_keepers {*|Ir[*]}]             -to [get_keepers {*|nanoAddr[*]}] 1

set_multicycle_path -start -setup -from [get_keepers {*|nanoLatch[*]}]        -to [get_keepers {*|excUnit|alu|pswCcr[*]}] 2
set_multicycle_path -start -hold  -from [get_keepers {*|nanoLatch[*]}]        -to [get_keepers {*|excUnit|alu|pswCcr[*]}] 1
set_multicycle_path -start -setup -from [get_keepers {*|excUnit|alu|oper[*]}] -to [get_keepers {*|excUnit|alu|pswCcr[*]}] 2
set_multicycle_path -start -hold  -from [get_keepers {*|excUnit|alu|oper[*]}] -to [get_keepers {*|excUnit|alu|pswCcr[*]}] 1

# -------------------------------------------------------------------------
# emu PLL cross-clock paths (counter[0] = clk_sys, counter[1] = clk_sdram)
# -------------------------------------------------------------------------
# clk_sdram has a -3000 ps phase shift for external SDRAM setup/hold.
set_multicycle_path -setup 2 -from [get_clocks {*|counter[1].output_counter|divclk}] -to [get_clocks {*|counter[0].output_counter|divclk}]
set_multicycle_path -hold  1 -from [get_clocks {*|counter[1].output_counter|divclk}] -to [get_clocks {*|counter[0].output_counter|divclk}]
set_multicycle_path -setup 2 -from [get_clocks {*|counter[0].output_counter|divclk}] -to [get_clocks {*|counter[1].output_counter|divclk}]
set_multicycle_path -hold  1 -from [get_clocks {*|counter[0].output_counter|divclk}] -to [get_clocks {*|counter[1].output_counter|divclk}]

# -------------------------------------------------------------------------
# Video scaler and framework filter paths
# -------------------------------------------------------------------------
set_multicycle_path -to {*Hq2x*} -setup 2
set_multicycle_path -to {*Hq2x*} -hold 1
set_multicycle_path -from [get_clocks {*|pll|pll_inst|altera_pll_i|*[0].*|divclk}] -to {ascal|*} -setup 2
set_multicycle_path -from [get_clocks {*|pll|pll_inst|altera_pll_i|*[0].*|divclk}] -to {ascal|*} -hold 1
