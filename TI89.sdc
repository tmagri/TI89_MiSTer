# TI-89 MiSTer Core — Timing Constraints

# Primary clocks are defined in sys_top.sdc (included by the sys framework)

# fx68k multicycle paths — microcode access is slow but not needed immediately
# See fx68k.txt for rationale
set_multicycle_path -start -setup -from [get_keepers {*fx68k*Ir[*]}] -to [get_keepers {*fx68k*microAddr[*]}] 2
set_multicycle_path -start -hold  -from [get_keepers {*fx68k*Ir[*]}] -to [get_keepers {*fx68k*microAddr[*]}] 1
set_multicycle_path -start -setup -from [get_keepers {*fx68k*Ir[*]}] -to [get_keepers {*fx68k*nanoAddr[*]}] 2
set_multicycle_path -start -hold  -from [get_keepers {*fx68k*Ir[*]}] -to [get_keepers {*fx68k*nanoAddr[*]}] 1
