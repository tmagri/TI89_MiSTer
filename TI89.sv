//
// TI89.sv — TI-89 Titanium MiSTer core top level (the MiSTer "emu" module)
// TI-89 MiSTer Core
//
// Instantiated as `emu` by sys/sys_top.v; the port list mirrors
// sys/emu_ports.vh.
//
// Clocking:
//   CLK_50M -> PLL -> 60 MHz master clock (clk_sys)
//   The whole core runs from clk_sys. The SDRAM chip clock (SDRAM_CLK)
//   is a clean 50% duty-cycle inversion of clk_sys, generated in the IO
//   row by altddio_out (same pattern as sys_top.v's hdmi/vga clocks).
//   CLK_VIDEO = clk_sys, video pixels are strobed by CE_PIXEL.
//
// Boot flow:
//   The user loads a .89u OS image from the OSD ("Load OS Image").
//   rom_loader parses the stream into SDRAM; mem_ctrl then clears RAM,
//   copies the 128-word OS header from $812088 to RAM $000000 and
//   releases the CPU (fx68k), which fetches its reset vectors from RAM
//   and starts executing the OS from the flash window.
//
// Legal note: The OS image file (TI89Titanium_OS.89u) is copyrighted by
// Texas Instruments. The core does **not** embed the image; it must be
// loaded by the user at runtime via the MiSTer OSD file browser (same
// pattern as Amiga Kickstart ROMs in Minimig).
//

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output  [1:0] VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM2_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

	///////////////////////////////////////////////////////////////////////////
	// Clocks and reset
	///////////////////////////////////////////////////////////////////////////

	wire clk_sys;      // 60 MHz master clock
	wire clk_sdram;    // 60 MHz SDRAM clock (clean ~clk_sys via altddio_out)
	wire pll_locked;

	// Wrapper (rtl/pll.v) so the PLL hierarchy matches the exclusive clock
	// group pattern in sys/sys_top.sdc (*|pll|pll_inst|altera_pll_i|...).
	pll pll
	(
		.refclk(CLK_50M),
		.rst(RESET),
		.outclk_0(clk_sys),        // 60 MHz
		.outclk_1(),               // unused (SDRAM clock now via altddio_out)
		.locked(pll_locked),
		.reconfig_to_pll(64'd0),
		.reconfig_from_pll()
	);

	// SDRAM clock: clean 50% duty-cycle inversion of clk_sys, generated in
	// the IO row next to the SDRAM_CLK pin (same pattern as sys_top.v's
	// hdmi/vga clocks). This replaces the fragile PLL phase-shifted output,
	// which caused intermittent SDRAM read bit-errors on hardware.
	altddio_out
	#(
		.extend_oe_disable("OFF"),
		.intended_device_family("Cyclone V"),
		.invert_output("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_out"),
		.oe_reg("UNREGISTERED"),
		.power_up_high("OFF"),
		.width(1)
	)
	sdramclk_ddr
	(
		.datain_h(1'b0),
		.datain_l(1'b1),
		.outclock(clk_sys),
		.dataout(clk_sdram),
		.aclr(1'b0),
		.aset(1'b0),
		.oe(1'b1),
		.outclocken(1'b1),
		.sclr(1'b0),
		.sset(1'b0)
	);

	wire reset = RESET | ~pll_locked;

	assign CLK_VIDEO = clk_sys;

	///////////////////////////////////////////////////////////////////////////
	// OSD / HPS bridge
	///////////////////////////////////////////////////////////////////////////

	localparam CONF_STR = {
		"TI89;;",
		"F0,89u,Load OS Image;",
		"-;",
		"O[3:2],LCD Color,Green,Blue,Amber,B&W;",
		"O[5:4],LCD Scale,4x,3x,2x,1x;",
		"O6,Debug Overlay,On,Off;",
		"O7,UART Status Line,On,Off;",
		"O8,SDRAM Dump,Off,On;",
		"-;",
		"R0,Reset;",
		"V,v1.0;"
	};

	wire [127:0] status;
	wire         ioctl_download;
	wire  [15:0] ioctl_index;
	wire         ioctl_wr;
	wire  [26:0] ioctl_addr;
	wire  [15:0] ioctl_dout;
	wire  [10:0] ps2_key;
	wire   [1:0] hps_buttons;
	wire         sdram_b_wait;   // SDRAM loader-FIFO backpressure

	// hps_io SD-card / image ports are unused: the OS image is a single
	// file streamed through the ioctl interface.
	wire [31:0] sd_lba[1];
	wire  [5:0] sd_blk_cnt[1];
	wire [15:0] sd_buff_din[1];
	assign sd_lba[0]      = 32'd0;
	assign sd_blk_cnt[0]  = 6'd0;
	assign sd_buff_din[0] = 16'd0;

	hps_io #(.CONF_STR(CONF_STR), .WIDE(1), .VDNUM(1)) hps_io
	(
		.clk_sys(clk_sys),
		.HPS_BUS(HPS_BUS),

		.joystick_0(),
		.joystick_1(),
		.joystick_2(),
		.joystick_3(),
		.joystick_4(),
		.joystick_5(),
		.joystick_l_analog_0(),
		.joystick_l_analog_1(),
		.joystick_l_analog_2(),
		.joystick_l_analog_3(),
		.joystick_l_analog_4(),
		.joystick_l_analog_5(),
		.joystick_r_analog_0(),
		.joystick_r_analog_1(),
		.joystick_r_analog_2(),
		.joystick_r_analog_3(),
		.joystick_r_analog_4(),
		.joystick_r_analog_5(),
		.joystick_0_rumble(16'd0),
		.joystick_1_rumble(16'd0),
		.joystick_2_rumble(16'd0),
		.joystick_3_rumble(16'd0),
		.joystick_4_rumble(16'd0),
		.joystick_5_rumble(16'd0),
		.paddle_0(),
		.paddle_1(),
		.paddle_2(),
		.paddle_3(),
		.paddle_4(),
		.paddle_5(),
		.spinner_0(),
		.spinner_1(),
		.spinner_2(),
		.spinner_3(),
		.spinner_4(),
		.spinner_5(),

		.ps2_kbd_clk_out(),
		.ps2_kbd_data_out(),
		.ps2_kbd_clk_in(1'b0),
		.ps2_kbd_data_in(1'b0),
		.ps2_kbd_led_status(3'd0),
		.ps2_kbd_led_use(3'd0),
		.ps2_mouse_clk_out(),
		.ps2_mouse_data_out(),
		.ps2_mouse_clk_in(1'b0),
		.ps2_mouse_data_in(1'b0),

		.ps2_key(ps2_key),
		.ps2_mouse(),
		.ps2_mouse_ext(),

		.buttons(hps_buttons),
		.forced_scandoubler(),
		.direct_video(),
		.video_rotated(1'b0),
		.new_vmode(1'b0),
		.gamma_bus(),

		.status(status),
		.status_in(128'd0),
		.status_set(1'b0),
		.status_menumask(16'd0),

		.info_req(1'b0),
		.info(8'd0),

		.img_mounted(),
		.img_readonly(),
		.img_size(),

		.sd_lba(sd_lba),
		.sd_blk_cnt(sd_blk_cnt),
		.sd_rd(1'b0),
		.sd_wr(1'b0),
		.sd_ack(),

		.sd_buff_addr(),
		.sd_buff_dout(),
		.sd_buff_din(sd_buff_din),
		.sd_buff_wr(),

		.ioctl_download(ioctl_download),
		.ioctl_index(ioctl_index),
		.ioctl_wr(ioctl_wr),
		.ioctl_addr(ioctl_addr),
		.ioctl_dout(ioctl_dout),
		.ioctl_upload(),
		.ioctl_upload_req(1'b0),
		.ioctl_upload_index(8'd0),
		.ioctl_din(16'd0),
		.ioctl_rd(),
		.ioctl_file_ext(),
		.ioctl_wait(sdram_b_wait),

		.sdram_sz(),
		.RTC(),
		.TIMESTAMP(),
		.uart_mode(),
		.uart_speed(),

		.EXT_BUS()
	);

	///////////////////////////////////////////////////////////////////////////
	// OS image loader (.89u -> SDRAM)
	///////////////////////////////////////////////////////////////////////////

	wire        rom_loaded;   // OS image valid in SDRAM
	wire        loading;      // download or fill in progress
	wire        load_failed;  // download ended without a valid signature
	wire        ld_wr;
	wire [20:0] ld_addr;      // Word address
	wire [15:0] ld_dout;

	rom_loader rom_loader
	(
		.clk(clk_sys),
		.reset(reset),
		.ioctl_download(ioctl_download),
		.ioctl_index(ioctl_index),
		.ioctl_wr(ioctl_wr),
		.ioctl_addr(ioctl_addr),
		.ioctl_dout(ioctl_dout),
		.sdram_wr(ld_wr),
		.sdram_addr(ld_addr),
		.sdram_dout(ld_dout),
		.sdram_wait(sdram_b_wait),
		.rom_loaded(rom_loaded),
		.load_failed(load_failed),
		.loading(loading)
	);

	///////////////////////////////////////////////////////////////////////////
	// SDRAM (backs the 4MB flash window and the 256KB calculator RAM)
	///////////////////////////////////////////////////////////////////////////

	// Port A: traffic from mem_ctrl's arbiter (flash_ctrl + RAM clients)
	wire [24:0] sd_addr;
	wire [15:0] sd_wdata;
	wire        sd_rd, sd_wr, sd_uds_n, sd_lds_n;
	wire [15:0] sd_rdata;
	wire        sd_ready;
	wire        sdram_init_done;

	// Bidirectional DQ rebuilt from the controller's split in/out/OE ports.
	wire [15:0] sdram_dq_out;
	wire        sdram_dq_oe;
	wire [15:0] sdram_dq_in = SDRAM_DQ;
	assign SDRAM_DQ = sdram_dq_oe ? sdram_dq_out : 16'hZZZZ;

	sdram sdram
	(
		.clk(clk_sys),
		.clk_sdram(clk_sdram),
		.reset(reset),

		.SDRAM_CLK(SDRAM_CLK),
		.SDRAM_CKE(SDRAM_CKE),
		.SDRAM_A(SDRAM_A),
		.SDRAM_BA(SDRAM_BA),
		.SDRAM_DQ_IN(sdram_dq_in),
		.SDRAM_DQ_OUT(sdram_dq_out),
		.SDRAM_DQ_OE(sdram_dq_oe),
		.SDRAM_DQML(SDRAM_DQML),
		.SDRAM_DQMH(SDRAM_DQMH),
		.SDRAM_nCS(SDRAM_nCS),
		.SDRAM_nCAS(SDRAM_nCAS),
		.SDRAM_nRAS(SDRAM_nRAS),
		.SDRAM_nWE(SDRAM_nWE),

		.a_addr(sd_addr),
		.a_wdata(sd_wdata),
		.a_rd(sd_rd),
		.a_wr(sd_wr),
		.a_uds_n(sd_uds_n),
		.a_lds_n(sd_lds_n),
		.a_rdata(sd_rdata),
		.a_ready(sd_ready),

		.b_addr(ld_addr),
		.b_wdata(ld_dout),
		.b_wr(ld_wr),
		.b_wait(sdram_b_wait),

		.init_done(sdram_init_done)
	);

	///////////////////////////////////////////////////////////////////////////
	// Flash controller (Sharp WSM over the SDRAM-backed flash window)
	///////////////////////////////////////////////////////////////////////////

	// Command side comes from mem_ctrl's bus FSM; the controller's own
	// SDRAM-side accesses go back through mem_ctrl's port A arbiter.
	wire [21:0] flash_addr;
	wire [15:0] flash_wdata;
	wire        flash_rd, flash_wr, flash_uds_n, flash_lds_n;
	wire [15:0] flash_rdata;
	wire        flash_ready;

	wire [21:0] fl_addr;
	wire [15:0] fl_wdata;
	wire        fl_rd, fl_wr, fl_uds_n, fl_lds_n;
	wire [15:0] fl_rdata;
	wire        fl_ready;

	flash_ctrl flash_ctrl
	(
		.clk(clk_sys),
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

	///////////////////////////////////////////////////////////////////////////
	// Memory controller (RAM, address decode, bus FSM, boot FSM)
	///////////////////////////////////////////////////////////////////////////

	wire [23:1] cpu_addr;
	wire [15:0] cpu_dout;
	wire [15:0] cpu_din;
	wire        cpu_as_n, cpu_uds_n, cpu_lds_n, cpu_rw_n;
	wire        cpu_dtack_n;
	wire  [2:0] cpu_fc;
	wire        boot_done;

	wire [17:0] lcd_ram_addr;
	wire [15:0] lcd_ram_data;
	wire        lcd_ram_req;
	wire        lcd_ram_ack;

	wire  [7:0] io_addr;
	wire [15:0] io_wdata;
	wire [15:0] io_rdata;
	wire        io_rd, io_wr;
	wire  [1:0] io_bank;
	wire        io_uds_n, io_lds_n;
	wire        protect;  // flash protection state (mem_ctrl hwprot)
	wire        prot_arm; // $600001 bit 2: AI7 low-RAM write protection armed
	wire        ai7_hit;  // CPU wrote below $000120 while prot_arm was set

	// Pre-boot SDRAM image dump (mem_ctrl -> dbg_uart)
	wire        dump_rdy;
	wire        dump_stb;
	wire [15:0] dump_word;
	wire        dump_pass_stb;
	wire        dump_active;

	// Host command dumps (dbg_uart RX -> mem_ctrl)
	wire        dbg_cmd_req;
	wire        dbg_cmd_mem;
	wire        dbg_cmd_wr;
	wire [23:0] dbg_cmd_start;
	wire [23:0] dbg_cmd_len;
	wire        dbg_trace_req;   // reserved: rings are emitted by dbg_uart itself
	wire        dump_cmd_mode;
	wire        dbg_rx = UART_RXD;

	mem_ctrl mem_ctrl
	(
		.clk(clk_sys),
		.reset(reset),

		.rom_loaded(rom_loaded),
		.init_done(sdram_init_done),
		.boot_done(boot_done),
		.dump_en(status[8]),

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

		.dump_rdy(dump_rdy),
		.dump_stb(dump_stb),
		.dump_word(dump_word),
		.dump_pass_stb(dump_pass_stb),
		.dump_active(dump_active),

		.cmd_req(dbg_cmd_req),
		.cmd_mem(dbg_cmd_mem),
		.cmd_wr(dbg_cmd_wr),
		.cmd_start(dbg_cmd_start),
		.cmd_len(dbg_cmd_len),
		.dump_cmd_mode(dump_cmd_mode)
	);

	///////////////////////////////////////////////////////////////////////////
	// Reset sources
	///////////////////////////////////////////////////////////////////////////
	// core_reset: power-up / PLL unlock, OSD "Reset" menu entry, I/O board
	// user button.
	// cpu_reset additionally holds the 68000 (and the interrupt/keyboard/
	// I/O register state) in reset until the boot FSM has cleared RAM and
	// copied the OS header to $000000.

	wire core_reset = reset | status[0] | hps_buttons[1];
	wire cpu_reset  = core_reset | ~boot_done;

	///////////////////////////////////////////////////////////////////////////
	// I/O ports ($600000 / $700000 / $710000)
	///////////////////////////////////////////////////////////////////////////

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

	io_ports io_ports
	(
		.clk(clk_sys),
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

	///////////////////////////////////////////////////////////////////////////
	// Timer / interrupt controller
	///////////////////////////////////////////////////////////////////////////

	wire [2:0] ipl;
	wire [7:0] int_pend;   // bit N = auto-interrupt N pending
	wire       kbd_int;
	wire       on_key_press;
	wire       intack_edge; // One-cycle IACK strobe (from the CPU section)

	timer_int timer_int
	(
		.clk(clk_sys),
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
		.ack_level(cpu_addr[3:1]),

		.ipl(ipl),
		.int_pend(int_pend)
	);

	///////////////////////////////////////////////////////////////////////////
	// Keyboard
	///////////////////////////////////////////////////////////////////////////

	keyboard keyboard
	(
		.clk(clk_sys),
		.reset(cpu_reset),
		.ps2_key(ps2_key),
		.row_mask(kbd_row_mask),
		.col_data(kbd_col_data),
		.on_key(on_key),
		.on_key_press(on_key_press),
		.kbd_int(kbd_int)
	);

	///////////////////////////////////////////////////////////////////////////
	// CPU (68000 core) and interrupt acknowledge
	///////////////////////////////////////////////////////////////////////////

	// IACK cycle: on TI-89 all hardware interrupts (levels 1..7) are autovectored.
	// After boot completes (boot_done == 1), every FC=111 cycle with AS asserted
	// is an Interrupt Acknowledge cycle, and VPA must stay asserted for the ENTIRE
	// acknowledge bus cycle — not just while ipl != 0.
	//
	// Without the latch: timer_int clears the pending flag as soon as intack_edge
	// fires (one cycle into the IACK). If that was the ONLY pending interrupt, ipl
	// drops to 0 mid-cycle, intack deasserts, VPA rises, and fx68k abandons the
	// autovector — it terminates off-bus and samples the cpu_din value ($1414 =
	// vector 20, unmapped) instead of the correct autovector. The CPU then jumps
	// to garbage code at $14141x. (Run-10 dense trace, §15 of DEBUG_STATUS.md.)
	//
	// Fix: latch intack for the whole bus cycle. Set when FC=111 & AS & ipl!=0;
	// clear when AS deasserts. vpa_n is driven from the latched version.
	// The reset-vector guard is preserved: those cycles start with ipl==0 so
	// they never latch (intack_raw stays 0 when ipl==0).
	wire intack_raw = (cpu_fc == 3'b111) && !cpu_as_n && boot_done;
	reg  intack_latch;
	always @(posedge clk_sys) begin
		if (cpu_as_n)
			intack_latch <= 1'b0;
		else if (intack_raw && (ipl != 3'd0))
			intack_latch <= 1'b1;
	end
	wire intack = intack_raw && (ipl != 3'd0 || intack_latch);
	wire vpa_n  = ~intack;

	// One-cycle IACK strobe for timer_int's pending-flag clearing
	reg intack_q;
	always @(posedge clk_sys) intack_q <= intack;
	assign intack_edge = intack && !intack_q;

	// Low-power STOP ($600005): freeze the CPU in place. Wake-up sources
	// (v12.js raise_interrupt): AI6 (ON key) always wakes; AI1..AI5 wake
	// only if their bit is set in the mask written to $600005.
	reg  stopped;
	// AI6 (ON key) always wakes; AI7 (NMI) always breaks through STOP;
	// AI1..AI5 wake only if their bit is set in the $600005 mask.
	wire wake = int_pend[7] | int_pend[6] | (|(int_pend[5:1] & stop_mask));

	always @(posedge clk_sys) begin
		if (core_reset || !boot_done)
			stopped <= 1'b0;
		else if (cpu_stop)
			stopped <= 1'b1;
		else if (wake)
			stopped <= 1'b0;
	end

	// Freeze only while the bus is idle so an in-flight bus cycle always
	// completes; on wake-up the CPU resumes exactly where it stopped.
	wire cpu_halt = stopped && cpu_as_n;

	cpu_wrapper cpu
	(
		.clk(clk_sys),
		.reset(cpu_reset),
		.cpu_en(!loading),
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

	///////////////////////////////////////////////////////////////////////////
	// LCD controller and video output
	///////////////////////////////////////////////////////////////////////////

	wire pixel_out;
	wire pixel_valid;

	lcd_ctrl lcd_ctrl
	(
		.clk(clk_sys),
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

	// Boot-chain diagnostic for the video_scaler's status fill:
	//   0 blue   = no OS image loaded yet
	//   1 orange = OS image download / fill in progress
	//   2 red    = download finished but the image was not recognized
	//   3 violet = image accepted, boot copy / CPU start in progress
	//   4        = boot_done, show the normal palette raster
	wire [2:0] boot_status = loading     ? 3'd1 :
	                         load_failed ? 3'd2 :
	                         !rom_loaded ? 3'd0 :
	                         !boot_done  ? 3'd3 :
	                                     3'd4;

	///////////////////////////////////////////////////////////////////////////
	// Live debug tracking registers for on-screen HUD
	///////////////////////////////////////////////////////////////////////////

	reg [23:0] dbg_last_pc;
	reg [23:0] dbg_last_addr;
	reg [15:0] dbg_last_data;
	reg        dbg_last_rw;
	reg [15:0] dbg_intack_cnt;
	reg [15:0] dbg_fl_wr_cnt;
	reg        dbg_ai7_latch;

	always @(posedge clk_sys) begin
		if (reset) begin
			dbg_last_pc    <= 24'd0;
			dbg_last_addr  <= 24'd0;
			dbg_last_data  <= 16'd0;
			dbg_last_rw    <= 1'b1;
			dbg_intack_cnt <= 16'd0;
			dbg_fl_wr_cnt  <= 16'd0;
			dbg_ai7_latch  <= 1'b0;
		end else begin
			if (!cpu_as_n) begin
				dbg_last_addr <= {cpu_addr, 1'b0};
				dbg_last_data <= cpu_rw_n ? cpu_din : cpu_dout;
				dbg_last_rw   <= cpu_rw_n;
				if (cpu_fc == 3'b010 || cpu_fc == 3'b110)
					dbg_last_pc <= {cpu_addr, 1'b0};
			end
			if (intack_edge)
				dbg_intack_cnt <= dbg_intack_cnt + 16'd1;
			if (flash_wr)
				dbg_fl_wr_cnt <= dbg_fl_wr_cnt + 16'd1;
			if (ai7_hit)
				dbg_ai7_latch <= 1'b1;
		end
	end

	// On-screen HUD sample latch (updates 4 times per second for solid, readable digits)
	reg [23:0] hud_timer;
	reg [23:0] disp_pc;
	reg [23:0] disp_addr;
	reg [15:0] disp_data;
	reg        disp_rw;
	reg [15:0] disp_int_cnt;
	reg [15:0] disp_flw_cnt;
	reg  [2:0] disp_ipl;
	reg        disp_lcd_on;
	reg        disp_protect;
	reg        disp_stopped;
	reg        disp_ai7;

	always @(posedge clk_sys) begin
		if (reset) begin
			hud_timer    <= 24'd0;
			disp_pc      <= 24'd0;
			disp_addr    <= 24'd0;
			disp_data    <= 16'd0;
			disp_rw      <= 1'b1;
			disp_int_cnt <= 16'd0;
			disp_flw_cnt <= 16'd0;
			disp_ipl     <= 3'd0;
			disp_lcd_on  <= 1'b0;
			disp_protect <= 1'b0;
			disp_stopped <= 1'b0;
			disp_ai7     <= 1'b0;
		end else begin
			if (hud_timer >= 24'd15_000_000) begin
				hud_timer    <= 24'd0;
				disp_pc      <= dbg_last_pc;
				disp_addr    <= dbg_last_addr;
				disp_data    <= dbg_last_data;
				disp_rw      <= dbg_last_rw;
				disp_int_cnt <= dbg_intack_cnt;
				disp_flw_cnt <= dbg_fl_wr_cnt;
				disp_ipl     <= ipl;
				disp_lcd_on  <= lcd_on;
				disp_protect <= protect;
				disp_stopped <= stopped;
				disp_ai7     <= dbg_ai7_latch;
			end else begin
				hud_timer <= hud_timer + 24'd1;
			end
		end
	end

	///////////////////////////////////////////////////////////////////////////
	// UART diagnostic transmitter (/dev/ttyS1 @ 115200 baud)
	///////////////////////////////////////////////////////////////////////////

	wire dbg_uart_txd;

	dbg_uart dbg_uart
	(
		.clk(clk_sys),
		.reset(reset),

		.rxd(dbg_rx),
		.boot_done(boot_done),
		.status_mute(status[7]),
		.cmd_req(dbg_cmd_req),
		.cmd_wr(dbg_cmd_wr),
		.cmd_mem(dbg_cmd_mem),
		.cmd_start(dbg_cmd_start),
		.cmd_len(dbg_cmd_len),
		.trace_req(dbg_trace_req),
		.dump_cmd_mode(dump_cmd_mode),

		.dbg_pc(dbg_last_pc),
		.dbg_addr(dbg_last_addr),
		.dbg_data(dbg_last_data),
		.dbg_rw(dbg_last_rw),
		.dbg_ipl(ipl),
		.dbg_int_cnt(dbg_intack_cnt),
		.dbg_flw_cnt(dbg_fl_wr_cnt),
		.dbg_lcd_on(lcd_on),
		.dbg_protect(protect),
	.dbg_stopped(stopped),
	.dbg_ai7(int_pend[7]),   // LIVE AI7 pending flag (not the sticky latch)
	.boot_status(boot_status),

	// Bus trace: raw CPU bus for the fault-triggered ring in dbg_uart
	.tr_addr({cpu_addr, 1'b0}),
	.tr_data(cpu_rw_n ? cpu_din : cpu_dout),
	.tr_rw(cpu_rw_n),
	.tr_fc(cpu_fc),
	.tr_as_n(cpu_as_n),
	.tr_ai7(ai7_hit),
	.tr_boot_done(boot_done),

	.dump_active(dump_active),
	.dump_stb(dump_stb),
	.dump_word(dump_word),
	.dump_pass_stb(dump_pass_stb),
	.dump_rdy(dump_rdy),

	.txd(dbg_uart_txd)
);

	video_scaler video_scaler
	(
		.clk(clk_sys),
		.reset(reset),

		.pix_ce(pixel_valid),
		.pixel(pixel_out),

		.scale_sel(status[5:4]),
		.color_sel(status[3:2]),
		.boot_status(boot_status),

		.dbg_en(~status[6]),
		.dbg_pc(disp_pc),
		.dbg_addr(disp_addr),
		.dbg_data(disp_data),
		.dbg_rw(disp_rw),
		.dbg_ipl(disp_ipl),
		.dbg_int_cnt(disp_int_cnt),
		.dbg_flw_cnt(disp_flw_cnt),
		.dbg_lcd_on(disp_lcd_on),
		.dbg_protect(disp_protect),
		.dbg_stopped(disp_stopped),
		.dbg_ai7(disp_ai7),

		.ce_pix(CE_PIXEL),
		.R(VGA_R),
		.G(VGA_G),
		.B(VGA_B),
		.HSync(VGA_HS),
		.VSync(VGA_VS),
		.DE(VGA_DE)
	);

	///////////////////////////////////////////////////////////////////////////
	// Fixed / unused emu outputs
	///////////////////////////////////////////////////////////////////////////

	// Active LCD area is 160x100 with square pixels -> 16:10
	assign VIDEO_ARX  = 13'd16;
	assign VIDEO_ARY  = 13'd10;

	assign VGA_F1      = 1'b0;
	assign VGA_SL      = 2'b00;   // no scanline effect
	assign VGA_SCALER  = 1'b0;
	assign VGA_DISABLE = 1'b0;

	assign HDMI_FREEZE    = 1'b0;
	assign HDMI_BLACKOUT  = 1'b0;
	assign HDMI_BOB_DEINT = 1'b0;

`ifdef MISTER_FB
	assign FB_EN           = 1'b0;
	assign FB_FORMAT       = 5'd0;
	assign FB_WIDTH        = 12'd0;
	assign FB_HEIGHT       = 12'd0;
	assign FB_BASE         = 32'd0;
	assign FB_STRIDE       = 14'd0;
	assign FB_FORCE_BLANK  = 1'b0;
`ifdef MISTER_FB_PALETTE
	assign FB_PAL_CLK   = 1'b0;
	assign FB_PAL_ADDR  = 8'd0;
	assign FB_PAL_DOUT  = 24'd0;
	assign FB_PAL_WR    = 1'b0;
`endif
`endif

`ifdef MISTER_DUAL_SDRAM
	assign SDRAM2_CLK  = 1'bz;
	assign SDRAM2_A    = 13'hZZZZ;
	assign SDRAM2_BA   = 2'bZZ;
	assign SDRAM2_DQ   = 16'hZZZZ;
	assign SDRAM2_nCS  = 1'bz;
	assign SDRAM2_nCAS = 1'bz;
	assign SDRAM2_nRAS = 1'bz;
	assign SDRAM2_nWE  = 1'bz;
`endif

	// LED on while an OS image is being loaded
	assign LED_USER  = loading;
	assign LED_POWER = 2'b00;
	assign LED_DISK  = 2'b00;
	assign BUTTONS   = 2'b00;

	// The TI-89 has no sound hardware
	assign AUDIO_L   = 16'd0;
	assign AUDIO_R   = 16'd0;
	assign AUDIO_S   = 1'b0;
	assign AUDIO_MIX = 2'b00;

	assign ADC_BUS = 4'bZZZZ;

	assign SD_SCK  = 1'b0;
	assign SD_MOSI = 1'b0;
	assign SD_CS   = 1'b1;

	assign DDRAM_CLK      = 1'b0;
	assign DDRAM_BURSTCNT = 8'd0;
	assign DDRAM_ADDR     = 29'd0;
	assign DDRAM_RD       = 1'b0;
	assign DDRAM_DIN      = 64'd0;
	assign DDRAM_BE       = 8'd0;
	assign DDRAM_WE       = 1'b0;

	assign UART_RTS = 1'b0;
	assign UART_TXD = dbg_uart_txd;
	assign UART_DTR = 1'b0;

	assign USER_OUT = 7'h7F;

endmodule
