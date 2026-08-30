//
// dbg_uart.sv — Live UART diagnostic reporter for TI-89 MiSTer Core
//
// Transmits live CPU execution status, PC, address, data, interrupt count,
// and hardware flags over the MiSTer UART interface (HPS UART /dev/ttyS1)
// at 115,200 baud (8N1) at ~4 Hz (every 250 ms).
//
// Access from MiSTer Linux over SSH:
//   stty -F /dev/ttyS1 115200 raw -echo && cat /dev/ttyS1
//
// 89u file location on SD card:
//   /media/fat/games/TI89/TI89Titanium_OS.89u
//

module dbg_uart (
    input             clk,          // 60 MHz master clock
    input             reset,

    input      [23:0] dbg_pc,
    input      [23:0] dbg_addr,
    input      [15:0] dbg_data,
    input             dbg_rw,
    input       [2:0] dbg_ipl,
    input      [15:0] dbg_int_cnt,
    input      [15:0] dbg_flw_cnt,
    input             dbg_lcd_on,
    input             dbg_protect,
    input             dbg_stopped,
    input             dbg_ai7,
    input       [2:0] boot_status,

    output reg        txd
);

    // 115200 baud @ 60 MHz: 60,000,000 / 115200 = 521 clocks/bit
    localparam [9:0]  BIT_PERIOD    = 10'd520;
    localparam [23:0] REPEAT_PERIOD = 24'd15_000_000; // ~250 ms (4 lines/sec)
    localparam [6:0]  MSG_LEN       = 7'd82;

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_LOAD = 2'd1;
    localparam [1:0] S_SEND = 2'd2;

    reg  [1:0] state;
    reg [23:0] period_cnt;
    reg  [9:0] baud_cnt;
    reg  [3:0] bit_idx;
    reg  [6:0] char_idx;
    reg  [9:0] tx_shift;

    // Snapshot registers (latched when a new line begins)
    reg [23:0] snap_pc;
    reg [23:0] snap_addr;
    reg [15:0] snap_data;
    reg        snap_rw;
    reg  [2:0] snap_ipl;
    reg [15:0] snap_int_cnt;
    reg [15:0] snap_flw_cnt;
    reg        snap_lcd_on;
    reg        snap_protect;
    reg        snap_stopped;
    reg        snap_ai7;
    reg  [2:0] snap_boot_status;

    function [7:0] hex2ascii;
        input [3:0] nib;
        begin
            hex2ascii = (nib < 4'd10) ? (8'h30 + {4'd0, nib}) : (8'h41 + {4'd0, nib - 4'd10});
        end
    endfunction

    reg [7:0] tx_byte;
    always @(*) begin
        case (char_idx)
            7'd0:  tx_byte = "[";
            7'd1:  tx_byte = "T";
            7'd2:  tx_byte = "I";
            7'd3:  tx_byte = "8";
            7'd4:  tx_byte = "9";
            7'd5:  tx_byte = "]";
            7'd6:  tx_byte = " ";
            7'd7:  tx_byte = "P";
            7'd8:  tx_byte = "C";
            7'd9:  tx_byte = "=";
            7'd10: tx_byte = hex2ascii(snap_pc[23:20]);
            7'd11: tx_byte = hex2ascii(snap_pc[19:16]);
            7'd12: tx_byte = hex2ascii(snap_pc[15:12]);
            7'd13: tx_byte = hex2ascii(snap_pc[11:8]);
            7'd14: tx_byte = hex2ascii(snap_pc[7:4]);
            7'd15: tx_byte = hex2ascii(snap_pc[3:0]);
            7'd16: tx_byte = " ";
            7'd17: tx_byte = "A";
            7'd18: tx_byte = "=";
            7'd19: tx_byte = hex2ascii(snap_addr[23:20]);
            7'd20: tx_byte = hex2ascii(snap_addr[19:16]);
            7'd21: tx_byte = hex2ascii(snap_addr[15:12]);
            7'd22: tx_byte = hex2ascii(snap_addr[11:8]);
            7'd23: tx_byte = hex2ascii(snap_addr[7:4]);
            7'd24: tx_byte = hex2ascii(snap_addr[3:0]);
            7'd25: tx_byte = " ";
            7'd26: tx_byte = "D";
            7'd27: tx_byte = "=";
            7'd28: tx_byte = hex2ascii(snap_data[15:12]);
            7'd29: tx_byte = hex2ascii(snap_data[11:8]);
            7'd30: tx_byte = hex2ascii(snap_data[7:4]);
            7'd31: tx_byte = hex2ascii(snap_data[3:0]);
            7'd32: tx_byte = " ";
            7'd33: tx_byte = snap_rw ? "R" : "W";
            7'd34: tx_byte = snap_rw ? "D" : "R";
            7'd35: tx_byte = " ";
            7'd36: tx_byte = "I";
            7'd37: tx_byte = "P";
            7'd38: tx_byte = "L";
            7'd39: tx_byte = "=";
            7'd40: tx_byte = hex2ascii({1'b0, snap_ipl});
            7'd41: tx_byte = " ";
            7'd42: tx_byte = "I";
            7'd43: tx_byte = "N";
            7'd44: tx_byte = "T";
            7'd45: tx_byte = "=";
            7'd46: tx_byte = hex2ascii(snap_int_cnt[15:12]);
            7'd47: tx_byte = hex2ascii(snap_int_cnt[11:8]);
            7'd48: tx_byte = hex2ascii(snap_int_cnt[7:4]);
            7'd49: tx_byte = hex2ascii(snap_int_cnt[3:0]);
            7'd50: tx_byte = " ";
            7'd51: tx_byte = "F";
            7'd52: tx_byte = "L";
            7'd53: tx_byte = "W";
            7'd54: tx_byte = "=";
            7'd55: tx_byte = hex2ascii(snap_flw_cnt[15:12]);
            7'd56: tx_byte = hex2ascii(snap_flw_cnt[11:8]);
            7'd57: tx_byte = hex2ascii(snap_flw_cnt[7:4]);
            7'd58: tx_byte = hex2ascii(snap_flw_cnt[3:0]);
            7'd59: tx_byte = " ";
            7'd60: tx_byte = "L";
            7'd61: tx_byte = "=";
            7'd62: tx_byte = snap_lcd_on ? "1" : "0";
            7'd63: tx_byte = " ";
            7'd64: tx_byte = "P";
            7'd65: tx_byte = "=";
            7'd66: tx_byte = snap_protect ? "1" : "0";
            7'd67: tx_byte = " ";
            7'd68: tx_byte = "S";
            7'd69: tx_byte = "=";
            7'd70: tx_byte = snap_stopped ? "1" : "0";
            7'd71: tx_byte = " ";
            7'd72: tx_byte = "7";
            7'd73: tx_byte = "=";
            7'd74: tx_byte = snap_ai7 ? "1" : "0";
            7'd75: tx_byte = " ";
            7'd76: tx_byte = "S";
            7'd77: tx_byte = "T";
            7'd78: tx_byte = "=";
            7'd79: tx_byte = hex2ascii({1'b0, snap_boot_status});
            7'd80: tx_byte = 8'h0D; // '\r'
            7'd81: tx_byte = 8'h0A; // '\n'
            default: tx_byte = 8'h20;
        endcase
    end

    always @(posedge clk) begin
        if (reset) begin
            state            <= S_IDLE;
            period_cnt       <= 24'd0;
            baud_cnt         <= 10'd0;
            bit_idx          <= 4'd0;
            char_idx         <= 7'd0;
            tx_shift         <= 10'h3FF;
            txd              <= 1'b1;
            snap_pc          <= 24'd0;
            snap_addr        <= 24'd0;
            snap_data        <= 16'd0;
            snap_rw          <= 1'b1;
            snap_ipl         <= 3'd0;
            snap_int_cnt     <= 16'd0;
            snap_flw_cnt     <= 16'd0;
            snap_lcd_on      <= 1'b0;
            snap_protect     <= 1'b0;
            snap_stopped     <= 1'b0;
            snap_ai7         <= 1'b0;
            snap_boot_status <= 3'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    txd <= 1'b1;
                    if (period_cnt >= REPEAT_PERIOD) begin
                        period_cnt       <= 24'd0;
                        snap_pc          <= dbg_pc;
                        snap_addr        <= dbg_addr;
                        snap_data        <= dbg_data;
                        snap_rw          <= dbg_rw;
                        snap_ipl         <= dbg_ipl;
                        snap_int_cnt     <= dbg_int_cnt;
                        snap_flw_cnt     <= dbg_flw_cnt;
                        snap_lcd_on      <= dbg_lcd_on;
                        snap_protect     <= dbg_protect;
                        snap_stopped     <= dbg_stopped;
                        snap_ai7         <= dbg_ai7;
                        snap_boot_status <= boot_status;
                        char_idx         <= 7'd0;
                        state            <= S_LOAD;
                    end else begin
                        period_cnt <= period_cnt + 24'd1;
                    end
                end

                S_LOAD: begin
                    // Load byte into shift register with start bit (0) and stop bit (1)
                    tx_shift <= {1'b1, tx_byte, 1'b0};
                    bit_idx  <= 4'd0;
                    baud_cnt <= 10'd0;
                    state    <= S_SEND;
                end

                S_SEND: begin
                    txd <= tx_shift[0];
                    if (baud_cnt >= BIT_PERIOD) begin
                        baud_cnt <= 10'd0;
                        tx_shift <= {1'b1, tx_shift[9:1]};
                        if (bit_idx == 4'd9) begin
                            // Byte completed
                            if (char_idx == MSG_LEN - 7'd1) begin
                                state <= S_IDLE;
                            end else begin
                                char_idx <= char_idx + 7'd1;
                                state    <= S_LOAD;
                            end
                        end else begin
                            bit_idx <= bit_idx + 4'd1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 10'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
