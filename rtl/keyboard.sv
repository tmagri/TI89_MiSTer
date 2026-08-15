//
// keyboard.sv — TI-89 Keyboard Matrix Controller
// TI-89 MiSTer Core
//
// Implements the 10×8 keyboard scan matrix from tiemu kbd.c (keyRow89)
// and maps PS/2 scancodes from MiSTer hps_io to TI key positions.
//
// Matrix layout (from tiemu/src/core/ti_hw/kbd.c):
//   Row 0: ALPHA,   DIAMOND, SHIFT,   2ND,     RIGHT,   DOWN,    LEFT,    UP
//   Row 1: F5,      CLEAR,   POWER,   DIVIDE,  MULTIPLY,MINUS,   PLUS,    ENTER
//   Row 2: F4,      BACKSP,  T,       COMMA,   9,       6,       3,       NEGATE
//   Row 3: F3,      CATALOG, Z,       PARIGHT, 8,       5,       2,       PERIOD
//   Row 4: F2,      MODE,    Y,       PALEFT,  7,       4,       1,       0
//   Row 5: F1,      HOME,    X,       EQUALS,  PIPE,    EE,      STORE,   APPS
//   Row 6: (void except col 7 = ESC)
//   Rows 7-9: void
//

module keyboard (
    input         clk,
    input         reset,

    // PS/2 keyboard from hps_io
    input  [10:0] ps2_key,      // {toggle, pressed, extended, scancode[7:0]}

    // Matrix scan interface (directly called by I/O ports)
    input   [9:0] row_mask,     // Row selection mask (from $600018-$600019)
    output  [7:0] col_data,     // Column data (active low — returned at $60001B)

    // ON key output (directly triggers AI6)
    output reg    on_key,       // ON key state (active high)
    output reg    on_key_press, // ON key edge (1 cycle pulse on press)

    // Keyboard interrupt output
    output reg    kbd_int       // Key state change (triggers AI2)
);

    // =========================================================================
    // Key state storage
    // Each bit represents whether a TI key is currently pressed.
    // We use a flat 80-bit register for the 10x8 matrix.
    // =========================================================================

    reg [7:0] key_matrix [0:9]; // 10 rows × 8 columns

    // =========================================================================
    // PS/2 scancode → matrix position mapping
    // Following TiEmu default keyboard mappings
    // =========================================================================
    //
    // PS/2 Set 2 scancodes used by MiSTer hps_io:
    // The ps2_key signal format (see hps_io.sv):
    //   ps2_key[10]   — toggles on every key event (strobe)
    //   ps2_key[9]    — pressed (1) / released (0)
    //   ps2_key[8]    — extended scancode prefix (0xE0)
    //   ps2_key[7:0]  — scancode
    //
    // We decode the scancode and map to {row, col} pairs.

    wire       ps2_strobe  = ps2_key[10];
    wire       ps2_pressed = ps2_key[9];
    wire       ps2_ext     = ps2_key[8];
    wire [7:0] ps2_code    = ps2_key[7:0];

    reg ps2_strobe_prev;

    always @(posedge clk) begin
        if (reset) begin
            integer i;
            for (i = 0; i < 10; i = i + 1)
                key_matrix[i] <= 8'h00;
            on_key        <= 1'b0;
            on_key_press  <= 1'b0;
            kbd_int       <= 1'b0;
            ps2_strobe_prev <= 1'b0;
        end else begin
            on_key_press <= 1'b0;
            kbd_int      <= 1'b0;
            ps2_strobe_prev <= ps2_strobe;

            // Process on rising edge of strobe
            if (ps2_strobe && !ps2_strobe_prev) begin
                reg [3:0] row;
                reg [2:0] col;
                reg       valid;
                reg       is_on_key;

                valid     = 1'b0;
                is_on_key = 1'b0;
                row       = 4'd0;
                col       = 3'd0;

                // Non-extended scancodes
                if (!ps2_ext) begin
                    case (ps2_code)
                        // Arrow keys (regular — some keyboards)
                        // Row 0: direction keys
                        8'h58: begin row = 4'd0; col = 3'd0; valid = 1'b1; end  // Caps Lock → ALPHA
                        8'h12: begin row = 4'd0; col = 3'd2; valid = 1'b1; end  // Left Shift → SHIFT
                        8'h14: begin row = 4'd0; col = 3'd1; valid = 1'b1; end  // Left Ctrl → DIAMOND
                        8'h11: begin row = 4'd0; col = 3'd3; valid = 1'b1; end  // Left Alt → 2ND

                        // Function keys → F1-F5
                        8'h05: begin row = 4'd5; col = 3'd0; valid = 1'b1; end  // F1 → TIKEY_F1
                        8'h06: begin row = 4'd4; col = 3'd0; valid = 1'b1; end  // F2 → TIKEY_F2
                        8'h04: begin row = 4'd3; col = 3'd0; valid = 1'b1; end  // F3 → TIKEY_F3
                        8'h0C: begin row = 4'd2; col = 3'd0; valid = 1'b1; end  // F4 → TIKEY_F4
                        8'h03: begin row = 4'd1; col = 3'd0; valid = 1'b1; end  // F5 → TIKEY_F5

                        // F6-F8 → special calc keys
                        8'h0B: begin row = 4'd3; col = 3'd1; valid = 1'b1; end  // F6 → CATALOG
                        8'h83: begin row = 4'd5; col = 3'd1; valid = 1'b1; end  // F7 → HOME
                        8'h0A: begin row = 4'd4; col = 3'd1; valid = 1'b1; end  // F8 → MODE

                        // Number row
                        8'h45: begin row = 4'd4; col = 3'd7; valid = 1'b1; end  // 0 → TIKEY_0
                        8'h16: begin row = 4'd4; col = 3'd6; valid = 1'b1; end  // 1 → TIKEY_1
                        8'h1E: begin row = 4'd3; col = 3'd6; valid = 1'b1; end  // 2 → TIKEY_2
                        8'h26: begin row = 4'd2; col = 3'd6; valid = 1'b1; end  // 3 → TIKEY_3
                        8'h25: begin row = 4'd4; col = 3'd5; valid = 1'b1; end  // 4 → TIKEY_4
                        8'h2E: begin row = 4'd3; col = 3'd5; valid = 1'b1; end  // 5 → TIKEY_5
                        8'h36: begin row = 4'd2; col = 3'd5; valid = 1'b1; end  // 6 → TIKEY_6
                        8'h3D: begin row = 4'd4; col = 3'd4; valid = 1'b1; end  // 7 → TIKEY_7
                        8'h3E: begin row = 4'd3; col = 3'd4; valid = 1'b1; end  // 8 → TIKEY_8
                        8'h46: begin row = 4'd2; col = 3'd4; valid = 1'b1; end  // 9 → TIKEY_9

                        // Letter keys (X, Y, Z, T used directly)
                        8'h22: begin row = 4'd5; col = 3'd2; valid = 1'b1; end  // X → TIKEY_X
                        8'h35: begin row = 4'd4; col = 3'd2; valid = 1'b1; end  // Y → TIKEY_Y
                        8'h1A: begin row = 4'd3; col = 3'd2; valid = 1'b1; end  // Z → TIKEY_Z
                        8'h2C: begin row = 4'd2; col = 3'd2; valid = 1'b1; end  // T → TIKEY_T

                        // Operators
                        8'h79: begin row = 4'd1; col = 3'd6; valid = 1'b1; end  // Numpad + → PLUS
                        8'h7B: begin row = 4'd1; col = 3'd5; valid = 1'b1; end  // Numpad - → MINUS
                        8'h7C: begin row = 4'd1; col = 3'd4; valid = 1'b1; end  // Numpad * → MULTIPLY
                        // Numpad / is extended, handled below

                        8'h55: begin row = 4'd5; col = 3'd3; valid = 1'b1; end  // = → EQUALS
                        8'h54: begin row = 4'd3; col = 3'd3; valid = 1'b1; end  // [ → PARIGHT (using [ for ))
                        8'h5B: begin row = 4'd4; col = 3'd3; valid = 1'b1; end  // ] → PALEFT (using ] for ()
                        8'h41: begin row = 4'd2; col = 3'd3; valid = 1'b1; end  // , → COMMA
                        8'h49: begin row = 4'd3; col = 3'd7; valid = 1'b1; end  // . → PERIOD
                        8'h4E: begin row = 4'd1; col = 3'd5; valid = 1'b1; end  // - → MINUS

                        // Special keys
                        8'h76: begin row = 4'd6; col = 3'd7; valid = 1'b1; end  // ESC → ESCAPE
                        8'h66: begin row = 4'd2; col = 3'd1; valid = 1'b1; end  // Backspace → BACKSPACE
                        8'h5A: begin row = 4'd1; col = 3'd7; valid = 1'b1; end  // Enter → ENTER
                        8'h0D: begin row = 4'd5; col = 3'd6; valid = 1'b1; end  // Tab → STORE
                        8'h29: begin row = 4'd2; col = 3'd7; valid = 1'b1; end  // Space → NEGATE

                        // Power (^) — using the ` key or dedicated
                        8'h0E: begin row = 4'd1; col = 3'd2; valid = 1'b1; end  // ` → POWER

                        default: valid = 1'b0;
                    endcase
                end else begin
                    // Extended scancodes (ps2_ext = 1)
                    case (ps2_code)
                        // Arrow keys
                        8'h6B: begin row = 4'd0; col = 3'd6; valid = 1'b1; end  // Left → LEFT
                        8'h72: begin row = 4'd0; col = 3'd5; valid = 1'b1; end  // Down → DOWN
                        8'h74: begin row = 4'd0; col = 3'd4; valid = 1'b1; end  // Right → RIGHT
                        8'h75: begin row = 4'd0; col = 3'd7; valid = 1'b1; end  // Up → UP

                        // Extended special keys
                        8'h71: begin row = 4'd1; col = 3'd1; valid = 1'b1; end  // Delete → CLEAR
                        8'h70: is_on_key = 1'b1;                                  // Insert → ON key
                        8'h6C: begin row = 4'd5; col = 3'd1; valid = 1'b1; end  // Home → HOME
                        8'h7D: begin row = 4'd5; col = 3'd7; valid = 1'b1; end  // Page Up → APPS
                        8'h7A: begin row = 4'd5; col = 3'd5; valid = 1'b1; end  // Page Down → EE
                        8'h69: begin row = 4'd3; col = 3'd1; valid = 1'b1; end  // End → CATALOG
                        8'h4A: begin row = 4'd1; col = 3'd3; valid = 1'b1; end  // Numpad / → DIVIDE

                        // Pipe key (backslash)
                        8'h5D: begin row = 4'd5; col = 3'd4; valid = 1'b1; end  // \ → PIPE

                        default: valid = 1'b0;
                    endcase
                end

                // Apply key state change
                if (valid) begin
                    key_matrix[row][col] <= ps2_pressed;
                    if (ps2_pressed)
                        kbd_int <= 1'b1; // Trigger AI2 on key press
                end

                if (is_on_key) begin
                    on_key <= ps2_pressed;
                    if (ps2_pressed)
                        on_key_press <= 1'b1; // Edge trigger for AI6
                end
            end
        end
    end

    // =========================================================================
    // Column readback logic
    // =========================================================================
    // When CPU reads $60001B, active rows (row_mask bits = 0) contribute
    // their key states to the column output. Result is inverted (active low).
    // This matches hw_kbd_read_cols() in kbd.c.

    reg [7:0] col_or;
    integer r;

    always @(*) begin
        col_or = 8'h00;
        for (r = 0; r < 10; r = r + 1) begin
            if (!row_mask[r])
                col_or = col_or | key_matrix[r];
        end
    end

    assign col_data = ~col_or; // Active low output

endmodule
