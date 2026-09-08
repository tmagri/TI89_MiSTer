//
// keyboard.sv — TI-89 Keyboard Matrix Controller
// TI-89 MiSTer Core
//
// Implements the 10x8 keyboard scan matrix from tiemu kbd.c (keyRow89)
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
// Letter key mapping (from tiemu/src/core/ti_hw/tichars.c ALPHA() macro):
//   T, X, Y, Z have dedicated keys in the matrix (no ALPHA needed).
//   All other letters use ALPHA + a secondary key:
//     A=EQUALS, B=PALEFT, C=PARIGHT, D=COMMA, E=DIVIDE, F=PIPE
//     G=7,      H=8,      I=9,       J=MULTIPLY, K=EE,  L=4
//     M=5,      N=6,      O=MINUS,   P=STORE,  Q=1,    R=2
//     S=3,      U=PLUS,   V=0,       W=PERIOD
//
// For letter keys A-S, U, V, W (not T/X/Y/Z), pressing the PC key
// simultaneously asserts ALPHA (row0,col0) AND the secondary TI key.
// A reference counter tracks how many letter keys are held so ALPHA
// is released only when the last letter key is released.
//

module keyboard (
    input         clk,
    input         reset,

    // PS/2 keyboard from hps_io
    input  [10:0] ps2_key,      // {toggle, pressed, extended, scancode[7:0]}

    // Matrix scan interface (directly called by I/O ports)
    input   [9:0] row_mask,     // Row selection mask (from $600018-$600019)
    output  [7:0] col_data,     // Column data (active low -- returned at $60001B)

    // ON key output (directly triggers AI6)
    output reg    on_key,       // ON key state (active high)
    output reg    on_key_press, // ON key edge (1 cycle pulse on press)

    // Keyboard interrupt output
    output reg    kbd_int       // Key state change (triggers AI2)
);

    // =========================================================================
    // Key state storage
    // Each bit represents whether a TI key is currently pressed.
    // key_matrix[row] stores 8 columns with bit(7-col) = col value,
    // matching TiEmu's get_rowmask() which maps col 0 -> bit 7, col 7 -> bit 0.
    // =========================================================================

    reg [7:0] key_matrix [0:9]; // 10 rows x 8 columns

    // =========================================================================
    // ALPHA auto-assertion for letter keys (A-S, U, V, W)
    // T, X, Y, Z have dedicated matrix positions and don't need auto-ALPHA.
    // We use a 5-bit saturating counter so ALPHA stays asserted as long as
    // any letter key (other than T/X/Y/Z) is held.
    // =========================================================================

    reg [4:0] alpha_count;   // Number of ALPHA-letter keys currently held

    // =========================================================================
    // PS/2 scancode -> matrix position mapping
    // Following TiEmu default keyboard mappings + tichars.c ALPHA() table
    // =========================================================================
    //
    // PS/2 Set 2 scancodes used by MiSTer hps_io:
    //   ps2_key[10] -- toggles on every key event (strobe)
    //   ps2_key[9]  -- pressed (1) / released (0)
    //   ps2_key[8]  -- extended scancode prefix (0xE0)
    //   ps2_key[7:0]-- scancode
    //
    // Column storage: key_matrix[row][7-col] per get_rowmask() convention.

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
            on_key          <= 1'b0;
            on_key_press    <= 1'b0;
            kbd_int         <= 1'b0;
            ps2_strobe_prev <= 1'b0;
            alpha_count     <= 5'd0;
        end else begin
            on_key_press <= 1'b0;
            kbd_int      <= 1'b0;
            ps2_strobe_prev <= ps2_strobe;

            // Process on any edge of the toggle bit (rising or falling).
            // hps_io alternates 0->1->0->1 on every event, so we detect both edges.
            if (ps2_strobe != ps2_strobe_prev) begin
                // -- Variables for this event -----------------------------------
                reg [3:0] row;
                reg [2:0] col;
                reg       valid;
                reg       is_on_key;
                reg       is_alpha_letter; // needs ALPHA auto-assert

                valid           = 1'b0;
                is_on_key       = 1'b0;
                is_alpha_letter = 1'b0;
                row             = 4'd0;
                col             = 3'd0;

                // -- Non-extended scancodes ------------------------------------
                if (!ps2_ext) begin
                    case (ps2_code)
                        // -- Row 0: modifier / lock keys -----------------------
                        8'h58: begin row = 4'd0; col = 3'd0; valid = 1'b1; end  // Caps Lock  -> ALPHA
                        8'h14: begin row = 4'd0; col = 3'd1; valid = 1'b1; end  // Left Ctrl  -> DIAMOND
                        8'h12: begin row = 4'd0; col = 3'd2; valid = 1'b1; end  // Left Shift -> SHIFT
                        8'h11: begin row = 4'd0; col = 3'd3; valid = 1'b1; end  // Left Alt   -> 2ND

                        // -- Function keys -------------------------------------
                        8'h05: begin row = 4'd5; col = 3'd0; valid = 1'b1; end  // F1 -> TIKEY_F1
                        8'h06: begin row = 4'd4; col = 3'd0; valid = 1'b1; end  // F2 -> TIKEY_F2
                        8'h04: begin row = 4'd3; col = 3'd0; valid = 1'b1; end  // F3 -> TIKEY_F3
                        8'h0C: begin row = 4'd2; col = 3'd0; valid = 1'b1; end  // F4 -> TIKEY_F4
                        8'h03: begin row = 4'd1; col = 3'd0; valid = 1'b1; end  // F5 -> TIKEY_F5

                        // F6/F7/F8 -> special calc keys
                        8'h0B: begin row = 4'd3; col = 3'd1; valid = 1'b1; end  // F6 -> CATALOG
                        8'h83: begin row = 4'd5; col = 3'd1; valid = 1'b1; end  // F7 -> HOME
                        8'h0A: begin row = 4'd4; col = 3'd1; valid = 1'b1; end  // F8 -> MODE

                        // -- Number row 0-9 ------------------------------------
                        8'h45: begin row = 4'd4; col = 3'd7; valid = 1'b1; end  // 0 -> TIKEY_0
                        8'h16: begin row = 4'd4; col = 3'd6; valid = 1'b1; end  // 1 -> TIKEY_1
                        8'h1E: begin row = 4'd3; col = 3'd6; valid = 1'b1; end  // 2 -> TIKEY_2
                        8'h26: begin row = 4'd2; col = 3'd6; valid = 1'b1; end  // 3 -> TIKEY_3
                        8'h25: begin row = 4'd4; col = 3'd5; valid = 1'b1; end  // 4 -> TIKEY_4
                        8'h2E: begin row = 4'd3; col = 3'd5; valid = 1'b1; end  // 5 -> TIKEY_5
                        8'h36: begin row = 4'd2; col = 3'd5; valid = 1'b1; end  // 6 -> TIKEY_6
                        8'h3D: begin row = 4'd4; col = 3'd4; valid = 1'b1; end  // 7 -> TIKEY_7
                        8'h3E: begin row = 4'd3; col = 3'd4; valid = 1'b1; end  // 8 -> TIKEY_8
                        8'h46: begin row = 4'd2; col = 3'd4; valid = 1'b1; end  // 9 -> TIKEY_9

                        // -- Letter keys: T, X, Y, Z -- dedicated matrix keys --
                        // (no ALPHA needed -- they are present in keyRow89 directly)
                        8'h2C: begin row = 4'd2; col = 3'd2; valid = 1'b1; end  // T -> TIKEY_T
                        8'h22: begin row = 4'd5; col = 3'd2; valid = 1'b1; end  // X -> TIKEY_X
                        8'h35: begin row = 4'd4; col = 3'd2; valid = 1'b1; end  // Y -> TIKEY_Y
                        8'h1A: begin row = 4'd3; col = 3'd2; valid = 1'b1; end  // Z -> TIKEY_Z

                        // -- Letter keys: ALPHA + secondary key ----------------
                        // From tichars.c: ALPHA(letter, ti89_key)
                        //   A=EQUALS(r5c3), B=PALEFT(r4c3),  C=PARIGHT(r3c3), D=COMMA(r2c3)
                        //   E=DIVIDE(r1c3), F=PIPE(r5c4),    G=7(r4c4),       H=8(r3c4)
                        //   I=9(r2c4),      J=MULTIPLY(r1c4),K=EE(r5c5),      L=4(r4c5)
                        //   M=5(r3c5),      N=6(r2c5),       O=MINUS(r1c5),   P=STORE(r5c6)
                        //   Q=1(r4c6),      R=2(r3c6),       S=3(r2c6),       U=PLUS(r1c6)
                        //   V=0(r4c7),      W=PERIOD(r3c7)
                        8'h1C: begin row = 4'd5; col = 3'd3; valid = 1'b1; is_alpha_letter = 1'b1; end  // A -> EQUALS
                        8'h32: begin row = 4'd4; col = 3'd3; valid = 1'b1; is_alpha_letter = 1'b1; end  // B -> PALEFT
                        8'h21: begin row = 4'd3; col = 3'd3; valid = 1'b1; is_alpha_letter = 1'b1; end  // C -> PARIGHT
                        8'h23: begin row = 4'd2; col = 3'd3; valid = 1'b1; is_alpha_letter = 1'b1; end  // D -> COMMA
                        8'h24: begin row = 4'd1; col = 3'd3; valid = 1'b1; is_alpha_letter = 1'b1; end  // E -> DIVIDE
                        8'h2B: begin row = 4'd5; col = 3'd4; valid = 1'b1; is_alpha_letter = 1'b1; end  // F -> PIPE
                        8'h34: begin row = 4'd4; col = 3'd4; valid = 1'b1; is_alpha_letter = 1'b1; end  // G -> 7
                        8'h33: begin row = 4'd3; col = 3'd4; valid = 1'b1; is_alpha_letter = 1'b1; end  // H -> 8
                        8'h43: begin row = 4'd2; col = 3'd4; valid = 1'b1; is_alpha_letter = 1'b1; end  // I -> 9
                        8'h3B: begin row = 4'd1; col = 3'd4; valid = 1'b1; is_alpha_letter = 1'b1; end  // J -> MULTIPLY
                        8'h42: begin row = 4'd5; col = 3'd5; valid = 1'b1; is_alpha_letter = 1'b1; end  // K -> EE
                        8'h4B: begin row = 4'd4; col = 3'd5; valid = 1'b1; is_alpha_letter = 1'b1; end  // L -> 4
                        8'h3A: begin row = 4'd3; col = 3'd5; valid = 1'b1; is_alpha_letter = 1'b1; end  // M -> 5
                        8'h31: begin row = 4'd2; col = 3'd5; valid = 1'b1; is_alpha_letter = 1'b1; end  // N -> 6
                        8'h44: begin row = 4'd1; col = 3'd5; valid = 1'b1; is_alpha_letter = 1'b1; end  // O -> MINUS
                        8'h4D: begin row = 4'd5; col = 3'd6; valid = 1'b1; is_alpha_letter = 1'b1; end  // P -> STORE
                        8'h15: begin row = 4'd4; col = 3'd6; valid = 1'b1; is_alpha_letter = 1'b1; end  // Q -> 1
                        8'h2D: begin row = 4'd3; col = 3'd6; valid = 1'b1; is_alpha_letter = 1'b1; end  // R -> 2
                        8'h1B: begin row = 4'd2; col = 3'd6; valid = 1'b1; is_alpha_letter = 1'b1; end  // S -> 3
                        8'h3C: begin row = 4'd1; col = 3'd6; valid = 1'b1; is_alpha_letter = 1'b1; end  // U -> PLUS
                        8'h2A: begin row = 4'd4; col = 3'd7; valid = 1'b1; is_alpha_letter = 1'b1; end  // V -> 0
                        8'h1D: begin row = 4'd3; col = 3'd7; valid = 1'b1; is_alpha_letter = 1'b1; end  // W -> PERIOD

                        // -- Numpad operators (Num Lock on, non-extended) ------
                        8'h79: begin row = 4'd1; col = 3'd6; valid = 1'b1; end  // Numpad + -> PLUS
                        8'h7B: begin row = 4'd1; col = 3'd5; valid = 1'b1; end  // Numpad - -> MINUS
                        8'h7C: begin row = 4'd1; col = 3'd4; valid = 1'b1; end  // Numpad * -> MULTIPLY

                        // -- Numpad 0-9 (Num Lock on) -- same as number row ---
                        8'h70: begin row = 4'd4; col = 3'd7; valid = 1'b1; end  // Numpad 0 -> TIKEY_0
                        8'h69: begin row = 4'd4; col = 3'd6; valid = 1'b1; end  // Numpad 1 -> TIKEY_1
                        8'h72: begin row = 4'd3; col = 3'd6; valid = 1'b1; end  // Numpad 2 -> TIKEY_2
                        8'h7A: begin row = 4'd2; col = 3'd6; valid = 1'b1; end  // Numpad 3 -> TIKEY_3
                        8'h6B: begin row = 4'd4; col = 3'd5; valid = 1'b1; end  // Numpad 4 -> TIKEY_4
                        8'h73: begin row = 4'd3; col = 3'd5; valid = 1'b1; end  // Numpad 5 -> TIKEY_5
                        8'h74: begin row = 4'd2; col = 3'd5; valid = 1'b1; end  // Numpad 6 -> TIKEY_6
                        8'h6C: begin row = 4'd4; col = 3'd4; valid = 1'b1; end  // Numpad 7 -> TIKEY_7
                        8'h75: begin row = 4'd3; col = 3'd4; valid = 1'b1; end  // Numpad 8 -> TIKEY_8
                        8'h7D: begin row = 4'd2; col = 3'd4; valid = 1'b1; end  // Numpad 9 -> TIKEY_9

                        // -- Mac / no-numpad alternatives ----------------------
                        8'h4A: begin row = 4'd1; col = 3'd3; valid = 1'b1; end  // / (slash)      -> DIVIDE
                        8'h52: begin row = 4'd1; col = 3'd6; valid = 1'b1; end  // ' (apostrophe) -> PLUS

                        // -- Punctuation / operators ---------------------------
                        8'h55: begin row = 4'd5; col = 3'd3; valid = 1'b1; end  // =  -> EQUALS
                        8'h54: begin row = 4'd4; col = 3'd3; valid = 1'b1; end  // [  -> PALEFT  (= '(')
                        8'h5B: begin row = 4'd3; col = 3'd3; valid = 1'b1; end  // ]  -> PARIGHT (= ')')
                        8'h41: begin row = 4'd2; col = 3'd3; valid = 1'b1; end  // ,  -> COMMA
                        8'h49: begin row = 4'd3; col = 3'd7; valid = 1'b1; end  // .  -> PERIOD
                        8'h4E: begin row = 4'd1; col = 3'd5; valid = 1'b1; end  // -  -> MINUS

                        // -- Special keys --------------------------------------
                        8'h76: begin row = 4'd6; col = 3'd7; valid = 1'b1; end  // ESC       -> ESCAPE
                        8'h66: begin row = 4'd2; col = 3'd1; valid = 1'b1; end  // Backspace -> BACKSPACE
                        8'h5A: begin row = 4'd1; col = 3'd7; valid = 1'b1; end  // Enter     -> ENTER
                        8'h0D: begin row = 4'd5; col = 3'd6; valid = 1'b1; end  // Tab       -> STORE
                        8'h29: begin row = 4'd2; col = 3'd7; valid = 1'b1; end  // Space     -> NEGATE
                        8'h0E: begin row = 4'd1; col = 3'd2; valid = 1'b1; end  // ` backtick -> POWER (^)
                        8'h5D: begin row = 4'd5; col = 3'd4; valid = 1'b1; end  // backslash -> PIPE

                        default: valid = 1'b0;
                    endcase
                end else begin
                    // -- Extended scancodes (ps2_ext = 1) ----------------------
                    case (ps2_code)
                        // Arrow keys
                        8'h6B: begin row = 4'd0; col = 3'd6; valid = 1'b1; end  // Left  -> LEFT
                        8'h72: begin row = 4'd0; col = 3'd5; valid = 1'b1; end  // Down  -> DOWN
                        8'h74: begin row = 4'd0; col = 3'd4; valid = 1'b1; end  // Right -> RIGHT
                        8'h75: begin row = 4'd0; col = 3'd7; valid = 1'b1; end  // Up    -> UP

                        // Extended special keys
                        8'h71: begin row = 4'd1; col = 3'd1; valid = 1'b1; end  // Delete    -> CLEAR
                        8'h70: is_on_key = 1'b1;                                  // Insert    -> ON key
                        8'h6C: begin row = 4'd5; col = 3'd1; valid = 1'b1; end  // Home      -> HOME
                        8'h7D: begin row = 4'd5; col = 3'd7; valid = 1'b1; end  // Page Up   -> APPS
                        8'h7A: begin row = 4'd5; col = 3'd5; valid = 1'b1; end  // Page Down -> EE
                        8'h69: begin row = 4'd3; col = 3'd1; valid = 1'b1; end  // End       -> CATALOG
                        8'h4A: begin row = 4'd1; col = 3'd3; valid = 1'b1; end  // Numpad /  -> DIVIDE

                        // Right Shift
                        8'h59: begin row = 4'd0; col = 3'd2; valid = 1'b1; end  // Right Shift -> SHIFT
                        // Right Alt (AltGr)
                        8'h11: begin row = 4'd0; col = 3'd3; valid = 1'b1; end  // Right Alt  -> 2ND

                        default: valid = 1'b0;
                    endcase
                end

                // -- Apply key state to matrix ---------------------------------
                // TiEmu get_rowmask(): col 0 -> bit 7, col 7 -> bit 0.
                // We store into bit (7-col).
                if (valid) begin
                    key_matrix[row][3'd7 - col] <= ps2_pressed;
                    if (ps2_pressed)
                        kbd_int <= 1'b1; // Trigger AI2 on key press
                end

                // -- ALPHA auto-assert for letter keys -------------------------
                // When a letter key (A-S, U, V, W) is pressed, also assert ALPHA.
                // Use a saturating counter so ALPHA releases when the last letter
                // key is released. ALPHA = row 0, col 0 -> bit 7 of key_matrix[0].
                if (is_alpha_letter) begin
                    if (ps2_pressed) begin
                        key_matrix[0][7] <= 1'b1;
                        alpha_count      <= alpha_count + 1;
                        kbd_int          <= 1'b1;
                    end else begin
                        if (alpha_count != 5'd0)
                            alpha_count <= alpha_count - 1;
                        if (alpha_count <= 5'd1) begin
                            // Last letter key released -- clear auto-ALPHA
                            key_matrix[0][7] <= 1'b0;
                        end
                    end
                end

                // -- ON key ---------------------------------------------------
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
    // When CPU reads $60001B, active rows (row_mask bit = 0) contribute
    // their key states to the column output. Result is inverted (active low).
    // This matches hw_kbd_read_cols() in tiemu/src/core/ti_hw/kbd.c.

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
