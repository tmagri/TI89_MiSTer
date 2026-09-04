//
// sdram_chip.sv — behavioral model of the DE10-Nano SDRAM chip
// (IS42S16160G-class: 4 banks x 8192 rows x 512 cols x 16 bits).
//
// Samples commands on its CLK input (the controller drives SDRAM_CLK =
// ~clk, so chip edges fall half a cycle after controller edges — same as
// real hardware). CAS latency 2, write latency 1. Refresh and mode
// register are accepted and ignored (no data retention modeling).
//
// Storage is flat: word index = {bank, row, col} = the controller's own
// address decomposition, so the layout matches the old ideal model
// exactly (flash image at word 0, calculator RAM at word 0x200000).
//
// DQ is split into DQ_IN / DQ_OUT / DQ_OE so the testbench can arbitrate
// the bidirectional line with a single driver.
//

module sdram_chip (
    input         CLK,     // = SDRAM_CLK (inverted master clock)
    input         CKE,
    input  [12:0] A,
    input   [1:0] BA,
    input  [15:0] DQ_IN,   // what the controller is driving
    output reg [15:0] DQ_OUT,
    output reg     DQ_OE,
    input         DQML,
    input         DQMH,
    input         nCS,
    input         nRAS,
    input         nCAS,
    input         nWE
);

    reg [15:0] mem [0:4194303];
    initial begin
        integer i;
        for (i = 0; i < 4194304; i = i + 1) mem[i] = 16'hFFFF;
        $readmemh("flash.hex", mem);
    end

    reg [12:0] act_row [0:3];
    reg        act_on  [0:3];

    reg [23:0] rd_addr;
    reg [23:0] wr_addr;
    reg  [2:0] cl;   // CAS-latency countdown (read)
    reg  [1:0] wl;   // write-latency countdown

    wire [2:0] cmd = {nRAS, nCAS, nWE};

    always @(posedge CLK) begin
        if (cl != 3'd0) begin
            cl <= cl - 3'd1;
            if (cl == 3'd1) begin
                DQ_OE  <= 1'b1;
                DQ_OUT <= mem[rd_addr];
            end
        end else if (DQ_OE) begin
            DQ_OE  <= 1'b0;
            DQ_OUT <= 16'h0000;
        end

        if (wl != 2'd0) begin
            wl <= wl - 2'd1;
            if (wl == 2'd1) begin
                if (!DQMH) mem[wr_addr][15:8] <= DQ_IN[15:8];
                if (!DQML) mem[wr_addr][7:0]  <= DQ_IN[7:0];
            end
        end

        if (!nCS) begin
            case (cmd)
                3'b011: begin // ACTIVATE
                    act_row[BA] <= A;
                    act_on[BA]  <= 1'b1;
                end
                3'b101: begin // READ
                    rd_addr <= {BA, act_row[BA], A[8:0]};
                    cl      <= 3'd2;
                end
                3'b100: begin // WRITE
                    wr_addr <= {BA, act_row[BA], A[8:0]};
                    wl      <= 2'd1;
                end
                3'b010: begin // PRECHARGE
                    DQ_OE <= 1'b0;
                    if (A[10]) begin
                        act_on[0] <= 1'b0; act_on[1] <= 1'b0;
                        act_on[2] <= 1'b0; act_on[3] <= 1'b0;
                    end else
                        act_on[BA] <= 1'b0;
                end
                default: ;    // REF / LMR / NOP
            endcase
        end
    end

endmodule
