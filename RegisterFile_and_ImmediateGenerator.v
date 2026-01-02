`timescale 1ns/1ps

module RegisterFile (
    input  wire        clk,
    input  wire        RegWrite,
    input  wire [4:0]  rs1,
    input  wire [4:0]  rs2,
    input  wire [4:0]  rd,
    input  wire [63:0] writeData,
    output wire [63:0] readData1,
    output wire [63:0] readData2
);

    reg [63:0] registers [0:31];

    assign readData1 = (rs1 == 0) ? 64'b0 : registers[rs1];
    assign readData2 = (rs2 == 0) ? 64'b0 : registers[rs2];

    always @(posedge clk) begin
        if (RegWrite && rd != 0)
            registers[rd] <= writeData;
    end

endmodule

module ImmediateGenerator (
    input  wire [31:0] instruction,
    output reg  [63:0] immediate
);

    wire [6:0] opcode;
    assign opcode = instruction[6:0];

    always @(*) begin
        case (opcode)

            7'b0010011,   
            7'b0000011:   
                immediate = {{52{instruction[31]}}, instruction[31:20]};

            7'b0100011:
                immediate = {{52{instruction[31]}},
                             instruction[31:25],
                             instruction[11:7]};

            7'b1100011:
                immediate = {{51{instruction[31]}},
                             instruction[31],
                             instruction[7],
                             instruction[30:25],
                             instruction[11:8],
                             1'b0};

            default:
                immediate = 64'b0;

        endcase
    end

endmodule


module Task3_tb;

    // -------- Clock --------
    reg clk;
    initial clk = 0;
    always #5 clk = ~clk;   // 10ns period

    // -------- Register File signals --------
    reg         RegWrite;
    reg  [4:0]  rs1, rs2, rd;
    reg  [63:0] writeData;
    wire [63:0] readData1, readData2;

    RegisterFile RF (
        .clk(clk),
        .RegWrite(RegWrite),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .writeData(writeData),
        .readData1(readData1),
        .readData2(readData2)
    );

    reg  [31:0] instruction;
    wire [63:0] immediate;

    ImmediateGenerator IG (
        .instruction(instruction),
        .immediate(immediate)
    );

    initial begin

	$dumpfile("Task3_tb.vcd");
        $dumpvars(0, Task3_tb);

        RegWrite   = 0;
        rs1        = 0;
        rs2        = 0;
        rd         = 0;
        writeData  = 0;
        instruction= 0;


        @(negedge clk);
        RegWrite  = 1;
        rd        = 0;            
        writeData = 64'd999;
        @(posedge clk);           

        @(negedge clk);
        rs1 = 0;
        #1;
        if (readData1 !== 64'd0) begin
            $display("FAIL: x0 changed! readData1=%0d", readData1);
            $stop;
        end else begin
            $display("PASS: x0 stays 0");
        end

        @(negedge clk);
        RegWrite  = 1;
        rd        = 5;
        writeData = 64'd42;
        @(posedge clk);

        @(negedge clk);
        rs1 = 5;
        #1;
        if (readData1 !== 64'd42) begin
            $display("FAIL: x5 expected 42, got %0d", readData1);
            $stop;
        end else begin
            $display("PASS: x5 write/read OK");
        end


        @(negedge clk);
        instruction = {12'hFFF, 5'd0, 3'b000, 5'd1, 7'b0010011}; 
        #1;
        if (immediate !== 64'hFFFFFFFFFFFFFFFF) begin
            $display("FAIL: I-type imm expected -1, got %h", immediate);
            $stop;
        end else begin
            $display("PASS: I-type immediate OK (-1)");
        end

        @(negedge clk);
        instruction = {7'b0000011, 5'd2, 5'd0, 3'b011, 5'b01000, 7'b0100011};
        #1;
        if (immediate !== 64'd104) begin
            $display("FAIL: S-type imm expected 104, got %0d", immediate);
            $stop;
        end else begin
            $display("PASS: S-type immediate OK (104)");
        end

        @(negedge clk);
        instruction = 32'b0000000_00010_00001_000_00100_1100011; 
        #1;
        $display("INFO: B-type immediate produced: %0d (sanity check)", immediate);

        $display("ALL TASK3 TESTS PASSED ");
        $stop;
    end

endmodule
