module ControlUnit (
    input  [6:0] opcode,
    input  [2:0] funct3,
    input  [6:0] funct7,

    output reg RegWrite,
    output reg ALUSrc,
    output reg MemRead,
    output reg MemWrite,
    output reg MemToReg,
    output reg Branch,
    output reg [3:0] ALUControl
);

always @(*) begin
    // Default values
    RegWrite  = 0;
    ALUSrc    = 0;
    MemRead   = 0;
    MemWrite  = 0;
    MemToReg = 0;
    Branch    = 0;
    ALUControl = 4'b0000;

    case (opcode)

        // R-Type
        7'b0110011: begin
            RegWrite = 1;
            case (funct3)
                3'b000: ALUControl = (funct7[5] ? 4'b0001 : 4'b0000); // sub/add
                3'b111: ALUControl = 4'b0010; // and
                3'b110: ALUControl = 4'b0011; // or
                3'b100: ALUControl = 4'b0100; // xor
                3'b001: ALUControl = 4'b0101; // sll
                3'b101: ALUControl = 4'b0110; // srl
                3'b010: ALUControl = 4'b0111; // slt
            endcase
        end

        // I-Type Arithmetic
        7'b0010011: begin
            RegWrite = 1;
            ALUSrc   = 1;
            case (funct3)
                3'b000: ALUControl = 4'b0000; // addi
                3'b111: ALUControl = 4'b0010; // andi
                3'b110: ALUControl = 4'b0011; // ori
                3'b100: ALUControl = 4'b0100; // xori
                3'b001: ALUControl = 4'b0101; // slli
                3'b101: ALUControl = 4'b0110; // srli
            endcase
        end

        // Load
        7'b0000011: begin
            RegWrite  = 1;
            ALUSrc    = 1;
            MemRead   = 1;
            MemToReg  = 1;
            ALUControl = 4'b0000;
        end

        // Store
        7'b0100011: begin
            ALUSrc    = 1;
            MemWrite  = 1;
            ALUControl = 4'b0000;
        end

        // Branch
        7'b1100011: begin
            Branch    = 1;
            ALUControl = 4'b0001;
        end

    endcase
end

endmodule

//---------------------------------------
//testbench 

module ControlUnit_tb;

reg [6:0] opcode;
reg [2:0] funct3;
reg [6:0] funct7;

wire RegWrite, ALUSrc, MemRead, MemWrite, MemToReg, Branch;
wire [3:0] ALUControl;

ControlUnit dut (
    opcode, funct3, funct7,
    RegWrite, ALUSrc, MemRead, MemWrite, MemToReg, Branch, ALUControl
);

initial begin
    // ADD
    opcode = 7'b0110011; funct3 = 3'b000; funct7 = 7'b0000000;
    #10;

    // SUB
    funct7 = 7'b0100000;
    #10;

    // ADDI
    opcode = 7'b0010011; funct3 = 3'b000;
    #10;

    // LOAD
    opcode = 7'b0000011;
    #10;

    // BEQ
    opcode = 7'b1100011;
    #10;

    $stop;
end

endmodule


