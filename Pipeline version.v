// Code your design here
`timescale 1ns/1ps

// ========== SIMPLE ALU ==========
module ALU(
    input  [63:0] A, B,
    input  [3:0]  ALUControl,
    output reg [63:0] Result,
    output Zero
);
    assign Zero = (Result == 64'b0);
    always @(*) begin
        case (ALUControl)
            4'b0000: Result = A + B;      // ADD
            4'b0001: Result = A - B;      // SUB
            4'b0010: Result = A & B;      // AND
            4'b0011: Result = A | B;      // OR
            4'b0100: Result = A ^ B;      // XOR
            4'b0101: Result = A << B[5:0]; // SLL
            4'b0110: Result = A >> B[5:0]; // SRL
            default: Result = 64'b0;
        endcase
    end
endmodule

// ========== SIMPLE CONTROL UNIT ==========
module ControlUnit(
    input  [6:0] opcode,
    input  [2:0] funct3,
    input  [6:0] funct7,
    output reg RegWrite,
    output reg ALUSrc,
    output reg MemRead,
    output reg MemWrite,
    output reg MemToReg,
    output reg Branch,
    output reg Jump,
    output reg [3:0] ALUControl
);
    always @(*) begin
        // Defaults
        RegWrite = 0; ALUSrc = 0; MemRead = 0; MemWrite = 0;
        MemToReg = 0; Branch = 0; Jump = 0; ALUControl = 4'b0000;
        
        case (opcode)
            // R-Type instructions
            7'b0110011: begin
                RegWrite = 1;
                case (funct3)
                    3'b000: begin
                        if (funct7[5])
                            ALUControl = 4'b0001; // SUB
                        else
                            ALUControl = 4'b0000; // ADD
                    end
                    3'b001: ALUControl = 4'b0101; // SLL
                    3'b101: ALUControl = 4'b0110; // SRL
                    3'b110: ALUControl = 4'b0011; // OR
                    3'b111: ALUControl = 4'b0010; // AND
                    3'b100: ALUControl = 4'b0100; // XOR
                endcase
            end
            
            // I-Type arithmetic
            7'b0010011: begin
                RegWrite = 1; ALUSrc = 1;
                case (funct3)
                    3'b000: ALUControl = 4'b0000; // ADDI
                    3'b111: ALUControl = 4'b0010; // ANDI
                    3'b110: ALUControl = 4'b0011; // ORI
                    3'b100: ALUControl = 4'b0100; // XORI
                    3'b001: ALUControl = 4'b0101; // SLLI
                    3'b101: ALUControl = 4'b0110; // SRLI
                endcase
            end
            
            // Load
            7'b0000011: begin
                RegWrite = 1; ALUSrc = 1; MemRead = 1; MemToReg = 1;
                ALUControl = 4'b0000; // ADD for address calculation
            end
            
            // Store
            7'b0100011: begin
                ALUSrc = 1; MemWrite = 1;
                ALUControl = 4'b0000; // ADD for address calculation
            end
            
            // Branch
            7'b1100011: begin
                Branch = 1;
                ALUControl = 4'b0001; // SUB for comparison
            end
            
            // JAL
            7'b1101111: begin
                RegWrite = 1; Jump = 1;
            end
        endcase
    end
endmodule

// ========== REGISTER FILE WITH INIT ==========
module RegisterFile(
    input         clk,
    input         RegWrite,
    input  [4:0]  rs1, rs2, rd,
    input  [63:0] writeData,
    output [63:0] readData1,
    output [63:0] readData2
);
    reg [63:0] registers [0:31];
    
    // Initialize ALL registers to 0
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 64'b0;
    end
    
    // Read ports - combinational
    assign readData1 = registers[rs1];
    assign readData2 = registers[rs2];
    
    // Write port - synchronous
    always @(posedge clk) begin
        if (RegWrite && (rd != 0)) begin
            registers[rd] <= writeData;
        end
    end
    
    // Debug outputs
    wire [63:0] x0_out, x8_out, x13_out, x14_out, x21_out, x30_out;
    assign x0_out = registers[0];
    assign x8_out = registers[8];
    assign x13_out = registers[13];
    assign x14_out = registers[14];
    assign x21_out = registers[21];
    assign x30_out = registers[30];
endmodule

// ========== IMMEDIATE GENERATOR ==========
module ImmediateGenerator(
    input  [31:0] instruction,
    output reg [63:0] immediate
);
    wire [6:0] opcode = instruction[6:0];
    
    always @(*) begin
        case (opcode)
            // I-type: addi, ld
            7'b0010011, 7'b0000011: 
                immediate = {{52{instruction[31]}}, instruction[31:20]};
            
            // S-type: sd
            7'b0100011: 
                immediate = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]};
            
            // B-type: beq
            7'b1100011: 
                immediate = {{51{instruction[31]}}, instruction[31], 
                            instruction[7], instruction[30:25], 
                            instruction[11:8], 1'b0};
            
            // J-type: jal
            7'b1101111:
                immediate = {{43{instruction[31]}}, instruction[31], 
                            instruction[19:12], instruction[20], 
                            instruction[30:21], 1'b0};
            
            default: immediate = 64'b0;
        endcase
    end
endmodule

// ========== SIMPLE PC MODULE ==========
module PC_Module(
    input         clk,
    input         reset,
    input         Branch,
    input         Jump,
    input         Zero,
    input  [63:0] immediate,
    input  [63:0] PC_current,
    output reg [63:0] PC_next
);
    always @(*) begin
        if (Jump) begin
            // Jump: PC + immediate
            PC_next = PC_current + immediate;
        end else if (Branch && Zero) begin
            // Branch taken: PC + immediate
            PC_next = PC_current + immediate;
        end else begin
            // Normal: PC + 4
            PC_next = PC_current + 64'd4;
        end
    end
endmodule

// ========== INSTRUCTION MEMORY ==========
module InstructionMemory(
    input  [63:0] address,
    output [31:0] instruction
);
    reg [31:0] mem [0:31];
    
    initial begin
        // Initialize program
        mem[0]  = 32'h00700013; // addi x0, x0, 7
        mem[1]  = 32'h07600993; // addi x19, x0, 118
        mem[2]  = 32'h00100593; // addi x11, x0, 1
        mem[3]  = 32'h00b98b33; // add x22, x19, x11
        mem[4]  = 32'h013b60b3; // or x1, x22, x19
        mem[5]  = 32'h00b0df33; // srl x30, x1, x11
        mem[6]  = 32'h41ef0433; // sub x8, x30, x30
        mem[7]  = 32'h01e03423; // sd x30, 8(x0)
        mem[8]  = 32'h00803a83; // ld x21, 8(x0)
        mem[9]  = 32'h001f0693; // addi x13, x30, 1
        mem[10] = 32'h00da8463; // beq x21, x13, 8 (skip 2 instructions)
        mem[11] = 32'h001f0713; // addi x14, x30, 1
        mem[12] = 32'h0080006f; // jal x0, 8 (jump +8 to instruction 14)
        mem[13] = 32'h02e00713; // L1: addi x14, x0, 46
        mem[14] = 32'h00000013; // END: nop
        
        // Initialize rest to NOP
        for (integer j = 15; j < 32; j = j + 1)
            mem[j] = 32'h00000013;
    end
    
    assign instruction = mem[address[6:2]];
endmodule

// ========== DATA MEMORY ==========
module DataMemory(
    input         clk,
    input         MemRead,
    input         MemWrite,
    input  [63:0] address,
    input  [63:0] writeData,
    output [63:0] readData
);
    reg [63:0] mem [0:127];
    
    integer j;
    initial begin
        for (j = 0; j < 128; j = j + 1)
            mem[j] = 64'b0;
    end
    
    // Synchronous read for proper timing
    reg [63:0] readData_reg;
    always @(posedge clk) begin
        if (MemRead)
            readData_reg <= mem[address[9:3]];
    end
    
    assign readData = readData_reg;
    
    // Synchronous write
    always @(posedge clk) begin
        if (MemWrite) begin
            mem[address[9:3]] <= writeData;
        end
    end
    
    wire [63:0] mem8;
    assign mem8 = mem[1];
endmodule

// ========== TOP-LEVEL CPU ==========
module RISCV_CPU(
    input  clk,
    input  reset,
    output [63:0] x0_out,
    output [63:0] x8_out,
    output [63:0] x13_out,
    output [63:0] x14_out,
    output [63:0] x21_out,
    output [63:0] x30_out,
    output [63:0] mem8_out,
    output [63:0] PC_out,
    output [31:0] instr_out
);
    // Pipeline registers
    reg [63:0] PC;
    reg [31:0] IF_ID_instruction;
    reg [63:0] IF_ID_PC;
    
    wire [31:0] instruction;
    wire [63:0] PC_next;
    
    // Control signals
    wire RegWrite, ALUSrc, MemRead, MemWrite, MemToReg, Branch, Jump;
    wire [3:0] ALUControl;
    
    // Register file
    wire [63:0] readData1, readData2, writeData;
    
    // ALU
    wire [63:0] ALU_result;
    wire Zero;
    
    // Immediate
    wire [63:0] immediate;
    
    // Memory
    wire [63:0] mem_read_data;
    
    // PC update
    always @(posedge clk or posedge reset) begin
        if (reset)
            PC <= 64'b0;
        else
            PC <= PC_next;
    end
    
    // Pipeline register update
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            IF_ID_instruction <= 32'h00000013; // NOP
            IF_ID_PC <= 64'b0;
        end else begin
            IF_ID_instruction <= instruction;
            IF_ID_PC <= PC;
        end
    end
    
    // Instruction Memory
    InstructionMemory imem(.address(PC), .instruction(instruction));
    
    // PC Module
    PC_Module pc_mod(.clk(clk), .reset(reset), 
                     .Branch(Branch), .Jump(Jump), .Zero(Zero), 
                     .immediate(immediate), 
                     .PC_current(IF_ID_PC),
                     .PC_next(PC_next));
    
    // Control Unit
    ControlUnit ctrl(.opcode(IF_ID_instruction[6:0]), 
                     .funct3(IF_ID_instruction[14:12]),
                     .funct7(IF_ID_instruction[31:25]),
                     .RegWrite(RegWrite),
                     .ALUSrc(ALUSrc),
                     .MemRead(MemRead),
                     .MemWrite(MemWrite),
                     .MemToReg(MemToReg),
                     .Branch(Branch),
                     .Jump(Jump),
                     .ALUControl(ALUControl));
    
    // Register File
    RegisterFile rf(.clk(clk),
                    .RegWrite(RegWrite),
                    .rs1(IF_ID_instruction[19:15]),
                    .rs2(IF_ID_instruction[24:20]),
                    .rd(IF_ID_instruction[11:7]),
                    .writeData(writeData),
                    .readData1(readData1),
                    .readData2(readData2));
    
    // Immediate Generator
    ImmediateGenerator imm(.instruction(IF_ID_instruction), .immediate(immediate));
    
    // ALU
    wire [63:0] ALU_B = ALUSrc ? immediate : readData2;
    ALU alu(.A(readData1), .B(ALU_B), .ALUControl(ALUControl),
            .Result(ALU_result), .Zero(Zero));
    
    // Data Memory
    DataMemory dmem(.clk(clk),
                    .MemRead(MemRead),
                    .MemWrite(MemWrite),
                    .address(ALU_result),
                    .writeData(readData2),
                    .readData(mem_read_data));
    
    // Write-back MUX
    assign writeData = MemToReg ? mem_read_data : ALU_result;
    
    // Debug outputs
    assign x0_out = rf.x0_out;
    assign x8_out = rf.x8_out;
    assign x13_out = rf.x13_out;
    assign x14_out = rf.x14_out;
    assign x21_out = rf.x21_out;
    assign x30_out = rf.x30_out;
    assign mem8_out = dmem.mem8;
    assign PC_out = PC;
    assign instr_out = IF_ID_instruction;
endmodule


// -----------------------------------------------------
// ========== TESTBENCH ==========
module testbench;
    reg clk, reset;
    
    wire [63:0] x0, x8, x13, x14, x21, x30, mem8, PC;
    wire [31:0] instruction;
    
    RISCV_CPU cpu(.clk(clk), .reset(reset),
                  .x0_out(x0), .x8_out(x8), .x13_out(x13), .x14_out(x14),
                  .x21_out(x21), .x30_out(x30), .mem8_out(mem8),
                  .PC_out(PC), .instr_out(instruction));
    
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    reg [7:0] cycle_count;
    
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, testbench);
        
        cycle_count = 0;
        reset = 1;
        #20 reset = 0;
        
        $display("========================================");
        $display("RISC-V CPU Test Starting...");
        $display("========================================");
        
        // Run for 25 cycles (more time for pipelined operations)
        #250;
        
        $display("\nFINAL RESULTS:");
        $display("x0  = %0d (should be 0)", x0);
        $display("x8  = %0d (should be 0)", x8);
        $display("x13 = %0d (should be 60)", x13);
        $display("x14 = %0d (should be 60)", x14);
        $display("x21 = %0d (should be 59)", x21);
        $display("x30 = %0d (should be 59)", x30);
        $display("mem[8] = %0d (should be 59)", mem8);
        
        if (x0 == 0 && x8 == 0 && x13 == 60 && 
            x14 == 60 && x21 == 59 && x30 == 59 && mem8 == 59) begin
            $display("\nSUCCESS! All tests passed!");
        end else begin
            $display("\nFAILURE! Some tests failed.");
        end
        
        $finish;
    end
    
    always @(posedge clk) begin
        if (!reset) begin
            cycle_count = cycle_count + 1;
            $display("Cycle %0d: PC=%h, Instr=%h, x8=%0d, x21=%0d, x30=%0d, mem[8]=%0d", 
                     cycle_count, PC, instruction, x8, x21, x30, mem8);
        end
    end
endmodule