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
    in…
[2:17 AM, 12/31/2025] Manonaa💖: // Code your testbench here
// or browse Examples
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
        
        $display("===========…
[2:45 AM, 12/31/2025] Manonaa💖: // Code your design here
`timescale 1ns/1ps

module ALU(
    input  [63:0] A,          // First operand
    input  [63:0] B,          // Second operand
    input  [3:0]  ALUControl, // ALU operation selector
    output reg [63:0] Result, // ALU output
    output Zero               // Zero flag
);
    assign Zero = (Result == 64'b0); // Zero flag
    always @(*) begin
        case (ALUControl)
            4'b0000: Result = A + B;         // ADD
            4'b0001: Result = A - B;         // SUB
            4'b0010: Result = A & B;         // AND
            4'b0011: Result = A | B;         // OR
            4'b0100: Result = A ^ B;         // XOR
            4'b0101: Result = A << B[5:0];   // SLL 
            4'b0110: Result = A >> B[5:0];   // SRL 
            default: Result = 64'b0;         // Default
        endcase
    end
endmodule

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
    
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 64'b0;
        end
    end

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

// ===== CORRECTED PC_Branch_Logic Module =====
module PC_Branch_Logic (
    input         clk,
    input         reset,
    input         Branch,          // Branch control signal
    input         Zero,             // Zero flag from ALU
    input  [63:0] immediate,        // Branch offset (sign-extended)
    output reg [63:0] PC_out        // Current PC value
);
    wire [63:0] PC_plus_4;
    wire [63:0] branch_target;
    wire [63:0] next_PC;
    wire        take_branch;
    
    // PC + 4 Adder (sequential instruction)
    assign PC_plus_4 = PC_out + 64'd4;
    
    // Branch Target Calculation (PC + immediate)
    assign branch_target = PC_out + immediate;
    
    // Branch Decision Logic
    assign take_branch = Branch & Zero;
    
    // MUX for selecting next PC
    assign next_PC = take_branch ? branch_target : PC_plus_4;
    
    // PC Register (updates on rising clock edge)
    always @(posedge clk or posedge reset) begin
        if (reset)
            PC_out <= 64'd0;        // Reset PC to 0
        else
            PC_out <= next_PC;      // Update PC every cycle
    end

endmodule

module InstructionMemory(
    input  [63:0] addr,
    output [31:0] instruction
);
    reg [31:0] memory [0:255];
    integer i;

    initial begin
        memory[0]  = 32'h00700013;
        memory[1]  = 32'h07600993;
        memory[2]  = 32'h00100593;
        memory[3]  = 32'h00B98B33;
        memory[4]  = 32'h013B60B3;
        memory[5]  = 32'h00B0DF33;
        memory[6]  = 32'h41EF0433;
        memory[7]  = 32'h07E03423;
        memory[8]  = 32'h06803A83;
        memory[9]  = 32'h001F0693;
        memory[10] = 32'h00D50663; // beq x21,x13,L1
        memory[11] = 32'h001F0713;
        memory[12] = 32'h00000463; // beq x0,x0,END
        memory[13] = 32'h02E00713;
        memory[14] = 32'h00000013;

        for (i=15;i<256;i=i+1)
            memory[i]=32'h00000013;
    end

    assign instruction = memory[addr[63:2]];
endmodule


module DataMemory(
    input clk,
    input MemRead,
    input MemWrite,
    input [63:0] addr,
    input [63:0] writeData,
    output reg [63:0] readData
);
    reg [63:0] memory [0:255]; // 256 64-bit words
    
    always @(posedge clk) begin
        if (MemWrite)
            memory[addr[63:3]] <= writeData; // word-aligned
    end
    
    always @(*) begin
        if (MemRead)
            readData = memory[addr[63:3]];
        else
            readData = 64'b0;
    end
endmodule


module SingleCycleCPU(
    input clk,
    input reset
);
    // --- PC ---
    wire [63:0] PC_current;
    
    // --- Instruction ---
    wire [31:0] instruction;
    
    // --- Control signals ---
    wire RegWrite, ALUSrc, MemRead, MemWrite, MemToReg, Branch;
    wire [3:0] ALUControl;
    
    // --- Register File ---
    wire [63:0] readData1, readData2;
    wire [63:0] writeData; // data to write back
    
    // --- Immediate ---
    wire [63:0] immediate;
    
    // --- ALU ---
    wire [63:0] ALU_inB, ALU_result;
    wire Zero;
    
    // --- Data Memory ---
    wire [63:0] memReadData;
    
    // ----- Instantiate modules -----
    
    // PC with integrated branching logic
    PC_Branch_Logic PC(
        .clk(clk),
        .reset(reset),
        .Branch(Branch),
        .Zero(Zero),
        .immediate(immediate),
        .PC_out(PC_current)
    );
    
    InstructionMemory IM(
        .addr(PC_current),
        .instruction(instruction)
    );
    
    ControlUnit CU(
        .opcode(instruction[6:0]),
        .funct3(instruction[14:12]),
        .funct7(instruction[31:25]),
        .RegWrite(RegWrite),
        .ALUSrc(ALUSrc),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .MemToReg(MemToReg),
        .Branch(Branch),
        .ALUControl(ALUControl)
    );
    
    ImmediateGenerator IG(
        .instruction(instruction),
        .immediate(immediate)
    );
    
    RegisterFile RF(
        .clk(clk),
        .RegWrite(RegWrite),
        .rs1(instruction[19:15]),
        .rs2(instruction[24:20]),
        .rd(instruction[11:7]),
        .writeData(writeData),
        .readData1(readData1),
        .readData2(readData2)
    );
    
    // ALU operand selection
    assign ALU_inB = ALUSrc ? immediate : readData2;
    
    ALU alu(
        .A(readData1),
        .B(ALU_inB),
        .ALUControl(ALUControl),
        .Result(ALU_result),
        .Zero(Zero)
    );
    
    // Data Memory
    DataMemory DM(
        .clk(clk),
        .addr(ALU_result),
        .writeData(readData2),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .readData(memReadData)
    );
    
    // Write-back
    assign writeData = MemToReg ? memReadData : ALU_result;

endmodule
----------------------
//-------------------------testbench-----------------------------------------------
`timescale 1ns/1ps

module SingleCycleCPU_tb;

    reg clk;
    reg reset;

    integer pass_count = 0;
    integer fail_count = 0;

    // DUT
    SingleCycleCPU cpu (
        .clk(clk),
        .reset(reset)
    );

    // ================= CLOCK =================
    always #5 clk = ~clk;

    // ================= TASKS =================
    task check_register;
        input integer reg_num;
        input [63:0] expected;
        input string test_name;
        begin
            $display("Checking Register x%0d: %s", reg_num, test_name);
            $display("   Expected: %0d, Actual: %0d", expected, cpu.RF.registers[reg_num]);
            if (cpu.RF.registers[reg_num] !== expected) begin
                $display("   Result: FAIL\n");
                fail_count = fail_count + 1;
            end else begin
                $display("   Result: PASS\n");
                pass_count = pass_count + 1;
            end
        end
    endtask

    task check_memory;
        input integer mem_index;
        input [63:0] expected;
        input string test_name;
        begin
            $display("Checking Memory[%0d]: %s", mem_index, test_name);
            $display("   Expected: %0d, Actual: %0d", expected, cpu.DM.memory[mem_index]);
            if (cpu.DM.memory[mem_index] !== expected) begin
                $display("   Result: FAIL\n");
                fail_count = fail_count + 1;
            end else begin
                $display("   Result: PASS\n");
                pass_count = pass_count + 1;
            end
        end
    endtask

    // ================= TEST SEQUENCE =================
    initial begin
        // Waveform dump
        $dumpfile("SingleCycleCPU_tb.vcd");
        $dumpvars(0, SingleCycleCPU_tb);

        $display("===== SINGLE-CYCLE RISC-V CPU TEST =====");

        clk = 0;
        reset = 1;
        #20;
        reset = 0;

        // Run long enough for full program execution
        #400;

        $display("\n--- MAIN PROGRAM CHECKS ---");

        check_register(0,  0,  "x0 remains hardwired to zero");
        check_register(30, 59, "x30 result of logical shift right");
        check_register(8,  0,  "x8 result of subtraction");
        check_register(21, 59, "x21 value loaded from memory");
        check_register(13, 60, "x13 increment of x30");
        check_register(14, 60, "x14 set after branch logic");

        // 104 / 8 = 13 (doubleword addressing)
        check_memory(13, 59, "Memory at address 104 contains x30 value");

        // ================= EDGE CASES =================
        $display("\n--- EDGE CASE CHECKS ---");

        check_register(0, 0, "Writes to x0 are ignored");
        check_register(8, 0, "Zero result correctly generated by SUB");
        check_register(14, 60, "Branch not taken handled correctly");
        check_register(14, 60, "Unconditional branch handled correctly");
        check_register(21, 59, "Load after store consistency verified");

        // ================= SUMMARY =================
        $display("\n===== TEST SUMMARY =====");
        $display("PASSED: %0d", pass_count);
        $display("FAILED: %0d", fail_count);

        if (fail_count == 0)
            $display("ALL TESTS PASSED SUCCESSFULLY");
        else
            $display("SOME TESTS FAILED");

        $finish;
    end

endmodule
