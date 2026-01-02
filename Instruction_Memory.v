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
//-----------------------------
//testbench
module InstructionMemory_TB;

    reg  [63:0] addr;
    wire [31:0] instruction;

    // Instantiate InstructionMemory
    InstructionMemory IM (
        .addr(addr),
        .instruction(instruction)
    );

    // Waveform dump
    initial begin
        $dumpfile("InstructionMemory_TB.vcd");
        $dumpvars(0, InstructionMemory_TB);
    end

    initial begin
        $display("Time\tAddr\tInstruction");

        // Edge case 1: PC = 0
        addr = 64'd0; #5; $display("%0t\t%0d\t%h", $time, addr, instruction);

        // Sequential instructions
        addr = 64'd4; #5; $display("%0t\t%0d\t%h", $time, addr, instruction);
        addr = 64'd8; #5; $display("%0t\t%0d\t%h", $time, addr, instruction);
        addr = 64'd12; #5; $display("%0t\t%0d\t%h", $time, addr, instruction);

        // Branch instruction
        addr = 64'd40; #5; $display("%0t\t%0d\t%h", $time, addr, instruction);

        // End program
        addr = 64'd56; #5; $display("%0t\t%0d\t%h", $time, addr, instruction);

        // Unaligned address (PC = 2) → should floor to memory[0]
        addr = 64'd2; #5; $display("%0t\t%0d\t%h", $time, addr, instruction);

        // Large address (beyond program) → NOP
        addr = 64'd1020; #5; $display("%0t\t%0d\t%h", $time, addr, instruction);

        $finish;
    end

endmodule