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

//-----------------------------------------

************************************ TestBench ************************************

module ALU_Task1_tb;

    reg  [63:0] A, B;
    reg  [3:0]  ALUControl;
    wire [63:0] Result;
    wire Zero;

    // Instantiate the ALU
    ALU alu_inst (
        .A(A),
        .B(B),
        .ALUControl(ALUControl),
        .Result(Result),
        .Zero(Zero)
    );

    initial begin
        // Test ADD
        A = 10; B = 5; ALUControl = 4'b0000;
        #10; $display("ADD: %d + %d = %d", A, B, Result);

        // Test SUB
        A = 10; B = 5; ALUControl = 4'b0001;
        #10; $display("SUB: %d - %d = %d", A, B, Result);

        // Test AND
        A = 12; B = 10; ALUControl = 4'b0010;
        #10; $display("AND: %d & %d = %d", A, B, Result);

        // Test OR
        A = 12; B = 10; ALUControl = 4'b0011;
        #10; $display("OR: %d | %d = %d", A, B, Result);

        // Test XOR
        A = 12; B = 10; ALUControl = 4'b0100;
        #10; $display("XOR: %d ^ %d = %d", A, B, Result);

        // Test SLL
        A = 3; B = 2; ALUControl = 4'b0101;
        #10; $display("SLL: %d << %d = %d", A, B, Result);

        // Test SRL
        A = 8; B = 2; ALUControl = 4'b0110;
        #10; $display("SRL: %d >> %d = %d", A, B, Result);

        $finish;
    end

endmodule
