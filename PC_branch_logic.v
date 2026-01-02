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
    // Note: For RISC-V, immediate is already shifted in decode stage
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


#==========================================================================
#==========================================================================


module PC_Branch_Logic_tb;
    reg         clk;
    reg         reset;
    reg         Branch;
    reg         Zero;
    reg  [63:0] immediate;
    wire [63:0] PC_out;
    
    // Instantiate the PC_Branch_Logic module
    PC_Branch_Logic uut (
        .clk(clk),
        .reset(reset),
        .Branch(Branch),
        .Zero(Zero),
        .immediate(immediate),
        .PC_out(PC_out)
    );
    
    // Clock generation (10ns period)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Test sequence
    initial begin
        $dumpfile("pc_branch_logic.vcd");
        $dumpvars(0, PC_Branch_Logic_tb);
        
        $display("=== PC Branch Logic Testbench ===\n");
        
        // Initialize signals
        reset = 1;
        Branch = 0;
        Zero = 0;
        immediate = 64'd0;
        
        // Test 1: Reset behavior
        #10;
        reset = 0;
        $display("Test 1: Reset");
        $display("PC after reset: %d (Expected: 0)", PC_out);
        if (PC_out !== 64'd0) $display("ERROR: Reset failed!\n");
        else $display("PASS\n");
        
        // Test 2: Sequential PC increment (PC + 4)
        #10;
        $display("Test 2: Sequential PC increment");
        $display("PC = %d (Expected: 4)", PC_out);
        if (PC_out !== 64'd4) $display("ERROR: PC+4 failed!\n");
        else $display("PASS\n");
        
        #10;
        $display("PC = %d (Expected: 8)", PC_out);
        if (PC_out !== 64'd8) $display("ERROR: PC+4 failed!\n");
        else $display("PASS\n");
        
        #10;
        $display("PC = %d (Expected: 12)", PC_out);
        if (PC_out !== 64'd12) $display("ERROR: PC+4 failed!\n");
        else $display("PASS\n");
        
        // Test 3: Branch NOT taken (Branch=1, Zero=0)
        Branch = 1;
        Zero = 0;
        immediate = 64'd100;
        #10;
        $display("Test 3: Branch NOT taken (Branch=1, Zero=0)");
        $display("PC = %d (Expected: 16)", PC_out);
        if (PC_out !== 64'd16) $display("ERROR: Should continue with PC+4!\n");
        else $display("PASS\n");
        
        // Test 4: Branch NOT taken (Branch=0, Zero=1)
        Branch = 0;
        Zero = 1;
        immediate = 64'd100;
        #10;
        $display("Test 4: Branch NOT taken (Branch=0, Zero=1)");
        $display("PC = %d (Expected: 20)", PC_out);
        if (PC_out !== 64'd20) $display("ERROR: Should continue with PC+4!\n");
        else $display("PASS\n");
        
        // Test 5: Branch TAKEN (Branch=1, Zero=1) - Forward branch
        Branch = 1;
        Zero = 1;
        immediate = 64'd32;  // Branch forward by 32 bytes
        #10;
        $display("Test 5: Branch TAKEN - Forward branch");
        $display("PC = %d (Expected: 52, which is 20+32)", PC_out);
        if (PC_out !== 64'd52) $display("ERROR: Branch taken failed!\n");
        else $display("PASS\n");
        
        // Test 6: Continue after branch
        Branch = 0;
        Zero = 0;
        #10;
        $display("Test 6: Sequential after branch");
        $display("PC = %d (Expected: 56)", PC_out);
        if (PC_out !== 64'd56) $display("ERROR: PC+4 after branch failed!\n");
        else $display("PASS\n");
        
        // Test 7: Backward branch (negative offset)
        Branch = 1;
        Zero = 1;
        immediate = -64'd40;  // Branch backward
        #10;
        $display("Test 7: Branch TAKEN - Backward branch");
        $display("PC = %d (Expected: 16, which is 56-40)", PC_out);
        if (PC_out !== 64'd16) $display("ERROR: Backward branch failed!\n");
        else $display("PASS\n");
        
        // Test 8: Large forward branch
        Branch = 1;
        Zero = 1;
        immediate = 64'd1000;
        #10;
        $display("Test 8: Large forward branch");
        $display("PC = %d (Expected: 1016, which is 16+1000)", PC_out);
        if (PC_out !== 64'd1016) $display("ERROR: Large branch failed!\n");
        else $display("PASS\n");
        
        // Test 9: Reset during operation
        reset = 1;
        #10;
        reset = 0;
        $display("Test 9: Reset during operation");
        $display("PC = %d (Expected: 0)", PC_out);
        if (PC_out !== 64'd0) $display("ERROR: Reset during operation failed!\n");
        else $display("PASS\n");
        
        // Test 10: Multiple sequential increments
        Branch = 0;
        Zero = 0;
        repeat(5) begin
            #10;
        end
        $display("Test 10: Multiple sequential increments");
        $display("PC = %d (Expected: 20)", PC_out);
        if (PC_out !== 64'd20) $display("ERROR: Multiple increments failed!\n");
        else $display("PASS\n");
        
        $display("\n=== All tests completed ===");
        #10;
        $finish;
    end
    
    // Monitor PC changes
    initial begin
        $monitor("Time=%0t | Branch=%b Zero=%b Imm=%d | PC=%d", 
                 $time, Branch, Zero, immediate, PC_out);
    end

endmodule