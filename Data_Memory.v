`timescale 1ns/1ps
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
------------------------------
//testbench

module DataMemory_TB;

    reg clk;
    reg MemRead, MemWrite;
    reg [63:0] addr, writeData;
    wire [63:0] readData;

    // Instantiate DataMemory
    DataMemory DM (
        .clk(clk),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .addr(addr),
        .writeData(writeData),
        .readData(readData)
    );

    // Clock generation: 10ns period
    initial clk = 0;
    always #5 clk = ~clk;

    // Waveform dump
    initial begin
        $dumpfile("DataMemory_TB.vcd");
        $dumpvars(0, DataMemory_TB);
    end

    initial begin
        $display("Time\tclk\tMemRead\tMemWrite\tAddr\t\tWriteData\tReadData");

        
        MemRead = 0; MemWrite = 0; addr = 0; writeData = 0;
        #10;

    
        addr = 64'd0; writeData = 64'hA5A5A5A5A5A5A5A5; MemWrite = 1;
        @(posedge clk); #1;  // wait for write to occur
        MemWrite = 0;

        /
        MemRead = 1; addr = 64'd0; #1;
        $display("%0t\t%b\t%b\t%b\t%h\t%h\t%h", $time, clk, MemRead, MemWrite, addr, writeData, readData);

       
        addr = 64'd8; writeData = 64'h123456789ABCDEF0; MemWrite = 1;
        @(posedge clk); #1;
        MemWrite = 0;

      
        addr = 64'd8; #1;
        $display("%0t\t%b\t%b\t%b\t%h\t%h\t%h", $time, clk, MemRead, MemWrite, addr, writeData, readData);

       
        addr = 64'd10; #1;
        $display("%0t\t%b\t%b\t%b\t%h\t%h\t%h", $time, clk, MemRead, MemWrite, addr, writeData, readData);

     
        MemRead = 0; addr = 64'd0; #1;
        $display("%0t\t%b\t%b\t%b\t%h\t%h\t%h", $time, clk, MemRead, MemWrite, addr, writeData, readData);

        
        addr = 64'd8; writeData = 64'hFFFFFFFFFFFFFFFF; MemWrite = 1;
        @(posedge clk); #1;
        MemWrite = 0;

       
        MemRead = 1; addr = 64'd8; #1;
        $display("%0t\t%b\t%b\t%b\t%h\t%h\t%h", $time, clk, MemRead, MemWrite, addr, writeData, readData);

        $finish;
    end

endmodule
