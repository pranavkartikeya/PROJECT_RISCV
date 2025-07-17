module RW(MA_RW_aluresult, Ldresult, MA_RW_inst, MA_RW_pc, clk, isCall, isld,muxaddr,muxdata,isWb);
    input clk, isCall, isld,isWb;
    input [31:0] MA_RW_aluresult, Ldresult, MA_RW_inst, MA_RW_pc;
    reg [3:0] temp_muxaddr; // Temporary register for delay
    output reg [3:0] muxaddr;
    output reg [31:0] muxdata;

    //RegFile1 Z(.clk(clk), .regwrite_en(isWb), .regwrite_dest(muxaddr), .regwrite_data(muxdata));

    always @(posedge clk) begin
        // Pipeline stage: assign to temp_muxaddr first
        if (isCall) 
            temp_muxaddr <= 4'b1111;
        else 
            temp_muxaddr <= MA_RW_inst[25:22];

        // Delayed assignment to muxaddr
        muxaddr <= temp_muxaddr;

        if (~isld && ~isCall) 
            muxdata <= MA_RW_aluresult;
        else if (isld && ~isCall) 
            muxdata <= Ldresult;
        else if (~isld && isCall) 
            muxdata <= MA_RW_pc + 4;
    end

endmodule


/*`timescale 1ns / 1ps

module RW_tb;
    // Inputs
    reg clk;
    reg isCall;
    reg isld;
    reg regwrite_en;
    reg [31:0] MA_RW_aluresult;
    reg [31:0] Ldresult;
    reg [31:0] MA_RW_inst;
    reg [31:0] MA_RW_pc; // Corrected the bit width to 32

    // Instantiate the Unit Under Test (UUT)
    RW uut (
        .clk(clk), 
        .isCall(isCall), 
        .isld(isld), 
        .regwrite_en(regwrite_en), 
        .MA_RW_aluresult(MA_RW_aluresult), 
        .Ldresult(Ldresult), 
        .MA_RW_inst(MA_RW_inst), 
        .MA_RW_pc(MA_RW_pc)
    );

    initial begin
        // Initialize Inputs
        clk = 0;
        isCall = 0;
        isld = 0;
        regwrite_en = 0;
        MA_RW_aluresult = 0;
        Ldresult = 0;
        MA_RW_inst = 0;
        MA_RW_pc = 0;

        // Wait for global reset

        // Test Case 1: Normal ALU result write
        MA_RW_aluresult = 32'hA5A5A5A5;
        MA_RW_inst = 32'h00000000;
        regwrite_en = 1;
        #10;
        regwrite_en = 0;
        #10;

        // Test Case 2: Load result write
        isld = 1;
        Ldresult = 32'h5A5A5A5A;
        regwrite_en = 1;
        #10;
        regwrite_en = 0;
        isld = 0;
        #10;

        // Test Case 3: Call instruction
        isCall = 1;
        MA_RW_pc = 32'h00000010;
        regwrite_en = 1;
        #10;
        regwrite_en = 0;
        isCall = 0;
        #10;

        // Add more test cases as needed

        // Finish simulation
        $finish;
    end

    always #5 clk = ~clk; // Clock generation

    always @(posedge clk) begin
        $display("Time: %0t | regwrite_en: %b | muxaddr: %b | muxdata: %h", $time, regwrite_en, uut.muxaddr, uut.muxdata);
        $display("Registers:");
        for (integer i = 0; i < 16; i = i + 1) begin
            $display("REG[%0d] = %h", i, uut.p9.REG[i]);
        end
    end

endmodule */
