`timescale 1ns/1ps



module Datamemory(clk, mar, mdr, ldresult, isld, isSt);
    input clk;
    input isld, isSt;
    input [31:0] mar, mdr;
    output reg [31:0] ldresult;

    reg [31:0] memory [0:15];
    wire [3:0] ram_addr = mar[3:0];
 integer i;
    initial begin
   
    for (i = 0; i < 16; i = i + 1)
        memory[i] = $random;
end

    always @(posedge clk) begin
        if (isld) begin
            ldresult <= memory[ram_addr];
        end else if (isSt) begin
            memory[ram_addr] <= mdr;
        end
    end
endmodule



`timescale 1ns/1ps

/* module Datamemory_tb;

    // Inputs
    reg clk;
    reg isld;
    reg isSt;
    reg [31:0] mar;
    reg [31:0] mdr;

    // Outputs
    wire [31:0] ldresult;

    // Instantiate the Unit Under Test (UUT)
    Datamemory uut (
        .clk(clk),
        .mar(mar),
        .mdr(mdr),
        .ldresult(ldresult),
        .isld(isld),
        .isSt(isSt)
    );

    initial begin
        // Initialize Inputs
        clk = 0;
        isld = 0;
        isSt = 0;
        mar = 0;
        mdr = 0;

        // Wait 100 ns for global reset to finish
        #100;

        // Write test cases

        // Test case 1: Write data to memory[0]
       
        // Test case 2: Read data from memory[0]
        isld = 1;
        #10;
        isld = 0;
        $display("Read data from memory[0]: %h", ldresult);

        // Test case 3: Write data to memory[1]
        mar = 32'h1;
        mdr = 32'hCAFEBABE;
        isSt = 1;
        #10;
        isSt = 0;
        $display("Written CAFEBABE to memory[1]");
        
        // Test case 4: Read data from memory[1]
        isld = 1;
        #10;
        isld = 0;
        $display("Read data from memory[1]: %h", ldresult);

        // Add additional test cases as needed

        // End simulation
        #100;
        $finish;
    end

    always begin
        #5 clk = ~clk; // Generate a clock signal with a period of 10 ns
    end

endmodule */
