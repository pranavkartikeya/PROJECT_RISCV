`include "instructionmemory.v"
`timescale 1ns/1ps

module IF(isBranchtaken,BranchPC,IF_OF_PC,IF_OF_INST,clk);
       input isBranchtaken;
       input [31:0]BranchPC;
       input clk;
       output reg [31:0] IF_OF_PC;
       output reg [31:0] IF_OF_INST;
       wire [31:0] NPC;
       wire [31:0] instruction;

       Instruction_Memory p2(NPC,instruction);

        initial 
       begin
       IF_OF_PC = 32'b0;
       end
       
       always @(posedge clk)
       begin
       if(isBranchtaken) IF_OF_PC <= BranchPC;
       else IF_OF_PC <= IF_OF_PC + 1;
       end

       
       assign NPC = IF_OF_PC;
       

       always @(IF_OF_PC)
       begin

       IF_OF_INST <= instruction;

       end


endmodule 

/*module tb();
       
       reg isBranchtaken;
       reg [31:0]BranchPC;
       reg clk;
       wire [31:0] IF_OF_PC;
       wire [31:0] IF_OF_INST;

       IF  v4(isBranchtaken,BranchPC,IF_OF_PC,IF_OF_INST,clk);
       
       initial begin
        clk = 0;
        forever #5 clk = ~clk;          // Toggle clock every 5 time units
    end

       initial 
       begin
       
       isBranchtaken = 0;
       #10 isBranchtaken = 0;

       $finish;
       end

       initial begin
        $monitor("Time: %0t | instruction: %b", $time,IF_OF_INST);
        end
endmodule */







       


       