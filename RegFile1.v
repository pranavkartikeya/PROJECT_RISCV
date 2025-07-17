module RegFile1 (clk,regwrite_en,regread_dest1,regread_dest2,regwrite_dest,regwrite_data,A,B,OP2,isImmediate,immx);
       input clk,regwrite_en;
       input [3:0] regread_dest1,regread_dest2,regwrite_dest;
       input [31:0] regwrite_data;
       input isImmediate;
       input [31:0] immx;
       output reg[31:0] A,B,OP2;
       reg [31:0] REG [0:15];
       

         
       
       initial begin
          REG[0] = 32'b00000000000000000000000000000001;
          REG[1] = 32'b00000000000000000000000000000010;
          REG[2] = 32'b00000000000000000000000000000011;
          REG[3] = 32'b00000000000000000000000000000111;
          REG[4] = 32'b00000000000000000000000000000110;
          REG[5] = 32'b00000000000000000000000000000100;
          REG[15]= 32'b01100011001100110011001100110011;
    // Initialize other registers as needed.
    end

      always @(posedge clk)
        begin
        if(regwrite_en) begin
            REG[regwrite_dest] <= regwrite_data;
            end
       end

       
       always @(*)
       begin
        A = REG[regread_dest1];
        OP2 = REG[regread_dest2];
        end

     always @(*) begin
        if (isImmediate)
            B = immx;
        else
            B = OP2;
    end
  endmodule

 /*module tb();
       reg clk;
       reg [3:0] regread_dest1,regread_dest2;
       wire [31:0] regread_data1,regread_data2;
        wire [31:0] regwrite_data;
        wire [3:0] regwrite_dest;

       
       RegFile1 Z(
            .clk(clk),
            .regread_dest1(regread_dest1),
            .regread_dest2(regread_dest2),
            .regread_data1(regread_data1),
            .regread_data2(regread_data2),
            .regwrite_en(isWb_MR),
            .regwrite_dest(regwrite_dest), 
            .regwrite_data(regwrite_data)
        ); 


       initial begin
        clk = 0;
        forever #5 clk = ~clk;   
             // Toggle clock every 5 time units
        end

        initial
        begin
        
        regread_dest1 = 4'b0;
        regread_dest2 = 4'b0;
       
       #5
      
        regread_dest1 = 4'b0000;
        #20;
        $finish;
       end

       initial begin

         $monitor("Time: %0t | regread_data1: %b", $time,regread_data1);
         end
      endmodule  */

          

        
       



                    