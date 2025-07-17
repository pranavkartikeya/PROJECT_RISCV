`include "Datamemory.v"
module MA(aluresult,op2,clk,inst,pc,MA_RW_aluresult,Ldresult,MA_RW_inst,MA_RW_pc,isld,isSt,isCall,isWb,isCall_MR,isWb_MR,isLd_MR);

       input [31:0]aluresult,op2,inst,pc;
       input isld,clk,isSt,isCall,isWb;
       output reg [31:0]MA_RW_aluresult,Ldresult,MA_RW_inst,MA_RW_pc;
       output reg isCall_MR,isWb_MR,isLd_MR;
       wire [31:0] ldresult;
      
       Datamemory  n(.clk(clk),.mar(aluresult),.mdr(op2),.ldresult(ldresult),.isld(isld),.isSt(isSt));

       always @(posedge clk)
              begin
              MA_RW_aluresult <= aluresult;
              MA_RW_inst <= inst;
              MA_RW_pc <= pc;
              Ldresult <= ldresult;
              end

         always @(*)
              begin
              isCall_MR<= isCall;
              isWb_MR <= isWb;
              isLd_MR <= isld;
              end
          always @(posedge clk)
              begin
              Ldresult <= ldresult;
              end
        


endmodule

  /* module MA_tb;

    // Inputs
    reg [31:0] aluresult, op2, inst, pc;
    reg isld, isSt, clk;

    // Outputs
    wire [31:0] MA_RW_aluresult, ldresult, MA_RW_inst, MA_RW_pc;

    // Instantiate the Unit Under Test (UUT)
    MA uut (
        .aluresult(aluresult), 
        .op2(op2), 
        .clk(clk), 
        .inst(inst), 
        .pc(pc), 
        .MA_RW_aluresult(MA_RW_aluresult), 
        .Ldresult(ldresult), 
        .MA_RW_inst(MA_RW_inst), 
        .MA_RW_pc(MA_RW_pc), 
        .isld(isld), 
        .isSt(isSt)
    );

    // Memory Display Task
    task display_memory;
        integer i;
        begin
            $display("time = %d", $time);
            for (i = 0; i < 16; i = i + 1) begin
                $display("\tmemory[%0d] = %h", i, uut.n.memory[i]);
            end
        end
    endtask

    initial begin
        // Initialize Inputs
        aluresult = 0;
        op2 = 0;
        inst = 0;
        pc = 0;
        isld = 0;
        isSt = 0;
        clk = 0;

        // Wait 100 ns for global reset to finish
      

        // Test Case 1: Write data to memory[0]
        aluresult = 0;
        op2 = 32'hDEADBEEF;
        isSt = 1;
        #10;
        isSt = 0;
        display_memory;

        // Test Case 2: Read data from memory[0]
        isld = 1;
        #20;
        isld = 0;
        $display("Read data from memory[0]: %h", ldresult);
        display_memory;

        // Test Case 3: Write data to memory[1]
        aluresult = 1;
        op2 = 32'hCAFEBABE;
        isSt = 1;
        #10;
        isSt = 0;
        display_memory;

        // Test Case 4: Read data from memory[1]
        isld = 1;
        #20;
        isld = 0;
        $display("Read data from memory[1]: %h", ldresult);
        display_memory;

        // Add additional test cases as needed

        // End simulation
        #100;
        $finish;
    end

    always begin
        #5 clk = ~clk; // Generate a clock signal with a period of 10 ns
    end
    initial begin
        $dumpfile("tb_MA.vcd"); // Specify the dump file name
        $dumpvars(0,MA_tb);    // Dump all variables in tb_OF
    end

endmodule  */
          



