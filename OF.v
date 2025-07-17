`include "controlpath.v"


module OF (
    input clk,
    input [31:0] inst, 
    input [31:0] pc,
    output reg  [31:0] OP2,
    output reg [31:0] OF_EX_PC,
    output reg [31:0] OF_EX_INST,
    output reg [31:0] branchtarget,
    output reg isSt_OE,
    output reg isLd_OE,
    output reg isBeq_OE,
    output reg isBgt_OE,
    output reg isRet_OE,
    output reg isImmediate_OE,
    output reg isWb_OE,
    output reg isUbranch_OE,
    output reg isCall_OE,
    output reg isAdd_OE,
    output reg isSub_OE,
    output reg isCmp_OE,
    output reg isMul_OE,
    output reg isDiv_OE,
    output reg isMod_OE,
    output reg isLsl_OE,
    output reg isLsr_OE,
    output reg isAsr_OE,
    output reg isOr_OE,
    output reg isAnd_OE,
    output reg isNot_OE,
    output reg isMov_OE,
    output reg [3:0] mux1out, mux2out,
    output reg [31:0] immx
    
);
 wire isSt, isLd, isBeq, isBgt, isRet, isImmediate, isWb, isUbranch, isCall;
 wire isAdd, isSub, isCmp, isMul, isDiv, isMod, isLsl, isLsr, isAsr;
 wire  isOr, isAnd, isNot, isMov;
    
    
    

    // RegFile1 instance

    
     Controlpath control_path (
    .inst(inst),
    .clk(clk),
    .isAdd(isAdd),.isSub(isSub), .isMul(isMul), .isDiv(isDiv), .isMod(isMod),
    .isAnd(isAnd), .isOr(isOr), .isNot(isNot), .isMov(isMov),
    .isLsl(isLsl), .isLsr(isLsr), .isAsr(isAsr),
    .isLd(isLd), .isSt(isSt), .isBeq(isBeq), .isBgt(isBgt),
    .isUbranch(isUbranch), .isCall(isCall), .isRet(isRet),
    .isImmediate(isImmediate), .isCmp(isCmp), .isWb(isWb)
);

    /*RegFile1 Z(
        .clk(clk),
        .regread_dest1(mux1out),
        .regread_dest2(mux2out),
        .regread_data1(A1),
        .regread_data2(OP21)
    );*/

    // Sign or zero extension
    always @(posedge clk) begin
        case (inst[17:16])
            2'b00: immx = {{16{inst[15]}}, inst[15:0]}; // Sign-extend
            2'b01: immx = {16'b0, inst[15:0]};          // Zero-extend
            default: immx = 32'b0;                      // Default case
        endcase
    end

    // Branch target calculation and `B` selection
    always @(posedge clk) begin
        branchtarget = { {3{inst[26]}}, inst[26:0], 2'b0 } + pc;
    end

    // Multiplexer logic
    always @(posedge clk) begin
        if (isRet)
            mux1out = 4'b1111; // Select REG[15]
        else
            mux1out = inst[21:18];
    end

    always @(posedge clk) begin
        if (isSt)
            mux2out = inst[25:22];
        else
            mux2out = inst[17:14];
    end

    // Combinational B and OP2 assignment
   /* always @(OP21) begin
        if (isImmediate)
            B = immx;
        else
            B = OP21;
    end
    always @(*) begin
         A <= A1;
         OP2 <= OP21;

      end */
  
     always @(posedge clk) begin
     OF_EX_PC <= pc;
     
     OF_EX_INST <= inst;
     end

    always @(*) begin
    isAdd_OE = isAdd;
    isSub_OE = isSub;
    isMul_OE = isMul;
    isDiv_OE = isDiv;
    isMod_OE = isMod;
    isAnd_OE = isAnd;
    isOr_OE = isOr;
    isNot_OE = isNot;
    isMov_OE = isMov;
    isLsl_OE = isLsl;
    isLsr_OE = isLsr;
    isAsr_OE = isAsr;
    isLd_OE = isLd;
    isSt_OE = isSt;
    isBeq_OE = isBeq;
    isBgt_OE = isBgt;
    isUbranch_OE = isUbranch;
    isCall_OE = isCall;
    isRet_OE = isRet;
    isImmediate_OE = isImmediate;
    isCmp_OE = isCmp;
    isWb_OE = isWb;
end

    
endmodule



/*module OF_tb;

    // Inputs
    reg clk;
    reg [31:0] inst;
    reg [31:0] pc;

    // Outputs
    wire [31:0] A;
    wire [31:0] B;
    wire [31:0] OP2;
    wire [31:0] OF_EX_PC;
    wire [31:0] OF_EX_INST;
    wire [31:0] branchtarget;
    wire isSt_OE, isLd_OE, isBeq_OE, isBgt_OE, isRet_OE, isImmediate_OE;
    wire isWb_OE, isUbranch_OE, isCall_OE, isAdd_OE, isSub_OE, isCmp_OE;
    wire isMul_OE, isDiv_OE, isMod_OE, isLsl_OE, isLsr_OE, isAsr_OE;
    wire isOr_OE, isAnd_OE, isNot_OE, isMov_OE;

    // Instantiate the OF module
    OF uut (
        .clk(clk), 
        .inst(inst), 
        .pc(pc), 
        .A(A), 
        .B(B), 
        .OP2(OP2), 
        .OF_EX_PC(OF_EX_PC), 
        .OF_EX_INST(OF_EX_INST), 
        .branchtarget(branchtarget), 
        .isSt_OE(isSt_OE), 
        .isLd_OE(isLd_OE), 
        .isBeq_OE(isBeq_OE), 
        .isBgt_OE(isBgt_OE), 
        .isRet_OE(isRet_OE), 
        .isImmediate_OE(isImmediate_OE), 
        .isWb_OE(isWb_OE), 
        .isUbranch_OE(isUbranch_OE), 
        .isCall_OE(isCall_OE), 
        .isAdd_OE(isAdd_OE), 
        .isSub_OE(isSub_OE), 
        .isCmp_OE(isCmp_OE), 
        .isMul_OE(isMul_OE), 
        .isDiv_OE(isDiv_OE), 
        .isMod_OE(isMod_OE), 
        .isLsl_OE(isLsl_OE), 
        .isLsr_OE(isLsr_OE), 
        .isAsr_OE(isAsr_OE), 
        .isOr_OE(isOr_OE), 
        .isAnd_OE(isAnd_OE), 
        .isNot_OE(isNot_OE), 
        .isMov_OE(isMov_OE)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

    // Test stimulus
    initial begin
        // Initialize inputs
        inst = 32'b0;
        pc = 32'b0;

        // Test case 1: Add instruction
        #5;
        pc = 32'h00000010;
        inst = 32'b00000_0_0100_0001_0010_00000000000000; // add R4 = R1 + R2
       

        #10; // Wait for the result

        // Test case 2: Immediate operation
        inst = 32'b00001_1_0110_0011_01_0000000000000011; // imm R6 = R3 + 3
       

        #20; // Wait for the result

        $finish;
    end

    // Dump waveforms for GTKWave
    initial begin
        $dumpfile("OF_tb.vcd");
        $dumpvars(0, OF_tb);
    end

endmodule
*/






    