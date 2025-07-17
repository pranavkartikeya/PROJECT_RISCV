`include "ALU.v"

module EX(
    input [31:0] A, B, OP2, INST, PC, branchtarget,
    input clk,
    input isBeq, isBgt,isUBranch, isRet,isWb,
    input isadd, isld, isst, issub, iscmp, ismul, islsl, islsr, isasr, isor, isnot, isand, isdiv, ismod, ismov,iscall,
    output reg [31:0] ALURESULT, EX_MA_PC, EX_MA_OP2, EX_MA_INST,
    output isBranchtaken,
    output reg [31:0] branchpc,
    output reg zero_flag, // Zero flag
    output reg overflow_flag, 
    output reg flag_gt,
    output reg isSt_EM,
    output reg isLd_EM,
    output reg isWb_EM,
    output reg isCall_EM

);
    wire [31:0] aluresult;
    wire flagE,Oflag,flagG;

always @(posedge clk) begin
    if (isRet) 
        branchpc <= A;
    else 
        branchpc <= branchtarget;
end

ALU uut (
    .a(A),
    .b(B),
    .isadd(isadd),
    .isld(isld),
    .isst(isst),
    .issub(issub),
    .iscmp(iscmp),
    .ismul(ismul),
    .islsl(islsl),
    .islsr(islsr),
    .isasr(isasr),
    .isor(isor),
    .isnot(isnot),
    .isand(isand),
    .isdiv(isdiv),
    .ismod(ismod),
    .ismov(ismov),
    .clk(clk),
    .alu_result(aluresult),
    .zero_flag(flagE),
    .overflow_flag(Oflag),
    .flag_gt(flagG)
);

assign isBranchtaken = isUBranch | (isBeq & zero_flag) | (isBgt & flag_gt);

always @(*) begin
    EX_MA_PC <= PC;
    EX_MA_OP2 <= OP2;  
    ALURESULT <= aluresult;
   zero_flag  <= flagE;
   overflow_flag <= Oflag;
     flag_gt <= flagG;
end
always @(*)
       begin
       EX_MA_INST <= INST;
       isSt_EM <= isst;
       isLd_EM  <= isld;
       isWb_EM  <= isWb;
       isCall_EM <= iscall;
       end

endmodule

/*module EX_tb;
    // Testbench variables
    reg [31:0] A, B, OP2, INST, PC, branchtarget;
    reg clk;
    reg isBeq, isBgt, isUBranch, isRet;
    reg isadd, isld, isst, issub, iscmp, ismul, islsl, islsr, isasr, isor, isnot, isand, isdiv, ismod, ismov;
    wire [31:0] ALURESULT, EX_MA_PC, EX_MA_OP2, EX_MA_INST;
    wire isBranchtaken;
    wire [31:0] branchpc;
    wire zero_flag, overflow_flag, flag_gt;

    // Instantiate the EX module
    EX uut (
        .A(A),
        .B(B),
        .OP2(OP2),
        .INST(INST),
        .PC(PC),
        .branchtarget(branchtarget),
        .clk(clk),
        .isBeq(isBeq),
        .isBgt(isBgt),
        .isUBranch(isUBranch),
        .isRet(isRet),
        .isadd(isadd),
        .isld(isld),
        .isst(isst),
        .issub(issub),
        .iscmp(iscmp),
        .ismul(ismul),
        .islsl(islsl),
        .islsr(islsr),
        .isasr(isasr),
        .isor(isor),
        .isnot(isnot),
        .isand(isand),
        .isdiv(isdiv),
        .ismod(ismod),
        .ismov(ismov),
        .ALURESULT(ALURESULT),
        .EX_MA_PC(EX_MA_PC),
        .EX_MA_OP2(EX_MA_OP2),
        .EX_MA_INST(EX_MA_INST),
        .isBranchtaken(isBranchtaken),
        .branchpc(branchpc),
        .zero_flag(zero_flag),
        .overflow_flag(overflow_flag),
        .flag_gt(flag_gt)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10 time units clock period
    end

    // Test vectors
    initial begin
        // Initialize inputs
        A = 32'h0;
        B = 32'h0;
        OP2 = 32'h0;
        INST = 32'h0;
        PC = 32'h0;
        branchtarget = 32'h0;
        isBeq = 0;
        isBgt = 0;
        isUBranch = 0;
        isRet = 0;
        isadd = 0;
        isld = 0;
        isst = 0;
        issub = 0;
        iscmp = 0;
        ismul = 0;
        islsl = 0;
        islsr = 0;
        isasr = 0;
        isor = 0;
        isnot = 0;
        isand = 0;
        isdiv = 0;
        ismod = 0;
        ismov = 0;

        // Apply test vectors
        #10;
        A = 32'h00000005;
        B = 32'h00000003;
        isadd = 1; // Test addition
        #10;
        isadd = 0;
        $display("Addition: A = %h, B = %h, ALURESULT = %h, zero_flag = %b, overflow_flag = %b", A, B, ALURESULT, zero_flag, overflow_flag);

        #10;
        isld = 1; // Test load
        #10;
        isld = 0;
        $display("Load: A = %h, B = %h, ALURESULT = %h, zero_flag = %b, overflow_flag = %b", A, B, ALURESULT, zero_flag, overflow_flag);

        #10;
        issub = 1; // Test subtraction
        #10;
        issub = 0;
        $display("Subtraction: A = %h, B = %h, ALURESULT = %h, zero_flag = %b, overflow_flag = %b", A, B, ALURESULT, zero_flag, overflow_flag);

        #10;
        iscmp = 1; // Test comparison
        #10;
        iscmp = 0;
        $display("Comparison: A = %h, B = %h, ALURESULT = %h, zero_flag = %b, flag_gt = %b", A, B, ALURESULT, zero_flag, flag_gt);

        #10 $finish;
    end
endmodule */

