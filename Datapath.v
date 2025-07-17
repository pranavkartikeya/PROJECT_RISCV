`include "IF.v"
`include "OF.v"
`include "EX.v"
`include "MA.v"
`include "RW.v"
`include "RegFile1.v"


module Datapath(
    input  clk
);
    wire isBeq, isBgt,isUBranch, isRet,isWb;
    wire isAdd, isLd, isSt, isSub, isCmp, isMul, isLsl, isLsr, isAsr, isOr, isNot, isAnd, isDiv, isMod, isMov,isCall;
    wire [31:0] Branchpc, branchtarget;
    wire isBranchtaken;
    wire [31:0] IF_OF_INST, IF_OF_PC, A, B, OP2, OF_EX_PC, OF_EX_INST, ALURESULT, EX_MA_PC, EX_MA_INST, EX_MA_OP2;
    wire zero_flag, overflow_flag, flag_gt;
    wire [31:0] MA_RW_aluresult, Ldresult, MA_RW_inst, MA_RW_pc;
    wire [3:0]mux1out,mux2out;
    wire [3:0] muxaddr;
    wire [31:0] muxdata;
    wire isSt_EM;
    wire isLd_EM;
    wire isWb_EM;
    wire isCall_EM;
    wire isCall_MR,isWb_MR,isLd_MR;
    wire [31:0]immx;


    IF A1 (
        .isBranchtaken(isBranchtaken), 
        .BranchPC(Branchpc), 
        .IF_OF_PC(IF_OF_PC), 
        .IF_OF_INST(IF_OF_INST), 
        .clk(clk)
    );

      RegFile1 Z(
        .clk(clk),
        .regread_dest1(mux1out),
        .regread_dest2(mux2out),
        .A(A),
        .OP2(OP2),
        .regwrite_en(isWb_MR),
        .regwrite_dest(muxaddr), 
        .regwrite_data(muxdata),
        .B(B),
        .isImmediate(isImmediate),
        .immx(immx)
    );

      OF A2 (
        .clk(clk), 
        .inst(IF_OF_INST),
        .pc(IF_OF_PC),
        .OF_EX_PC(OF_EX_PC),
        .OF_EX_INST(OF_EX_INST),
        .branchtarget(branchtarget),
        .isSt_OE(isSt), 
        .isLd_OE(isLd), 
        .isBeq_OE(isBeq), 
        .isBgt_OE(isBgt), 
        .isRet_OE(isRet), 
        .isImmediate_OE(isImmediate), 
        .isWb_OE(isWb), 
        .isUbranch_OE(isUbranch), 
        .isCall_OE(isCall), 
        .isAdd_OE(isAdd), 
        .isSub_OE(isSub), 
        .isCmp_OE(isCmp), 
        .isMul_OE(isMul), 
        .isDiv_OE(isDiv), 
        .isMod_OE(isMod), 
        .isLsl_OE(isLsl), 
        .isLsr_OE(isLsr), 
        .isAsr_OE(isAsr), 
        .isOr_OE(isOr), 
        .isAnd_OE(isAnd), 
        .isNot_OE(isNot), 
        .isMov_OE(isMov),
        .mux1out(mux1out),
        .mux2out(mux2out),
        .immx(immx)
      );

      
        EX A3 (
            .A(A),
            .B(B),
            .OP2(OP2),
            .INST(OF_EX_INST),
            .PC(OF_EX_PC),
            .branchtarget(branchtarget),
            .clk(clk),
            .isBeq(isBeq),
            .isBgt(isBgt),
            .isUBranch(isUbranch),
            .isRet(isRet),
            .isadd(isAdd),
            .isld(isLd),
            .isst(isSt),
            .issub(isSub),
            .iscmp(isCmp),
            .ismul(isMul),
            .islsl(isLsl),
            .islsr(isLsr),
            .isasr(isAsr),
            .isor(isOr),
            .isnot(isNot),
            .isand(isAnd),
            .isdiv(isDiv),
            .ismod(isMod),
            .ismov(isMov),
            .isWb(isWb),
            .iscall(isCall),
            .isSt_EM(isSt_EM),
            .isLd_EM(isLd_EM),
            .isWb_EM(isWb_EM),
            .isCall_EM(isCall_EM),
            .ALURESULT(ALURESULT),
            .EX_MA_PC(EX_MA_PC),
            .EX_MA_OP2(EX_MA_OP2),
            .EX_MA_INST(EX_MA_INST),
            .isBranchtaken(isBranchtaken),
            .branchpc(Branchpc),
            .zero_flag(zero_flag),
            .overflow_flag(overflow_flag),
            .flag_gt(flag_gt)
        );

    MA A4 (
        .aluresult(ALURESULT),
        .op2(EX_MA_OP2),
        .clk(clk),
        .inst(EX_MA_INST),
        .pc(EX_MA_PC),
        .MA_RW_aluresult(MA_RW_aluresult),
        .Ldresult(Ldresult),
        .MA_RW_inst(MA_RW_inst),
        .MA_RW_pc(MA_RW_pc),
        .isld(isLd_EM),
        .isSt(isSt_EM),
        .isCall(isCall_EM),
        .isWb(isWb_EM),
        .isCall_MR(isCall_MR),
        .isWb_MR(isWb_MR),
        .isLd_MR(isLd_MR)     
    );

    RW A5 (
        .MA_RW_aluresult(MA_RW_aluresult),
        .Ldresult(Ldresult),
        .MA_RW_inst(MA_RW_inst),
        .MA_RW_pc(MA_RW_pc),
        .clk(clk),
        .isCall(isCall_MR),
        .isld(isLd_MR),
        .muxaddr(muxaddr),
        .muxdata(muxdata)
    );

endmodule

`timescale 1ns/1ps


module RISC_tb;

    // Inputs
    reg clk;

    // Instantiate the RISC module
    Datapath uut (
        .clk(clk)
    );

    initial begin
        // Initialize Inputs
        clk = 1;
        // Stop simulation after sufficient time
        #200;
        $finish;
    end

    // Clock generation
    always begin
        #5 clk = ~clk;  // Toggle clock every 10 time units
    end

    initial begin
        $dumpfile("RISC_tb.vcd"); // Specify the dump file name
        $dumpvars(0, RISC_tb);    // Dump all variables in this module
    end
    endmodule
