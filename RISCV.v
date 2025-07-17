


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
    wire isImmediate;
    wire isWb;
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

module Instruction_Memory(
    input  [7:0] pc,                 // 8-bit PC for up to 256 instructions
    output reg [31:0] instruction
);

    always @(*) begin
        case (pc)
            8'd0:  instruction = 32'b00000000000000000000000000000000;
            8'd1:  instruction = 32'b00000000110000000001000000000000; // ADD   REG[6] = REG[0] + REG[1]
            8'd2:  instruction = 32'b00000000111001000011000000000000; // SUB   REG[7] = REG[2] - REG[3]
            8'd3:  instruction = 32'b00000001000010000101000000000000; // MUL   REG[8] = REG[4] * REG[5]
            8'd4:  instruction = 32'b00000001001001110010000000000000; // DIV   REG[9] = REG[3] / REG[2]
            8'd5:  instruction = 32'b00000001010010000000000000000000; // MOD   REG[10] = REG[4] % REG[0]
            8'd6:  instruction = 32'b00000001011001000001000000000000; // AND   REG[11] = REG[2] & REG[1]
            8'd7:  instruction = 32'b00000001100000000101000000000000; // OR    REG[12] = REG[0] | REG[5]
            8'd8:  instruction = 32'b00000001101010000000000000000000; // NOT   REG[13] = ~REG[4]
            8'd9:  instruction = 32'b00000001110000000001000000000000; // MOV   REG[14] = REG[1]
            8'd10: instruction = 32'b10000001111001000000000000010000; // LSL   REG[15] = REG[2] << 2
            8'd11: instruction = 32'b10000001000001100000000000000000; // LOAD  REG[8] = MEM[REG[6]]
            8'd12: instruction = 32'b10000000111010010000000000000000; // STORE MEM[REG[9]] = REG[7]
            default: instruction = 32'b0;
        endcase
    end
endmodule


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

module Controlpath (
    input [31:0] inst,
    input clk, // opcode from inst[31:27]
    output reg isSt,
    output reg isLd,
    output reg isBeq,
    output reg isBgt,
    output reg isRet,
    output reg isImmediate,
    output reg isWb,
    output reg isUbranch,
    output reg isCall,
    output reg isAdd,
    output reg isSub,
    output reg isCmp,
    output reg isMul,
    output reg isDiv,
    output reg isMod,
    output reg isLsl,
    output reg isLsr,
    output reg isAsr,
    output reg isOr,
    output reg isAnd,
    output reg isNot,
    output reg isMov
);

    always @(posedge clk) begin
        // Initialize all control signals to 0
        
       { isSt, isLd, isBeq, isBgt, isRet, isImmediate, isWb, 
          isUbranch, isCall, isAdd, isSub, isCmp, isMul, isDiv, 
          isMod, isLsl, isLsr, isAsr, isOr, isAnd, isNot, isMov } = 22'b0;

        case(inst[31:27])
    5'b00000: begin isAdd = 1; isWb = 1; end
    5'b00001: begin isSub = 1; isWb = 1; end
    5'b00010: begin isMul = 1; isWb = 1; end
    5'b00011: begin isDiv = 1; isWb = 1; end
    5'b00100: begin isMod = 1; isWb = 1; end
    5'b00101: begin isCmp = 1; isWb = 1; end
    5'b00110: begin isAnd = 1; isWb = 1; end
    5'b00111: begin isOr = 1; isWb = 1; end
    5'b01000: begin isNot = 1; isWb = 1; end
    5'b01001: begin isMov = 1; isWb = 1; end
    5'b01010: begin isLsl = 1; isWb = 1; end
    5'b01011: begin isLsr = 1; isWb = 1; end
    5'b01100: begin isAsr = 1; isWb = 1; end
    5'b01110: begin isLd = 1; isWb = 1; end
    5'b01111: begin isSt = 1; end
    5'b10000: begin isBeq = 1; end
    5'b10001: begin isBgt = 1; end
    5'b10010: begin isUbranch = 1; end
    5'b10011: begin isCall = 1; isWb = 1; end
    5'b10100: begin isRet = 1; end
    default: ; // Default case
endcase
        if (inst[26]) isImmediate = 1;
    end
             
endmodule


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


module ALU (
    input [31:0] a,        // Operand A (32-bit)
    input [31:0] b,        // Operand B (32-bit)
    input isadd,           // Control signal for addition
    input isld,            // Control signal for load
    input isst,            // Control signal for store
    input issub,           // Control signal for subtraction
    input iscmp,           // Control signal for comparison
    input ismul,           // Control signal for multiplication
    input islsl,           // Control signal for logical shift left
    input islsr,           // Control signal for logical shift right
    input isasr,           // Control signal for arithmetic shift right
    input isor,            // Control signal for OR
    input isnot,           // Control signal for NOT
    input isand,           // Control signal for AND
    input isdiv,           // Control signal for division
    input ismod,           // Control signal for modulo
    input ismov,           // Control signal for move
    input clk,             // Clock signal
    output reg [31:0] alu_result, // ALU result (32-bit)
    output reg zero_flag,         // Zero flag
    output reg overflow_flag,     // Overflow flag
    output reg flag_gt            // Greater than flag
);

always @(posedge clk) begin
    // Initialize flags
    zero_flag = 0;
    overflow_flag = 0;
    flag_gt = 0;

    // Combine control signals into a single value
    case (1'b1) // Case selector is based on active control signal
        isadd: begin
            alu_result = a + b;
            overflow_flag = (a + b > 32'hFFFFFFFF) ? 1 : 0; // Overflow detection
        end
        isld, isst: begin
            alu_result = a + b; // Load/Store address calculation
        end
        issub: begin
            alu_result = a - b;
            overflow_flag = (a < b) ? 1 : 0; // Overflow detection for subtraction
        end
        iscmp: begin
            if ((a - b) > 0) flag_gt = 1;
            else if ((a - b) == 0) zero_flag = 1;
        end
        ismul: begin
            alu_result = a * b; // Multiplication
        end
        islsl: begin
            alu_result = a << b; // Logical shift left
        end
        islsr: begin
            alu_result = a >> b; // Logical shift right
        end
        isasr: begin
            alu_result = $signed(a) >>> b; // Arithmetic shift right
        end
        isor: begin
            alu_result = a | b; // OR
        end
        isnot: begin
            alu_result = ~a; // NOT
        end
        isand: begin
            alu_result = a & b; // AND
        end
        isdiv: begin
            alu_result = a / b; // Division
        end
        ismod: begin
            alu_result = a % b; // Modulo
        end
        ismov: begin
            alu_result = b; // Move operation
        end
        default: begin
            alu_result = 32'h00000000; // Default case
        end
    endcase

    // Set zero flag
    zero_flag = (alu_result == 32'h00000000) ? 1 : 0;
end

endmodule


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
