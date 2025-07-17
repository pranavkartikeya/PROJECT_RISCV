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
