module Instruction_Memory(
  input [31:0] pc,
  output [31:0] instruction
);

  reg [31:0] memory [0:255]; // 256 instructions

  initial begin
    $readmemb("test.prog", memory); // Load instructions from file
  end

  assign instruction = memory[pc[7:0]]; // directly use byte address as word index
endmodule