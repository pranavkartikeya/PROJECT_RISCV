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

/* module ALU_tb;
    // Testbench variables
    reg [31:0] a, b;
    reg isadd, isld, isst, issub, iscmp, ismul, islsl, islsr, isasr, isor, isnot, isand, isdiv, ismod, ismov, clk;
    wire [31:0] alu_result;
    wire zero_flag, overflow_flag, flag_gt;

    // Instantiate the ALU
    ALU uut (
        .a(a),
        .b(b),
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
        .alu_result(alu_result),
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
        a = 32'h0;
        b = 32'h0;
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
        a = 32'h00000005;
        b = 32'h00000003;
        isadd = 1; // Test addition
        #10;
        isadd = 0;
        $display("Addition: a = %h, b = %h, result = %h, zero_flag = %b, overflow_flag = %b", a, b, alu_result, zero_flag, overflow_flag);

        #10;
        isld = 1; // Test load
        #10;
        isld = 0;
        $display("Load: a = %h, b = %h, result = %h, zero_flag = %b, overflow_flag = %b", a, b, alu_result, zero_flag, overflow_flag);

        #10;
        issub = 1; // Test subtraction
        #10;
        issub = 0;
        $display("Subtraction: a = %h, b = %h, result = %h, zero_flag = %b, overflow_flag = %b", a, b, alu_result, zero_flag, overflow_flag);

        #10;
        iscmp = 1; // Test comparison
        #10;
        iscmp = 0;
        $display("Comparison: a = %h, b = %h, result = %h, zero_flag = %b, overflow_flag = %b, flag_gt = %b", a, b, alu_result, zero_flag, overflow_flag, flag_gt);

        // Add more test vectors as needed
        #10 $finish;
    end
endmodule */

