/*
Northwestern University
CompEng361 - Fall 2024
Lab 2
Name: Mark Chauhan
NetID: mcq8127
*/

module ExecutionUnit(
output [31:0] out,
input [31:0] opA,
input [31:0] opB,
input [2:0] func,
input [6:0] auxFunc);

wire [31:0] add_ans = opA + opB;
wire [31:0] sub_ans = opA - opB; 
wire [31:0] sll_ans = opA << opB[4:0];
wire [31:0] slt_ans = ($signed(opA) < $signed(opB)) ? 32'b1 : 32'b0;
wire [31:0] sltu_ans = (opA < opB) ? 32'b1 : 32'b0;
wire [31:0] xor_ans = opA ^ opB;
wire [31:0] srl_ans = opA >> opB[4:0];
wire [31:0] sra_ans = $signed(opA) >>> opB[4:0];
wire [31:0] or_ans = opA | opB;
wire [31:0] and_ans = opA & opB;

assign out = (func == 3'b000 && auxFunc == 7'b0000000) ? add_ans :   // ADD
             (func == 3'b000 && auxFunc == 7'b0100000) ? sub_ans :   // SUB
             (func == 3'b001 && auxFunc == 7'b0000000) ? sll_ans :   // SLL
             (func == 3'b010 && auxFunc == 7'b0000000) ? slt_ans :   // SLT
             (func == 3'b011 && auxFunc == 7'b0000000) ? sltu_ans :  // SLTU
             (func == 3'b100 && auxFunc == 7'b0000000) ? xor_ans :   // XOR
             (func == 3'b101 && auxFunc == 7'b0000000) ? srl_ans :   // SRL
             (func == 3'b101 && auxFunc == 7'b0100000) ? sra_ans :   // SRA
             (func == 3'b110 && auxFunc == 7'b0000000) ? or_ans :    // OR
             (func == 3'b111 && auxFunc == 7'b0000000) ? and_ans :   // AND
             32'b0; 

endmodule // ExecutionUnit
