// Template for Northwestern - CompEng 361 - Lab3 -- Version 1.1
// Groupname: GeniusStudents
// NetIDs: Vab6121, MCQ8127

// Some useful defines...please add your own
`define WORD_WIDTH 32
`define NUM_REGS 32
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011 
`define OPCODE_LUI        7'b0110111
`define OPCODE_AUIPC      7'b0010111
`define OPCODE_JAL        7'b1101111  
`define OPCODE_JALR       7'b1100111 
`define OPCODE_IMM        7'b0010011

// Define function codes for R-type operations
`define FUNC_ADD      3'b000
`define FUNC_SUB      3'b000
`define FUNC_XOR      3'b100
`define FUNC_OR       3'b110
`define FUNC_AND      3'b111
`define FUNC_SLL      3'b001
`define FUNC_SRL      3'b101
`define FUNC_SRA      3'b101
`define FUNC_SLT      3'b010
`define FUNC_SLTU     3'b011
// Define function codes for Branch operations
`define FUNC_BEQ      3'b000
`define FUNC_BNE      3'b001
`define FUNC_BLT      3'b100
`define FUNC_BGE      3'b101
`define FUNC_BLTU     3'b110
`define FUNC_BGEU     3'b111
// Define function codes for Load/Store operations
`define FUNC_LB       3'b000
`define FUNC_LH       3'b001
`define FUNC_LW       3'b010
`define FUNC_LBU      3'b100
`define FUNC_LHU      3'b101
`define FUNC_SB       3'b000
`define FUNC_SH       3'b001
`define FUNC_SW       3'b010
// Define function codes for Multiplication/Division
`define FUNC_MUL       3'b000  
`define FUNC_MULH      3'b001  
`define FUNC_MULHSU    3'b010  
`define FUNC_MULHU     3'b011  
`define FUNC_DIV       3'b100  
`define FUNC_DIVU      3'b101  
`define FUNC_REM       3'b110  
`define FUNC_REMU      3'b111  

// Define auxiliary function codes
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define AUX_FUNC_SRA  7'b0100000
`define AUX_FUNC_MUL  7'b0000001

`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10

module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC, InstWord;
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, RWrdata;
   wire        RWrEn;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;
   wire [6:0]  opcode;

   wire [6:0]  funct7;
   wire [2:0]  funct3;

   wire branchTaken; 

   wire invalid_op;
   
   // Only support R-TYPE ADD and SUB
   assign halt = invalid_op;
   
   assign invalid_op = !((opcode == `OPCODE_COMPUTE && (funct3 == `FUNC_ADD || funct3 == `FUNC_SUB ||
                      funct3 == `FUNC_XOR || funct3 == `FUNC_OR || funct3 == `FUNC_AND ||
                      funct3 == `FUNC_SLL || funct3 == `FUNC_SRL || funct3 == `FUNC_SRA ||
                      funct3 == `FUNC_SLT || funct3 == `FUNC_SLTU ||
                      funct3 == `FUNC_MUL || funct3 == `FUNC_MULH || funct3 == `FUNC_MULHSU ||
                      funct3 == `FUNC_MULHU || funct3 == `FUNC_DIV || funct3 == `FUNC_DIVU ||
                      funct3 == `FUNC_REM || funct3 == `FUNC_REMU)) ||
                      (opcode == `OPCODE_LUI) ||
                      (opcode == `OPCODE_AUIPC) ||
                      (opcode == `OPCODE_JAL) ||
                      (opcode == `OPCODE_JALR) ||
                      (opcode == `OPCODE_BRANCH) ||
                      (opcode == `OPCODE_LOAD) ||
                      (opcode == `OPCODE_STORE) || 
                      (opcode == `OPCODE_IMM));

   wire [`WORD_WIDTH-1:0] immU; // LUI imm
   assign immU = {InstWord[31:12], 12'b0}; // Immediate for LUI with lower 12 bits zeroed

   //Instruction Decode
   assign opcode = InstWord[6:0];   
   assign Rdst = InstWord[11:7]; 
   assign Rsrc1 = InstWord[19:15]; 
   assign Rsrc2 = InstWord[24:20];
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = InstWord[31:25];  // R-Type
  
  //check and add - any instruction that writes to a register needs to be added
   assign RWrEn = (opcode == `OPCODE_COMPUTE) || 
                  (opcode == `OPCODE_IMM) ||   
                  (opcode == `OPCODE_LUI) ||
                  (opcode == `OPCODE_AUIPC) ||
                  (opcode == `OPCODE_JAL) ||
                  (opcode == `OPCODE_JALR) ||
                  (opcode == `OPCODE_LOAD);



   //JAL and JALQ
   wire [`WORD_WIDTH-1:0] immJ; 
   //wire [`WORD_WIDTH-1:0] immJr; 
   wire [`WORD_WIDTH-1:0] immI; //Immediate for Load

   assign immJ = {{12{InstWord[31]}}, InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0};  
   assign immI = {{20{InstWord[31]}}, InstWord[31:20]}; 


    //store data and load processing
    wire [`WORD_WIDTH-1:0] immS; 
    assign immS = {{20{InstWord[31]}}, InstWord[31:25], InstWord[11:7]}; // S-Type
   
    assign MemSize = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? funct3[1:0] : 2'b00; // Load/store operations - what should the default be? 
    assign MemWrEn = (opcode == `OPCODE_STORE); // Store operations
    //assign DataAddr = Rdata1 + immS;           // Compute effective address
    assign DataAddr = (opcode == `OPCODE_LOAD) ? (Rdata1 + immI) : (Rdata1 + immS); 

    assign StoreData = (opcode == `OPCODE_STORE && funct3 == `FUNC_SB)   ? {24'b0, Rdata2[7:0]} :   //LSB for sb
                      (opcode == `OPCODE_STORE && funct3 == `FUNC_SH)   ? {16'b0, Rdata2[15:0]} :  // Least significant 16 bits for sh
                      (opcode == `OPCODE_STORE && funct3 == `FUNC_SW)   ? Rdata2 :                 // Full word for sw
                      32'hXXXXXXXX; // Default for unsupported funct3

    // System State 
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	      .AddrB(Rsrc2), .DataOutB(Rdata2), 
	      .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));

    // Load Data Processing
    wire [`WORD_WIDTH-1:0] loadData;
    assign loadData = (opcode == `OPCODE_LOAD && funct3 == `FUNC_LB)  ? {{24{DataWord[7]}}, DataWord[7:0]}  : // Sign-extend byte
                     (opcode == `OPCODE_LOAD && funct3 == `FUNC_LH)  ? {{16{DataWord[15]}}, DataWord[15:0]} : // Sign-extend halfword
                     (opcode == `OPCODE_LOAD && funct3 == `FUNC_LW)  ? DataWord :                            // Load word
                     (opcode == `OPCODE_LOAD && funct3 == `FUNC_LBU) ? {{24{1'b0}}, DataWord[7:0]} :         // Zero-extend byte
                     (opcode == `OPCODE_LOAD && funct3 == `FUNC_LHU) ? {{16{1'b0}}, DataWord[15:0]} :        // Zero-extend halfword
                     32'hXXXXXXXX; 

   //note: opcode descruct was here


   //BRANCH
   wire [`WORD_WIDTH-1:0] immB;
   assign immB = {{20{InstWord[31]}}, InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0};


   assign branchTaken = (opcode == `OPCODE_BRANCH) && (
                         (funct3 == `FUNC_BEQ  && (Rdata1 == Rdata2)) ||
                         (funct3 == `FUNC_BNE  && (Rdata1 != Rdata2)) ||
                         (funct3 == `FUNC_BLT  && ($signed(Rdata1) < $signed(Rdata2))) ||
                         (funct3 == `FUNC_BGE  && ($signed(Rdata1) >= $signed(Rdata2))) ||
                         (funct3 == `FUNC_BLTU && (Rdata1 < Rdata2)) ||
                         (funct3 == `FUNC_BGEU && (Rdata1 >= Rdata2))
                     );

    //load and store originally here

   wire [`WORD_WIDTH-1:0] selectedImm; 
   assign selectedImm = (opcode == `OPCODE_LUI || opcode == `OPCODE_AUIPC) ? immU : 
                        (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR) ? immJ : immI; 
   
   ExecutionUnit EU(.out(RWrdata), .opA(Rdata1), .opB(Rdata2), .func(funct3), .auxFunc(funct7), .imm(selectedImm), .opcode(opcode), .pc(PC), .loadData(loadData));

   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC = (opcode == `OPCODE_JAL)  ? (PC + immJ) :             // JAL
                 (branchTaken) ? (PC + immB) :                        //Branch
                 (opcode == `OPCODE_JALR) ? ((Rdata1 + immI) & ~1) :  // JALR
                 PC_Plus_4;  // PC + 4
   
endmodule // SingleCycleCPU



/*
module ExecutionUnit(out, opA, opB, func, auxFunc, imm, opcode, pc, loadData);
   output [`WORD_WIDTH-1:0] out;
   input [`WORD_WIDTH-1:0]  opA, opB, imm, pc, loadData;
   input [2:0] 	 func;
   input [6:0] 	 auxFunc, opcode;

   wire [`WORD_WIDTH-1:0] result;

   //for multiplication 
   wire signed [63:0] signed_product = $signed(opA) * $signed(opB); 
   wire [63:0] unsigned_product = opA * opB; 

   //division
   wire signed [64:0] signed_div = $signed(opA) / $signed(opB);
   wire [64:0] unsigned_div = opA / opB;

   //rem                     
   wire signed [64:0] signed_rem = $signed(opA) % $signed(opB);

    // Immediate operations
   assign result = (func == 3'b000 && opcode == `OPCODE_IMM  && auxFunc == `AUX_FUNC_ADD) ? (opA + imm) :                 // ADDI -CHECK 
                   (func == 3'b010 && opcode == `OPCODE_IMM) && auxFunc == 7'b0000001 ? ($signed(opA) < $signed(imm)) : // SLTI
                   (func == 3'b011 && opcode == `OPCODE_IMM) && auxFunc != 7'b0000001 ? ($unsigned(opA) < $unsigned(imm)) : // SLTIU
                   (func == 3'b100 && opcode == `OPCODE_IMM && auxFunc != 7'b0000001) ? (opA ^ imm) :   // XORI - added auxfunc bc it was interfering with div
                   (func == 3'b110 && opcode == `OPCODE_IMM && auxFunc != 7'b0000001) ? (opA | imm) :   // ORI
                   (func == 3'b111 && opcode == `OPCODE_IMM && auxFunc != 7'b0000001) ? (opA & imm) :                 // ANDI
                   (func == 3'b001 && opcode == `OPCODE_IMM && auxFunc != 7'b0000001) ? (opA << imm[4:0]) :           // SLLI - added auxFunc for mulh
                   (func == 3'b101 && opcode == `OPCODE_IMM && opcode != `OPCODE_LOAD && auxFunc == 7'b0000000) ? (opA >> imm[4:0]) : // SRLI
                   (func == 3'b101 && opcode == `OPCODE_IMM && auxFunc == 7'b0100000) ? ((opA >> imm[4:0]) | ({32{opA[31]}} << (32 - imm[4:0]))): // SRAI

                   //register operations
                   (func == 3'b000 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_ADD) ? (opA + opB) : // ADD
                   (func == 3'b000 && opcode != `OPCODE_IMM && auxFunc == `AUX_FUNC_SUB) ? (opA - opB) : // SUB
                   (func == 3'b010 && opcode != `OPCODE_IMM && opcode != `OPCODE_LOAD) && auxFunc != 7'b0000001 ? ($signed(opA) < $signed(opB)) : // SLT
                   (func == 3'b011 && opcode != `OPCODE_IMM) && auxFunc != 7'b0000001 ? ($unsigned(opA) < $unsigned(opB)) : // SLTU
                   (func == 3'b100 && opcode != `OPCODE_IMM && opcode != `OPCODE_LOAD && auxFunc != 7'b0000001) ? (opA ^ opB) : // XOR - auxFunc so it doesn't interfere with div
                   (func == 3'b110 && opcode != `OPCODE_IMM && auxFunc != 7'b0000001) ? (opA | opB) : // OR
                   (func == 3'b101 && opcode != `OPCODE_IMM && opcode != `OPCODE_LOAD && auxFunc == 7'b0000000) ? (opA >> opB[4:0]) : // SRL
                   (func == 3'b101 && opcode != `OPCODE_IMM && opcode != `OPCODE_LOAD && auxFunc == 7'b0100000) ? ((opA >> opB) | ({32{opA[31]}} << (32 - opB[4:0]))): // SRA
                   (func == 3'b001 && opcode != `OPCODE_IMM && opcode != `OPCODE_LOAD && auxFunc != 7'b0000001) ? (opA << opB[4:0]) : // SLL
                   (func == 3'b111 && opcode != `OPCODE_IMM && auxFunc != 7'b0000001) ? (opA & opB) : // AND

                   //multiply
                   (func == 3'b000 && opcode != `OPCODE_IMM) && auxFunc == 7'b0000001 ? ($signed(opA) * $signed(opB)) : // MUL
                   (func == 3'b001 && opcode != `OPCODE_IMM && opcode != `OPCODE_LOAD) && auxFunc == 7'b0000001 ? signed_product[63:32] : // MULH 
                   (func == 3'b010 && opcode != `OPCODE_IMM && opcode != `OPCODE_LOAD) && auxFunc == 7'b0000001 ? (({{32{opA[31]}}, opA } * {32'b0, opB }) >> 32) : // MULHSU 
                   (func == 3'b011 && opcode != `OPCODE_IMM) && auxFunc == 7'b0000001 ? unsigned_product[63:32] : // MULHU

                   //divide and rem
                   (func == 3'b100 && opcode != `OPCODE_IMM && opcode != `OPCODE_LOAD) && auxFunc == 7'b0000001 ? 
                     (opB != 0 ? signed_div : 32'hXXXXXXXX) : // DIV, also accounting if dividing by zero

                   (func == 3'b110 && opcode != `OPCODE_IMM) && auxFunc == 7'b0000001 ? 
                     (opB != 0 ? signed_rem : 32'hXXXXXXXX) : // REM 

                   (func == 3'b10 && opcode != `OPCODE_IMM && opcode != `OPCODE_LOAD) && auxFunc == 7'b0000001 ? 
                     (opB != 0 ? unsigned_div : 32'hXXXXXXXX) : // DIVU
                   (func == 3'b111 && opcode != `OPCODE_IMM) && auxFunc == 7'b0000001 ? 
                     (opB != 0 ? opA % opB : 32'hXXXXXXXX) : // REMU 

                   //lui and auipc 
                   (opcode == `OPCODE_LUI) ? imm : // LUI
                   (opcode == `OPCODE_AUIPC) ? (pc + imm) : // AUIPC

                   //jal and jalr 
                   (opcode == `OPCODE_JAL) ? (pc + 4) :
                   (opcode == `OPCODE_JALR) ? (pc + 4) :

                   //loads
                   (opcode == `OPCODE_LOAD) ? loadData : 

                   32'hXXXXXXXX;  // Default value in case no condition is met

      assign out = result; 


endmodule // ExecutionUnit

*/


module ExecutionUnit(out, opA, opB, func, auxFunc, imm, opcode, pc, loadData);
   output [`WORD_WIDTH-1:0] out;
   input [`WORD_WIDTH-1:0]  opA, opB, imm, pc, loadData;
   input [2:0] 	 func;
   input [6:0] 	 auxFunc, opcode;

   wire [`WORD_WIDTH-1:0] result;

   //for multiplication 
   wire signed [63:0] signed_product = $signed(opA) * $signed(opB); 
   wire [63:0] unsigned_product = opA * opB; 

   //division
   wire signed [64:0] signed_div = $signed(opA) / $signed(opB);
   wire [64:0] unsigned_div = opA / opB;

   //rem                     
   wire signed [64:0] signed_rem = $signed(opA) % $signed(opB);

   //sll and sra 
   wire [31:0] srl_result = (opA >> opB[4:0]
   wire [31:0] sra_result;
   assign sra_result = (!opA[31]) ? srl_result :
                       32'hffffffff << (32 - opB[4:0]) | srl_result;

    // Immediate operations
   assign result = (func == 3'b000 && opcode == `OPCODE_IMM) ? (opA + imm) : // ADDI
                   (func == 3'b010 && opcode == `OPCODE_IMM) ? ($signed(opA) < $signed(imm)) : // SLTI
                   (func == 3'b011 && opcode == `OPCODE_IMM) ? ($unsigned(opA) < $unsigned(imm)) : // SLTIU
                   (func == 3'b100 && opcode == `OPCODE_IMM) ? (opA ^ imm) :   // XORI 
                   (func == 3'b110 && opcode == `OPCODE_IMM) ? (opA | imm) :   // ORI
                   (func == 3'b111 && opcode == `OPCODE_IMM) ? (opA & imm) :   // ANDI
                   (func == 3'b001 && opcode == `OPCODE_IMM) ? (opA << imm[4:0]) :  // SLLI 
                   (func == 3'b101 && opcode == `OPCODE_IMM && auxFunc == 7'b0000000) ? (opA >> imm[4:0]) : // SRLI
                   (func == 3'b101 && opcode == `OPCODE_IMM && auxFunc == 7'b0100000) ? ((opA >> imm[4:0]) | ({32{opA[31]}} << (32 - imm[4:0]))): // SRAI

                   //register operations
                   (func == 3'b000 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_ADD) ? (opA + opB) : // ADD
                   (func == 3'b000 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_SUB) ? (opA - opB) : // SUB
                   (func == 3'b010 && opcode == `OPCODE_COMPUTE && auxFunc != `AUX_FUNC_MUL) ? ($signed(opA) < $signed(opB)) : // SLT
                   (func == 3'b011 && opcode == `OPCODE_COMPUTE && auxFunc != `AUX_FUNC_MUL) ? ($unsigned(opA) < $unsigned(opB)) : // SLTU
                   (func == 3'b100 && opcode == `OPCODE_COMPUTE && auxFunc != `AUX_FUNC_MUL) ? (opA ^ opB) : // XOR 
                   (func == 3'b110 && opcode == `OPCODE_COMPUTE && auxFunc != `AUX_FUNC_MUL) ? (opA | opB) : // OR
                   (func == 3'b101 && opcode == `OPCODE_COMPUTE && auxFunc == 7'b0000000) ? srl_result : // SRL
                   (func == 3'b101 && opcode == `OPCODE_COMPUTE && auxFunc == 7'b0100000) ? sra_result: // SRA
                   (func == 3'b001 && opcode == `OPCODE_COMPUTE && auxFunc != `AUX_FUNC_MUL) ? (opA << opB[4:0]) : // SLL
                   (func == 3'b111 && opcode == `OPCODE_COMPUTE && auxFunc != `AUX_FUNC_MUL) ? (opA & opB) : // AND

                   //multiply
                   (func == 3'b000 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_MUL) ? ($signed(opA) * $signed(opB)) : // MUL
                   (func == 3'b001 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_MUL) ? signed_product[63:32] : // MULH 
                   (func == 3'b010 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_MUL) ? (({{32{opA[31]}}, opA } * {32'b0, opB }) >> 32) : // MULHSU 
                   (func == 3'b011 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_MUL) ? unsigned_product[63:32] : // MULHU

                   //divide and rem
                   (func == 3'b100 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_MUL) ? 
                     (opB != 0 ? signed_div : 32'hXXXXXXXX) : // DIV, also accounting if dividing by zero
                   (func == 3'b110 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_MUL) ? 
                     (opB != 0 ? signed_rem : 32'hXXXXXXXX) : // REM 
                   (func == 3'b101 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_MUL) ? 
                     (opB != 0 ? unsigned_div : 32'hXXXXXXXX) : // DIVU
                   (func == 3'b111 && opcode == `OPCODE_COMPUTE && auxFunc == `AUX_FUNC_MUL) ? 
                     (opB != 0 ? opA % opB : 32'hXXXXXXXX) : // REMU 

                   //lui and auipc 
                   (opcode == `OPCODE_LUI) ? imm : // LUI
                   (opcode == `OPCODE_AUIPC) ? (pc + imm) : // AUIPC

                   //jal and jalr 
                   (opcode == `OPCODE_JAL) ? (pc + 4) :
                   (opcode == `OPCODE_JALR) ? (pc + 4) :

                   //loads
                   (opcode == `OPCODE_LOAD) ? loadData : 

                   32'hXXXXXXXX;  // Default value in case no condition is met

      assign out = result; 


endmodule // ExecutionUnit