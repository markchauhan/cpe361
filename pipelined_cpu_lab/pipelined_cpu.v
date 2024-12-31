// Template for Northwestern - CompEng 361 - Lab4 -- Version 1.1
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

// Latency definitions
`define LATENCY_MUL   4
`define LATENCY_DIV  20

/*
module HazardDetectionUnit(
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg stall
);
    always @(*) begin
        // Check for multiply/divide instructions
        if (opcode == `OPCODE_COMPUTE && funct7 == `AUX_FUNC_MUL &&
            (funct3 == `FUNC_MUL || funct3 == `FUNC_DIV || 
             funct3 == `FUNC_REM || funct3 == `FUNC_DIVU || funct3 == `FUNC_REMU)) begin
            stall = 1'b1; // Signal stall
        end else begin
            stall = 1'b0; // No stall
        end
    end
endmodule
*/


module PipelinedCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC, InstWord; //take out PC
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
    wire [`WORD_WIDTH-1:0] branchTarget;
    wire [`WORD_WIDTH-1:0] jumpTarget;  

    //check if all of these are used and if not, delete them.
   wire invalid_op;
   wire [31:0] Imm;
   wire [31:0] MemAddress;
   wire [31:0] MemWriteData;
   wire [31:0] PC_next;
   wire [31:0] Result;
   wire [31:0] ExecResult;
   wire [31:0] loadData;
   wire [31:0] ExecMemAddress;

   wire [31:0] NPC_IF;


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
                      
   //JAL and JALQ
   wire [`WORD_WIDTH-1:0] immJ; 
   wire [`WORD_WIDTH-1:0] immI; //Immediate for Load
   assign immJ = {{12{InstWord[31]}}, InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0};  
   assign immI = {{20{InstWord[31]}}, InstWord[31:20]}; 

    // System State 
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	      .AddrB(Rsrc2), .DataOutB(Rdata2), 
	      .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));

   //BRANCH
   wire [`WORD_WIDTH-1:0] immB;
   assign immB = {{20{InstWord[31]}}, InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0};

    /*
   assign branchTaken = (opcode == `OPCODE_BRANCH) && (
                         (funct3 == `FUNC_BEQ  && (Rdata1 == Rdata2)) ||
                         (funct3 == `FUNC_BNE  && (Rdata1 != Rdata2)) ||
                         (funct3 == `FUNC_BLT  && ($signed(Rdata1) < $signed(Rdata2))) ||
                         (funct3 == `FUNC_BGE  && ($signed(Rdata1) >= $signed(Rdata2))) ||
                         (funct3 == `FUNC_BLTU && (Rdata1 < Rdata2)) ||
                         (funct3 == `FUNC_BGEU && (Rdata1 >= Rdata2))
                     );
                     */

    /* - idk where to put NPC?
   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC = (opcode == `OPCODE_JAL)  ? (PC + immJ) :             // JAL
                 (branchTaken) ? (PC + immB) :                        //Branch
                 (opcode == `OPCODE_JALR) ? ((Rdata1 + immI) & ~1) :  // JALR
                 PC_Plus_4;  // PC + 4

    */

    //pipeline stages

    wire [31:0] IF_InstData;
    assign IF_InstData = InstWord; //for debugging 
    wire [31:0] IF_InstData_out;

    wire [31:0] selected_NPC_imm;
    assign selected_NPC_imm = (opcode == `OPCODE_JAL)   ? immJ :           // JAL
                          (branchTaken)            ? immB :           // Branch
                          (opcode == `OPCODE_JALR) ? immI :            // JALR immediate
                          32'b0;  // Default: no immediate used, but for safety

    wire stall;

/*
// Instantiate Hazard Detection Unit
HazardDetectionUnit HDU (
    .opcode(opcode),
    .funct3(funct3),
    .funct7(funct7),
    .stall(stall)
);

reg [`WORD_WIDTH-1:0] IF_ID_PC, IF_ID_Inst;
reg [`WORD_WIDTH-1:0] ID_EX_PC, ID_EX_Inst, ID_EX_Rdata1, ID_EX_Rdata2, ID_EX_Imm;
reg [4:0] ID_EX_Rsrc1, ID_EX_Rsrc2, ID_EX_Rdst;
reg [2:0] ID_EX_funct3;
reg [6:0] ID_EX_funct7, ID_EX_opcode;

// Control logic for stalls
always @(posedge clk or posedge rst) begin
    if (rst || halt) begin
        // Reset pipeline registers
        IF_ID_PC <= 0;
        IF_ID_Inst <= 0;
        ID_EX_PC <= 0;
        ID_EX_Inst <= 0;
        ID_EX_Rdata1 <= 0;
        ID_EX_Rdata2 <= 0;
        ID_EX_Imm <= 0;
        ID_EX_Rsrc1 <= 0;
        ID_EX_Rsrc2 <= 0;
        ID_EX_Rdst <= 0;
        ID_EX_funct3 <= 0;
        ID_EX_funct7 <= 0;
        ID_EX_opcode <= 0;
    end else if (stall) begin
        // Hold values in current pipeline stages
        IF_ID_PC <= IF_ID_PC;
        IF_ID_Inst <= IF_ID_Inst;
        ID_EX_PC <= ID_EX_PC;
        ID_EX_Inst <= ID_EX_Inst;
        ID_EX_Rdata1 <= ID_EX_Rdata1;
        ID_EX_Rdata2 <= ID_EX_Rdata2;
        ID_EX_Imm <= ID_EX_Imm;
        ID_EX_Rsrc1 <= ID_EX_Rsrc1;
        ID_EX_Rsrc2 <= ID_EX_Rsrc2;
        ID_EX_Rdst <= ID_EX_Rdst;
        ID_EX_funct3 <= ID_EX_funct3;
        ID_EX_funct7 <= ID_EX_funct7;
        ID_EX_opcode <= ID_EX_opcode;
    end else begin
        // Normal pipeline operation
        IF_ID_PC <= PC;
        IF_ID_Inst <= InstWord;
        ID_EX_PC <= IF_ID_PC;
        ID_EX_Inst <= IF_ID_Inst;
        ID_EX_Rdata1 <= Rdata1;
        ID_EX_Rdata2 <= Rdata2;
        ID_EX_Imm <= Imm;
        ID_EX_Rsrc1 <= Rsrc1;
        ID_EX_Rsrc2 <= Rsrc2;
        ID_EX_Rdst <= Rdst;
        ID_EX_funct3 <= funct3;
        ID_EX_funct7 <= funct7;
        ID_EX_opcode <= opcode;
    end
end
*/



    //instruction fetch 
    InstructionFetch IF (
        .clk(clk),            
        .rst(rst),               
        .opcode(opcode),
        .PC_in(PC),        
        .InstWord_in(IF_InstData),
        .selected_NPC_imm(selected_NPC_imm),
        .InstWord_out(IF_InstData_out),     // output 
        .NPC_IF(NPC_IF),         // Output
        .Rdata1(Rdata1)
    );

    wire [31:0] ID_InstData_in;
    assign ID_InstData_in = IF_InstData_out; //for debugging - take out later 
    wire [31:0] NPC_ID;

    //instruction decode
    InstructionDecode ID (
        .clk(clk),            
        .rst(rst),                
        .InstWord(ID_InstData_in),      // inputs below this line 
        .Rdata1(Rdata1),            
        .Rdata2(Rdata2),           
        .PC_in(PC),                    
        .Imm(Imm),                      // outputs below this line
        .Rsrc1(Rsrc1),         
        .Rsrc2(Rsrc2),          
        .Rdst(Rdst),              
        .opcode(opcode),          
        .funct3(funct3),          
        .funct7(funct7),        
        .RWrEn(RWrEn),           
        .branchTaken(branchTaken),
        .jumpTaken(jumpTaken),
        .jumpRTaken(jumpRTaken),
        .NPC(NPC),               
        .halt(halt)             
    );

    //for debugging
    wire [31:0] Imm_ID_in;
    assign Imm_ID_in = Imm;
    wire [2:0] funct3_result;
    assign funct3_result = funct3;

    // Execute
    Execute EX(
        .clk(clk),                    // inputs below this line
        .rst(rst),
        .halt(halt),                  
        .PC_in(PC),                    
        .InstWord(InstWord),       
        .Rdata1(Rdata1),             
        .Rdata2(Rdata2),           
        .Imm(Imm_ID_in),                
        .Rsrc1(Rsrc1),               
        .Rsrc2(Rsrc2),         
        .Rdst(Rdst),                 
        .opcode(opcode),      
        .funct3(funct3),           
        .funct7(funct7),                  
        .branchTaken(branchTaken),  
        .jumpTaken(jumpTaken),
        //.NPC_In(NPC_ID),
        //.branchAddr(branchAddr),      // outputs below this line
        .Result(ExecResult),        
        .MemAddress(ExecMemAddress)
        //.PC_next(ExecPC_next)
        //.JumpAddr(ExecJumpAddr)     
    );

    //for debugging - make sure you are getting the right result, and also that the code is coming out of EX
    wire [31:0] ExecResult_EX;
    assign ExecResult_EX = ExecResult;


    //memory
    Memory PP_MEM (
        .clk(clk),                  //inputs below
        .rst(rst),
        .opcode(opcode),
        .Imm(Imm_ID_in),
        .funct3(funct3),
        .Rdata1(Rdata1),
        .Rdata2(Rdata2),
        .DataWord(DataWord),
        .MemAddress(ExecMemAddress),
        .PC(PC),
        
        .MemSize(MemSize),          //outputs below
        .MemWrEn(MemWrEn),
        .DataAddr(DataAddr),
        .StoreData(StoreData),
        .loadData(loadData)
    );
    
    //for debugging - make sure you are getting the right result, and also that the code is coming out of EX
    wire [1:0] MemSize_MEMO;
    assign MemSize_MEMO = MemSize;


    // writeback
    WriteBack WB( 
        .clk(clk),                      //inputs
        .rst(rst),
        .Result(ExecResult_EX),
        .MemSize(MemSize),
        .MemWrEn(MemWrEn),
        .MemAddress(ExecMemAddress),        
        .StoreData(StoreData),
        .loadData(loadData),
        .opcode(opcode),
        .Rsrc1(Rsrc1),
        .Rsrc2(Rsrc2),
        .Rdst(Rdst),
        .RegWrite(RWrEn),

        .halt_overall(halt_overall),    //outputs - do we need this one?
        .RWrdata(RWrdata),
        .RwrEn(RwrEn)
    );

    //for debugging
    wire [31:0] RWrData_Out;
    assign RWrData_Out = RWrdata;


endmodule 

module InstructionFetch (
    input clk,             
    input rst,            
    input [6:0] opcode,
    input [31:0] PC_in,    
    input [31:0] InstWord_in,
    input [31:0] selected_NPC_imm,
    output [31:0] InstWord_out,
    output [31:0] NPC_IF,   //is this needed? for stalls?
    input [31:0] Rdata1
);
    //reg [31:0] PC_next; 
    wire [31:0] IF_PC_Plus_4;
    assign InstWord_out = InstWord_in; //used for debugging mostly - should be getting right InstWord
    assign IF_PC_Plus_4 = PC_in + 4;
    
    //I don't think this is needed 
    /* Undo later - do everything but branches/jumps first
    assign NPC_IF = (opcode == `OPCODE_JALR) ? (Rdata1 + selected_NPC_imm) & ~1 :  // Handle JALR
                ((opcode == `OPCODE_JAL) || (opcode == `OPCODE_BRANCH)) ? (PC_in + selected_NPC_imm) : // Handle normal cases (JAL, Branch)
                IF_PC_Plus_4;  // Default to PC + 4
                */
    //assign NPC = NPC_IF;


    assign NPC_IF = PC_in + 4;

endmodule

module InstructionDecode (
    input clk,                // Clock signal
    input rst,                // Reset signal
    input [31:0] InstWord,    // Instruction word from IF stage
    input [31:0] Rdata1,      // Register data from Register 1 (from RegFile)
    input [31:0] Rdata2,      // Register data from Register 2 (from RegFile)
    input [31:0] PC_in,       // Program Counter value from IF stage
    output [31:0] Imm,        // Immediate value
    output [4:0] Rsrc1,       // Source register 1 (rs1)
    output [4:0] Rsrc2,       // Source register 2 (rs2)
    output [4:0] Rdst,        // Destination register (rd)
    output [6:0] opcode,      // Opcode (to determine the instruction type)
    output [2:0] funct3,      // Function code (for R/I/S-type instructions)
    output [6:0] funct7,      // Function code (for R-type instructions)
    output RWrEn,             // Register Write Enable
    output branchTaken,       // Branch taken signal
    output jumpTaken,         // Jump taken signal
    output jumpRTaken,
    output [31:0] NPC,        // Next Program Counter (for branch/jump operations)
    output halt               // Stop execution for invalid opcode
);
    // Define immediate values for each instruction format
    wire [31:0] immI, immS, immB, immU, immJ;
    wire [31:0] Imm_result;
    
    //Instruction Decode
    assign opcode = InstWord[6:0];   
    assign Rdst = InstWord[11:7]; 
    assign Rsrc1 = InstWord[19:15]; 
    assign Rsrc2 = InstWord[24:20];
    assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
    assign funct7 = InstWord[31:25];  // R-Type

    //immediates
    assign immI = {{20{InstWord[31]}}, InstWord[31:20]};  // I-type
    assign immS = {{20{InstWord[31]}}, InstWord[31:25], InstWord[11:7]};  // S-type
    assign immB = {{19{InstWord[31]}}, InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0}; // B-type
    assign immU = {InstWord[31:12], 12'b0};  // U-type (LUI, AUIPC)
    assign immJ = {{12{InstWord[31]}}, InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0};  // J-type (JAL, JALR)

    // choose correct immediate value based on the opcode
    assign Imm_result = (opcode == `OPCODE_IMM || opcode == `OPCODE_LOAD) ? immI :
                 (opcode == `OPCODE_STORE) ? immS :
                 (opcode == `OPCODE_BRANCH) ? immB :
                 (opcode == `OPCODE_LUI || opcode == `OPCODE_AUIPC) ? immU :
                 (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR) ? immJ : 
                 32'b0;  // Default to 0 for unsupported opcodes

    assign Imm = Imm_result; //for debugging - idk if the extra line is needed 

    // Register Write Enable (RWrEn) is set
    assign RWrEn = (opcode == `OPCODE_COMPUTE) || 
                   (opcode == `OPCODE_IMM) || 
                   (opcode == `OPCODE_LUI) ||
                   (opcode == `OPCODE_AUIPC) ||
                   (opcode == `OPCODE_JAL) ||
                   (opcode == `OPCODE_JALR) ||
                   (opcode == `OPCODE_LOAD);

    // Branch flag
    wire branchTakenSignal;
    assign branchTakenSignal = (opcode == `OPCODE_BRANCH) && (
                                (funct3 == `FUNC_BEQ && (Rdata1 == Rdata2)) ||
                                (funct3 == `FUNC_BNE && (Rdata1 != Rdata2)) ||
                                (funct3 == `FUNC_BLT && ($signed(Rdata1) < $signed(Rdata2))) ||
                                (funct3 == `FUNC_BGE && ($signed(Rdata1) >= $signed(Rdata2))) ||
                                (funct3 == `FUNC_BLTU && (Rdata1 < Rdata2)) ||
                                (funct3 == `FUNC_BGEU && (Rdata1 >= Rdata2))
    );
    assign branchTaken = branchTakenSignal; 
    
    // Jal and Jalr flag 
    wire jumpTakenSignal = (opcode == `OPCODE_JAL);
    assign jumpTaken = jumpTakenSignal;

    wire jumpRTakenSignal = (opcode == `OPCODE_JALR);
    assign jumpRTaken = jumpRTakenSignal;

    //could also use the flags here, but is NPC needed in this module?
    assign NPC = (opcode == `OPCODE_JALR) ? (Rdata1 + immI) & ~1 :  // Handle JALR
                 (opcode == `OPCODE_JAL)  ? (PC_in + immJ) :           // Handle JAL
                 (opcode == `OPCODE_BRANCH) ? (PC_in + immB) : //Branch
                 (PC_in + 4);  // Default to PC + 4 for non-branch/jump instructions

endmodule // InstructionDecode

module Execute (
    input clk,                        // Clock signal
    input rst,                        // Reset signal
    input halt,
    input [31:0] PC_in,                  // Program Counter (PC) value
    input [31:0] InstWord,            // Current Instruction from Instruction Fetch
    input [31:0] Rdata1,              // Data from source register 1 (rs1)
    input [31:0] Rdata2,              // Data from source register 2 (rs2)
    input [31:0] Imm,                 // Immediate value for instruction
    input [4:0] Rsrc1,                // Source register 1 (rs1)
    input [4:0] Rsrc2,                // Source register 2 (rs2)
    input [4:0] Rdst,                 // Destination register (rd)
    input [6:0] opcode,               // Opcode
    input [2:0] funct3,               // Function code (3 bits)
    input [6:0] funct7,               // Function code (7 bits) for R-type
    input branchTaken,                // Flag indicating if branch is taken
    input jumpTaken,
    //input [31:0] NPC_In,
    //input [31:0] branchAddr,          // Address for branching 
    output [31:0] Result,             // The final result of the executed instruction
    output [31:0] MemAddress          // Memory address to be written or read from
    //output [31:0] PC_next             // Next Program Counter (NPC)
    //output [31:0] JumpAddr            // Address to jump to for JAL/JALR
);

    wire [31:0] executionResult;

    // EU for computation
    ExecutionUnit execUnit (
        .out(executionResult),           
        .opA(Rdata1),                    
        .opB(Rdata2),                    
        .imm(Imm),                      
        .opcode(opcode),                
        .func(funct3),                
        .auxFunc(funct7),            
        .pc(PC_in)                                  
    );

    // load/store address calculation
    assign MemAddress = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? (Rdata1 + Imm) : 32'b0;

    // Logic for determining if the instruction is a jump or branch
    
    assign PC_next = (jumpTaken) ? (PC_in + Imm) :
                     (branchTaken) ? (PC_in + Imm) : // Use the branch address if branch is taken
                     (opcode == `OPCODE_JALR) ? ((Rdata1 + Imm) & ~32'h1) : // JALR Address calculation
                     (PC_in + 4); // Default increment by 4 for normal instructions
                      
    assign Result = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? MemAddress : // For load instructions, use data from memory
                    executionResult; // For all other operations, use execution result
    

endmodule

module Memory (
    input clk, 
    input rst,
    input [6:0] opcode,
    input [31:0] Imm, //should be ImmS or ImmI from Ex
    input [2:0] funct3,
    input [31:0] Rdata1,
    input [31:0] Rdata2,
    input [31:0] DataWord,
    input [31:0] MemAddress, //may not need - check 
    input [31:0] PC,

    output [1:0] MemSize,
    output MemWrEn,
    output [31:0] DataAddr,
    output [31:0] StoreData,
    output [31:0] loadData
);

    assign MemSize = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? funct3[1:0] : 2'b00; 
    assign MemWrEn = (opcode == `OPCODE_STORE);
    assign DataAddr = MemAddress; 


    //for reference - Mem Module writes the store to memory when MemWrEn == 1
    /*
       Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

    */
    
    assign StoreData = (opcode == `OPCODE_STORE && funct3 == `FUNC_SB)   ? {24'b0, Rdata2[7:0]} :   //LSB for sb
                      (opcode == `OPCODE_STORE && funct3 == `FUNC_SH)   ? {16'b0, Rdata2[15:0]} :  // Least significant 16 bits for sh
                      (opcode == `OPCODE_STORE && funct3 == `FUNC_SW)   ? Rdata2 :                 // Full word for sw
                      32'hXXXXXXXX; // Default 

    assign loadData = (opcode == `OPCODE_LOAD && funct3 == `FUNC_LB)  ? {{24{DataWord[7]}}, DataWord[7:0]}  : // Sign-extend byte
                     (opcode == `OPCODE_LOAD && funct3 == `FUNC_LH)  ? {{16{DataWord[15]}}, DataWord[15:0]} : // Sign-extend halfword
                     (opcode == `OPCODE_LOAD && funct3 == `FUNC_LW)  ? DataWord :                            // Load word
                     (opcode == `OPCODE_LOAD && funct3 == `FUNC_LBU) ? {{24{1'b0}}, DataWord[7:0]} :         // Zero-extend byte
                     (opcode == `OPCODE_LOAD && funct3 == `FUNC_LHU) ? {{16{1'b0}}, DataWord[15:0]} :        // Zero-extend halfword
                     32'hXXXXXXXX; // Default
    
endmodule 

module WriteBack(
    input clk,
    input rst,
    input [31:0] Result, //from EX - should give the result from the MemAddress calculation or the EU
    input [1:0] MemSize, // maybe remove
    input MemWrEn,          //maybe remove
    input [31:0] MemAddress, //maybe remove
    input [31:0] StoreData, //CHECK - idk if we need store
    input [31:0] loadData,
    input [6:0] opcode,
    input [4:0] Rsrc1,
    input [4:0] Rsrc2,
    input [4:0] Rdst,
    input RegWrite,

    output halt_overall, //later? 
    output [31:0] RWrdata,
    output RwrEn // for debugging - maybe just move to input

);
    //Reference: consolidate all this info in this module - this module is the one that writes to the registers in Lab 3
    /*
       RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	      .AddrB(Rsrc2), .DataOutB(Rdata2), 
	      .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

    */

    wire [31:0] final_Data; //for debugging I think?
    assign final_Data = (opcode == `OPCODE_LOAD) ? loadData : Result;

    assign RwrEn = RegWrite;
    assign RWrdata = final_Data;

endmodule

module ExecutionUnit(out, opA, opB, func, auxFunc, imm, opcode, pc);
   output [`WORD_WIDTH-1:0] out;
   input [`WORD_WIDTH-1:0]  opA, opB, imm, pc;
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
   wire [31:0] srl_result = (opA >> opB[4:0]);
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

                   //loads - to be handled during Mem
                   //(opcode == `OPCODE_LOAD) ? loadData : 

                   32'hXXXXXXXX;  // Default value in case no condition is met

      assign out = result; 


endmodule // ExecutionUnit