/*
  
  Northwestern University
  CompEng361 - Fall 2024
  Lab 2 Testbench
 
  DO NOT TURN THIS FILE IN!!!!
  
  */

`define WIDTH 32
`define FWIDTH 3
`define AFWIDTH 7
`define STEP 10

module testbench();
   reg [`WIDTH-1:0] opA;
   reg [`WIDTH-1:0] opB;
   wire [`WIDTH-1:0] result;

   reg [`FWIDTH-1:0]   func;
   reg [`AFWIDTH-1:0] 	    auxFunc;

   ExecutionUnit EU( .out(result),
		     .opA(opA),
		     .opB(opB),
		     .func(func),
		     .auxFunc(auxFunc));

   initial begin
      $monitor($time,, "%03b %07b %08x %08x %08x", func, auxFunc, opA, opB, result);
	       
      doExecute(3'h0, 7'h0, 32'h1, 32'h2); // Here's a basic test

      // Place your tests here

      $finish;
   end

   task doExecute(input [`FWIDTH-1:0] f, 
		  input [`AFWIDTH-1:0] af, 
		  input [`WIDTH-1:0] a, 
		  input [`WIDTH-1:0] b); // set inputs for execute unit
      begin
         func = f; auxFunc = af; opA = a; opB = b;
	 #`STEP;
      end
   endtask

   
endmodule // testbench

