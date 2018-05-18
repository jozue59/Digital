
module test;
  reg clk, reset;
  wire [12:0] count_PC;
  wire [13:0] instruction;
  wire [3:0] alu_Control;
  wire [7:0] alu_A;
  wire [7:0] bus;
  wire [7:0] alu_B;
  wire [6:0] addr_REG;
  wire enable_REG, enable_W, vdd, sel;
  wire clk_n, clk2, clk2_n, clk3;
  wire [1:0] codigo;
  wire instFetch, dataFetch, aluResults, saveFiles, controlReg, enableRAM;
  wire [7:0] literal;
  wire [7:0] alu_C;

  assign vdd = 1;
  assign addr_REG = instruction[7:0];
  assign codigo = instruction[13:12];
  assign literal = instruction [7:0];

  Counter 		  PC 			      (clk2_n,	reset,	count_PC);
  Inst_Memory 	Instruction	  (instFetch, vdd, count_PC, instruction );
  Decoder	      Master        (codigo, instruction, alu_Control, enable_REG, enable_W, enableRAM, sel);
  ALU 			    ALU1 		      (aluResults, alu_Control, alu_A, alu_C, bus,codigo);
  register 		  W_REG 		    (saveFiles, enable_W, bus, alu_A);
  generalReg 	  F_REG	 	      (saveFiles, enable_REG, addr_REG, bus, alu_B);
  MUX           MUX1          (sel, alu_B, literal, alu_C);

  not			not1		(enable_W, enable_REG); // 0 = W, 1 = F

  not			not2		(clk_n,clk);
  not			not2		(clk2_n,clk2);

  half_Freq		clock2		(clk, reset, clk2);
  half_Freq		clock3		(clk2, reset, clk3);

  and			and0	(instFetch, clk, 	clk2);
  and			and1	(dataFetch, clk_n, 	clk2);
  and			and2	(aluResults,clk, 	clk2_n);
  and			and3	(saveFiles, clk_n, 	clk2_n);



initial begin
  clk = 0;
  reset = 1;
  #1 clk = 1;
  #1 clk = 0;
  #1 clk = 1;
  #1 clk = 0;
  reset = 0;
end

initial
  #500 $finish;

initial begin
  $display ("PC \tInst \tW ");
  $monitor("%d \t%h\t%d", count_PC, instruction,alu_A);
end

  always
    #5	clk = !clk;

endmodule
`include "pruebas1.sv"
