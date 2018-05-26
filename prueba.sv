// Universidad del valle de Guatemala
// Electronica digital 1, seccion 20
// Josue David del cid Ramirez, 16048
// Samuel Alexander Hernandez, #carnet

module test;
//Variables dependientes del tiempo
  reg clk, reset;
  wire clk_n, clk2, clk2_n, clk3;
  wire [12:0] count_PC;
  wire instFetch, dataFetch, aluResults, saveFiles, controlReg;

//Variables que dependen de la instruccion
  wire [13:0] instruction;
  wire enable_REG, enable_W, enablestak, enableRAM, branch;
  wire vdd, sel,push, pop, zero, flag, carry, Dcarry;
  wire [1:0] codigo;
  wire [6:0] addr_REG;
  wire [2:0] Numero_bit;

//Variables que dependen de una operacion arimetica
  wire [3:0] alu_Control;
  wire [7:0] literal;
  wire [7:0] alu_C;
  wire [7:0] alu_A;
  wire [7:0] bus;
  wire [7:0] alu_B;
  wire [10:0] salto;

//Registros especiales
  wire [7:0] portA;
  wire [7:0] portB;
  wire [7:0] Status;

// Distribucion parcial de la instruccion
  assign vdd = 1;
  assign addr_REG = instruction[6:0];
  assign codigo = instruction[13:12];
  assign literal = instruction [7:0];
  assign Status [2] = zero;
  assign Status [0] = carry;
  assign Status [1] = Dcarry;
  assign portA [7:0] = 8'b001001;

//Modulos principales

  Counter 		  PC 			      (clk2_n,	reset,	count_PC, salto, enablestak, push, flag, pop, branch);
  Inst_Memory 	Instruction	  (instFetch, vdd, count_PC, instruction );
  Decoder	      Master        (codigo, instruction, alu_Control, enable_REG, enable_W, enableRAM, sel, salto, enablestak, push, Numero_bit, pop, branch);
  ALU 			    ALU1 		      (aluResults, alu_Control, alu_A, alu_C, bus, codigo, zero, carry, Dcarry, Numero_bit);
  register 		  W_REG 		    (saveFiles, enable_W, bus, alu_A);
  generalReg 	  F_REG	 	      (saveFiles, enable_REG, addr_REG, bus, alu_B, Status, portA, portB);
  MUX           Literal_o_F   (sel, alu_B, literal, alu_C);
  MUX2          Saltos        (zero, codigo, alu_Control, flag);

  not			not1		(enable_W, enable_REG); // 0 = W, 1 = F

//Cambio de flanco
  not			not2		(clk_n,clk);
  not			not2		(clk2_n,clk2);

//Generador de clocks basado en el principal
  half_Freq		clock2		(clk, reset, clk2);
  half_Freq		clock3		(clk2, reset, clk3);


//Pulsos de reloj
  and			and0	(instFetch, clk, 	clk2);
  and			and1	(dataFetch, clk_n, 	clk2);
  and			and2	(aluResults,clk, 	clk2_n);
  and			and3	(saveFiles, clk_n, 	clk2_n);

//Valores iniciales
initial begin
  clk = 0;
  reset = 1;
  #1 clk = 1;
  #1 clk = 0;
  #1 clk = 1;
  #1 clk = 0;
  reset = 0;
end

// Tiempo en que se ejecuta el programa
initial
  #200 $finish;

// Despliegue en consola
initial begin
  $display ("PC \tInst \tW \tpuetoA  \tpuertoB ");
  $monitor("%d \t%h \t%d \t%d \t%d ", count_PC, instruction,alu_A, alu_C, zero);
end

//T/2 del clock general
  always
    #5	clk = !clk;

endmodule

//Enlace con otros archivos
`include "pruebas1.sv"
`include "alu.sv"
`include "decoder.sv"
`include "PC.sv"
