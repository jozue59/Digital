// Modulo para jalar la instruccion
module Inst_Memory(
  input clk, enable,
  input[12:0]address,
  output[13:0] out
);

  reg [13:0] out;
  reg [13:0] memory [0:8194];

  always @ (posedge clk)
// meto la instruccion
    if (enable) begin
      out <= memory[address];
    end
//leo la memoria
initial begin
 $readmemh("memory.list", memory);

end

endmodule
//Modulo para guardar en W
module register(
    input clk, enable,
    input [7:0] in,
    output [7:0] out
);

    reg [7:0] out;

    always @ (posedge clk)

      if(enable) begin
        out <= in;

      end

endmodule

//Modulo para jalar y guardar registros
module generalReg (
    input clk, input enable,
  	input [6:0] address,
    input [7:0] in,
    output [7:0] out,
    input [7:0] status,
    input [7:0] portA,
    output [7:0] portB
);
    wire [7:0] out;
  	reg [7:0] memory [0:128];

  	assign out = memory[address];




  	always @ (posedge clk) begin
        memory[3] <= status;
        memory[5] <= portA;
      	if(enable) begin
          if (address == 5) begin
            memory[5] <= portA;
          end else if (address == 6) begin
            memory[6] <= in;
          end else begin
            memory[address] <= in;
          end
      	end
  	end

  // reset memory
  	integer i;
  	initial begin
      for (i = 0; i < 128; i= i + 1)
        memory[i] = i;
    end

endmodule

//produzco los ciclos de reloj (cuadratura)
module half_Freq ( clk ,rst,out_clk );

input clk ;
input rst;
output reg out_clk;

always @(posedge clk) begin

  if (rst)
       out_clk <= 1'b0;

  else
       out_clk <= ~out_clk;

end

initial begin
    out_clk <= 1'b0;

end

endmodule

//Mux para literal y registro
module MUX (input sel, input [7:0] A, input [7:0] B, output [7:0] C);

  reg [7:0] C;

  always @ (A or B)begin

    case (sel)
      0: C <= A;
      1: C <= B;

    endcase

  end

endmodule

//Modulo para establecer un salto
module MUX2 (input zero, input [1:0] codigo, input [3:0]alu_Control, output flag);
  reg flag;
  always @ (alu_Control)begin
    if (codigo == 2'b00 & zero == 1'b1)begin
      if (alu_Control == 4'b1111 | alu_Control == 4'b1011)begin
        flag <= 1'b1;
      end
    end else if ( codigo == 2'b01 & zero == 1'b1)begin
      if (alu_Control == 4'b0110 | alu_Control == 4'b0111)begin
        flag <= 1'b1;
      end
    end else begin
      flag <= 1'b0;
    end
  end

endmodule // MUX2
