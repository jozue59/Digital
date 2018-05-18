module Counter(
    input clk, reset,
  output [12:0] out,
  input [10:0] salto,
  input enablestak,
  input branch,
  input flag
);

  reg [12:0]out;
  reg [12:0] out2;
  reg [1:0] complemento;
  assign complemento = 2'b0;

  	always @ (posedge clk)

      if(reset) begin
      	out <= 0;

      end else begin
        if (branch == 1'b1)begin
          out<= {{complemento[1:0]},{salto[10:0]}};
        end else if (flag == 1'b1 )begin
          out<= out +2;
        end else begin
            out <= out + 1;
        end

  	  end

initial begin
    out <= 1'b0;

end

endmodule


module Inst_Memory(
  input clk, enable,
  input[12:0]address,
  output[13:0] out
);

  reg [13:0] out;
  reg [13:0] memory [0:8194];

  always @ (posedge clk)

    if (enable) begin
      out <= memory[address];
    end

initial begin
 $readmemh("memory.list", memory);

end

endmodule

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

module generalReg (
    input clk, input enable,
  	input [6:0] address,
    input [7:0] in,
    output [7:0] out
);
    wire [7:0] out;
  	reg [7:0] memory [0:128];

  	assign out = memory[address];

  	always @ (posedge clk) begin
      	if(enable) begin
          memory[address] <= in;
      	end
  	end

  // reset memory
  	integer i;
  	initial begin
      for (i = 0; i < 128; i= i + 1)
        memory[i] = i;
    end

endmodule

`define registros 2'b00
`define literales 2'b11

module ALU(
  input clk,
  input [3:0] control,
  input [7:0] A,B,
  output [7:0] out,
  input [1:0] codigo,
  output zero
);

  reg [7:0] out;
  reg zero;

  wire [7:0] result1;
  wire [7:0] result2;
  wire RLF;
  wire RRF;

  assign RLF = B[7];
  assign result1 = B<<1;
  assign result1[0]=RLF;
  assign RRF = B[0];
  assign result2 = B>>1;
  assign result2[7] = RRF;
  assign zero = (out==0);

  always @ (posedge clk)begin

    if (codigo ==`registros)begin

      case (control)
        0:out <= A;
        1:out <= 0;
        2:out <= B - A;
        3:out <= B - 1;
        4:out <= A | B;
        5:out <= A & B;
        6:out <= A ^ B;
        7:out <= A + B;
        8:out <= B;
        9:out <= -B;
        10:out <= B + 1;
        11:out <= B - 1;//salto DECFSZ
        12:out <= result1;//B<<;
        13:out <= result2;//B>>;
        14:out <= {{B[3:0]},{B[7:4]}}; //swap
        15:out <= B+1; // INCFSZ

      endcase

    end else if (codigo == `literales)begin

      casex (control)
        4'b111x: out <= A + B;
        4'b1001: out <= A & B;
        4'b1000: out <= A | B;
        4'b00xx: out <= B;
        4'b01xx: out <= B; //RETLW
        4'b110x: out <= B - A;
        4'b1010: out <= A ^ B;

      endcase

    end else if (codigo == 2'b10)begin
      out <= A;
    end

  end

endmodule

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

`define codigo 2'b00
`define destino 2'b11
`define jump 2'b10

module Decoder ( input [1:0] codigo,
  input [13:0] instruction,
  output [3:0] alu_Control,
  output enable_REG,
  output enable_W,
  output enableRAM,
  output sel,
  output [10:0] salto,
  output enablestak,
  output branch
);

  reg [3:0] alu_Control;
  reg [10:0] salto;
  reg enablestak;
  reg enable_REG;
  reg enableRAM;
  reg sel;
  reg branch;

  always @ (instruction) begin

    if (codigo == `codigo)begin
      alu_Control <= instruction [11:8];
      enable_REG <= instruction[7];
      enableRAM <= 1'b1;
      sel <= 1'b0;
      branch <= 1'b0;

    end else if ( codigo == `destino) begin
      enableRAM <= 1'b0;
      sel <= 1'b1;
      alu_Control <= instruction [11:8];
      enable_REG <=1'b0;
      branch <= 1'b0;

    end else if (codigo == `jump) begin
      enableRAM<=1'b0;
      salto <=instruction [10:0];
      enablestak <= instruction [11];
      enable_REG<=1'b0;
      branch <= 1'b1;

    end

  end

endmodule

module MUX (input sel, input [7:0] A, input [7:0] B, output [7:0] C);

  reg [7:0] C;

  always @ (A or B)begin

    case (sel)
      0: C <= A;
      1: C <= B;

    endcase

  end

endmodule

module MUX2 (input zero, input [1:0] codigo, input [3:0]alu_Control, output flag);
  reg flag;
  always @ (alu_Control)begin
    if (codigo == 2'b00 & zero == 1'b1)begin
      if (alu_Control == 4'b1111 | alu_Control == 4'b1011)begin
        flag <= 1'b1;
      end
    end else begin
      flag <= 1'b0;
    end
  end

endmodule // MUX2

module Stack (
	clk,
	reset,
	q,
	d,
	push,
	pop,
);

	parameter WIDTH = 13;
	parameter DEPTH = 8;

	input                    clk;
	input                    reset;
	input      [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	input                    push;
	input                    pop;

	reg [DEPTH - 1:0] ptr;
	reg [WIDTH - 1:0] stack [0:(1 << DEPTH) - 1];

	always @(posedge clk) begin
		if (reset)
			ptr <= 0;
		else if (push)
			ptr <= ptr + 1;
		else if (pop)
			ptr <= ptr - 1;
	end

	always @(posedge clk) begin
		if (push || pop) begin
			if(push)
				stack[ptr] <= q;

			q <= stack[ptr - 1];
		end
	end

endmodule
