
//PC, saltos y stack
module Counter(
    input clk, reset,
  output [12:0] out,
  input [10:0] salto,
  input enablestak,
  input push,
  input flag,
  input pop,
  input branch
);

//posicion de PC
  reg [12:0]out;

//MSB para PC en los saltos
  reg [1:0] complemento;
  assign complemento = 2'b00;

//
  parameter WIDTH = 13;
	parameter DEPTH = 3;
  reg [DEPTH - 1:0] ptr;
	reg [WIDTH - 1:0] stack [0:(1 << DEPTH) - 1];
  //

  always @(posedge clk) begin
		if (reset)
			ptr <= 0;
		else if (push)
			ptr <= ptr + 1;
		else if (pop)
			ptr <= ptr - 1;
	end

  	always @ (posedge clk)

      if(reset) begin
      	out <= 0;
      end else begin
        if (branch == 1'b1) begin
          if (enablestak == 1'b1)begin
            out<= {{complemento[1:0]},{salto[10:0]}};
          end else if (enablestak == 1'b0)begin
            if (push || pop) begin
        			if(push)begin
                stack[ptr] <= out + 1;
                out<= {{complemento[1:0]},{salto[10:0]}};
              end
        			out <= stack[ptr - 1];
        		end
          end

        end else if (flag == 1'b1 )begin
          out<= out +2;

        end else begin
            out <= out + 1;
        end

  	  end

initial begin
    out <= 1'b0;
    ptr <= 1'b0;

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
  output zero,
  output carry,
  output Dcarry,
  input [2:0] Numero_bit
);

  reg [7:0] out, out1, out2, ceros, unos;
  reg zero, prueba, carry, Dcarry;
  reg [8:0] suma9, resta9;
  reg [8:0] A1, B1;
  reg [7:0] set;
  assign A1 = {{1'b0},{A[7:0]}};
  assign B1 = {{1'b0},{B[7:0]}};

  assign zero = (out==0);
  assign suma9 = A1 + B1;
  //assign carry = suma9[8];
  assign resta9 = B1 - A1;



  always @ (A or B or Numero_bit) begin
    if (control == 4'b0100)begin
      case (Numero_bit)
        0: set <= 8'b00000001;
        1: set <= 8'b00000010;
        2: set <= 8'b00000100;
        3: set <= 8'b00001000;
        4: set <= 8'b00010000;
        5: set <= 8'b00100000;
        6: set <= 8'b01000000;
        7: set <= 8'b10000000;
      endcase
    end else if (control == 4'b0101)begin
      case (Numero_bit)
        0: set <= 8'b11111110;
        1: set <= 8'b11111101;
        2: set <= 8'b11111011;
        3: set <= 8'b11110111;
        4: set <= 8'b11101111;
        5: set <= 8'b11011111;
        6: set <= 8'b10111111;
        7: set <= 8'b01111111;
      endcase
    end
    ceros <= B | set;
    unos <= B & set;

  end

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
        12:out <= {{B[0]},{B[7:1]}};//B>>;
        13:out <= {{B[6:0]},{B[7]}};//B>>;
        14:out <= {{B[3:0]},{B[7:4]}}; //swap
        15:out <= B+1; // INCFSZ
      endcase
      if (control == 4'b0111)begin
        carry <= suma9[8];
        Dcarry <= suma9[8];
      end else if (control== 4'b1101) begin
        carry <= B[7];
      end else if (control == 4'b1100) begin
        carry <= B[0];
      end else if (control == 4'b0010) begin
        carry <= resta9[8];
        Dcarry <= resta9[8];
      end

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
      if (control[3:1] == 3'b111) begin
        carry <= suma9[8];
        Dcarry <= suma9[8];
      end else if (control[3:1] == 3'b110)begin
        carry <= resta9[8];
        Dcarry <= resta9[8];
      end


    end else if (codigo == 2'b01)begin
      case (control)
        4'b0100: out<= ceros;
        4'b0101: out<= unos;
        4'b0110: prueba<= (B[Numero_bit] == 1'b1) ;
        4'b0111: prueba<= (B[Numero_bit] == 1'b0) ;
      endcase
      if (prueba == 1'b0 & (control == 4'b0110 | control == 4'b0111))begin
        out <= 8'b00000000;
      end
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
  output push,
  output [2:0] Numero_bit,
  output pop,
  output branch
);

  reg [3:0] alu_Control;
  reg [10:0] salto;
  reg enablestak;
  reg enable_REG;
  reg enableRAM;
  reg sel, push, pop;
  reg [2:0] Numero_bit;
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
      if (alu_Control[3:2] == 2'b01) begin
        branch <=1'b1;
        enablestak <= 1'b0;
        push <= 1'b0;
        pop <=1'b1;
      end else begin
        branch <= 1'b0;
      end

    end else if (codigo == `jump) begin
      enableRAM<=1'b0;
      salto <=instruction [10:0];
      enablestak <= instruction [11];
      enable_REG<=1'b0;
      push <= 1'b1;
      pop <= 1'b0;
      branch<=1'b1;
    end else if (codigo == 2'b01) begin
      enableRAM <=1'b1;
      alu_Control<= instruction [13:10];
      if (alu_Control == 4'b0100 | alu_Control== 4'b0101)begin
        enable_REG<= 1'b1;
      end else begin
        enable_REG<= 1'b0;
      end
      Numero_bit <= instruction [9:7];
      sel <= 1'b0;
    end else if (instruction == 14'b00000000001000) begin
      enablestak <= 1'b1;
      push <= 1'b0;
      pop <=1'b1;
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
    end else if ( codigo == 2'b01 & zero == 1'b1)begin
      if (alu_Control == 4'b0110 | alu_Control == 4'b0111)begin
        flag <= 1'b1;
      end
    end else begin
      flag <= 1'b0;
    end
  end

endmodule // MUX2
