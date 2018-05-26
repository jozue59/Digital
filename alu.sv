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

//Variables para operaciones
  reg [7:0] out, ceros, unos;
  reg [8:0] suma9, resta9;
  reg zero, prueba, carry, Dcarry;
  reg [8:0] A1, B1;
  reg [7:0] set;

//Expando mis entradas a 9 bits
  assign A1 = {{1'b0},{A[7:0]}};
  assign B1 = {{1'b0},{B[7:0]}};

//Bandera cero, si el resultados es 0 zero=1
  assign zero = (out==0);

//operaciones con las entradas expandidas
  assign suma9 = A1 + B1;
  assign resta9 = B1 - A1;


//Bloque para operandos de bit
  always @ (A or B or Numero_bit) begin
    if (control == 4'b0100)begin
      //Valor de set para BSF
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
      //Valor de set para BCF
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
    // Realiza operacion de bit
    ceros <= B & set;//BCF
    unos <= B | set;//BSF

  end

  always @ (posedge clk)begin

    if (codigo ==`registros)begin

      case (control)
        0:out <= A;//MOVF
        1:out <= 0;//CLEAR
        2:out <= B - A;//SUBWF
        3:out <= B - 1;//DECF
        4:out <= A | B;//ORWF
        5:out <= A & B;//ANDWF
        6:out <= A ^ B;//XOR
        7:out <= A + B;//ADDWF
        8:out <= B;//MOVWF
        9:out <= -B;//COMF
        10:out <= B + 1;//incf
        11:out <= B - 1;//salto DECFSZ
        12:out <= {{B[0]},{B[7:1]}};//B>>;
        13:out <= {{B[6:0]},{B[7]}};//B>>;
        14:out <= {{B[3:0]},{B[7:4]}}; //swap
        15:out <= B+1; // INCFSZ
      endcase
      //BANDERAS DE CARRY
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
      //OPERACIONES DE LITERALES
      casex (control)
        4'b111x: out <= A + B;//ADDLW
        4'b1001: out <= A & B;//ANDLW
        4'b1000: out <= A | B;//ORLW
        4'b00xx: out <= B;//MOVLW
        4'b01xx: out <= B; //RETLW
        4'b110x: out <= B - A;//SUBLW
        4'b1010: out <= A ^ B;//XORLW
      endcase
      //BANDERAS DE CARRY
      if (control[3:1] == 3'b111) begin
        carry <= suma9[8];
        Dcarry <= suma9[4];
      end else if (control[3:1] == 3'b110)begin
        carry <= resta9[8];
        Dcarry <= resta9[4];
      end

      //OPERACIONES DE BIT
    end else if (codigo == 2'b01)begin
      case (control)
        4'b0100: out<= ceros;//BCF
        4'b0101: out<= unos;//BSF
        4'b0110: prueba<= (B[Numero_bit] == 1'b1) ;//BTFSC
        4'b0111: prueba<= (B[Numero_bit] == 1'b0) ;//BTFSS
      endcase
      //SALIDAS PARA LOS BTFXX
      if (prueba == 1'b0 & (control == 4'b0110 | control == 4'b0111))begin
        out <= 8'b00000000;
      end
    end

  end

endmodule
