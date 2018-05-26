//DEFINO VARIABLES PARA EL MODULO
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

//ESTABLESCO LAS SALIDAS COMO REG
  reg [3:0] alu_Control;
  reg [10:0] salto;
  reg enablestak;
  reg enable_REG;
  reg enableRAM;
  reg sel, push, pop;
  reg [2:0] Numero_bit;
  reg branch;

  always @ (instruction) begin

    //DECODIFICACION PARA OPERACIONES DE BYTE
    if (codigo == `codigo)begin
      alu_Control <= instruction [11:8];
      enable_REG <= instruction[7];
      enableRAM <= 1'b1;
      sel <= 1'b0;
      branch <= 1'b0;
      //INSTRUCCION RETURN DECODIFICADA
      if (instruction == 14'b00000000001000) begin
        enablestak <= 1'b0;
        push <= 1'b0;
        pop <=1'b1;
        enable_REG<=1'b0;
        branch <= 1'b1;
        alu_Control<=0;
      end
    //DECODIFICACION PARA OPERACIONES DE LITERAL
    end else if ( codigo == `destino) begin
      enableRAM <= 1'b0;
      sel <= 1'b1;
      alu_Control <= instruction [11:8];
      enable_REG <=1'b0;
      //DECODIFICACION PARA INSTRUCCION RETLW
      if (alu_Control[3:2] == 2'b01) begin
        branch <=1'b1;
        enablestak <= 1'b0;
        push <= 1'b0;
        pop <=1'b1;
      end else begin
        branch <= 1'b0;
      end

      //DECODIFICACION PARA SALTOS (GOTO, CALL)
    end else if (codigo == `jump) begin
      enableRAM<=1'b0;
      salto <=instruction [10:0];
      enablestak <= instruction [11];
      enable_REG<=1'b0;
      push <= 1'b1;
      pop <= 1'b0;
      branch<=1'b1;

      //DECODIFICACION PARA INSTRUCCIONES DE BIT
    end else if (codigo == 2'b01) begin
      enableRAM <=1'b1;
      alu_Control<= instruction [13:10];
      //ES UNA PRUEBA DE BIT?
      if (alu_Control == 4'b0100 | alu_Control== 4'b0101)begin
        enable_REG<= 1'b1;
      end else begin
        enable_REG<= 1'b0;
      end
      Numero_bit <= instruction [9:7];
      sel <= 1'b0;
    end

  end

endmodule
