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

  reg [12:0]out;
  reg [12:0] out2;
  reg [1:0] complemento;
  assign complemento = 2'b0;

  //
  parameter WIDTH = 13;
	parameter DEPTH = 3;
  reg [DEPTH - 1:0] ptr;
	reg [WIDTH - 1:0] stack [0:(1 << DEPTH) - 1];
  //

  always @(posedge clk) begin
		if (reset)
			ptr <= 0;
		//else if (push)
			//ptr <= ptr + 1;
		//else if (pop)
			//ptr <= ptr - 1;
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
        			if(push == 1'b1)begin
                stack[ptr] <= out  + 1'b1;
                out<= {{complemento[1:0]},{salto[10:0]}};
                ptr<= ptr + 1;

              end else begin
                out <= stack[ptr-1];
                ptr<= ptr -1;
              end
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
