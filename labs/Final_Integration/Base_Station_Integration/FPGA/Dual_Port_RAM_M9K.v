`define SCREEN_WIDTH 300
`define SCREEN_HEIGHT 300

module Dual_Port_RAM_M9K(
	input_data,
	w_addr,
	r_addr,
	w_en,
	clk_W,
	clk_R,
	output_data
);

//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCKS //////////
input 		          		clk_W, clk_R;

//////////// input enables (write, read) //////////
input 		    				w_en;

//////////// write and read addresses //////////
input 		    [16:0]		w_addr;
input 		    [16:0]		r_addr;

//////////// input/output data busses //////////
input 		    [3:0]		input_data;
output reg 		 [3:0]		output_data;

//////////// memory array //////////
(* ramstyle = "M9K" *) reg	[3:0]	 mem [(`SCREEN_WIDTH * `SCREEN_HEIGHT)-1:0];
reg				 [16:0]		r_addr_reg;


///// WRITE BLOCK /////
always @ (posedge clk_W) begin
	if(w_en) 
		mem[w_addr] <= input_data;
end

///// READ BLOCK  /////
always @ (posedge clk_R) begin
	output_data <= mem[r_addr];
	r_addr_reg <= r_addr;
end

endmodule