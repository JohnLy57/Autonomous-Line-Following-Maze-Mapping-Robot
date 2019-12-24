`define SCREEN_WIDTH 300
`define SCREEN_HEIGHT 300

///////* DON'T CHANGE THIS PART *///////
module DE0_NANO(
	CLOCK_50,
	GPIO_0_D,
	GPIO_1_D,
	KEY
);


//=======================================================
//  PARAMETER declarations
//=======================================================

localparam RED = 8'b111_000_00;
localparam GREEN = 8'b000_111_00;
localparam BLUE = 8'b000_000_11;
localparam WHITE = 8'b111_111_11;
localparam BLACK = 8'b000_000_00;

//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK - DON'T NEED TO CHANGE THIS //////////
input 		          		CLOCK_50;

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
output 		    [33:0]		GPIO_0_D;
//////////// GPIO_0, GPIO_1 connect to GPIO Default //////////
input 		    [33:0]		GPIO_1_D;
input 		     [1:0]		KEY;

///// PIXEL DATA /////
wire [3:0] pixel_data_RGB332;

///// READ/WRITE ADDRESS /////
wire [9:0] X_ADDR;
wire [9:0] Y_ADDR;
wire [16:0] WRITE_ADDRESS;
reg [16:0] READ_ADDRESS; 


///// VGA INPUTS/OUTPUTS /////
wire 			VGA_RESET;
wire [7:0]	VGA_COLOR_IN;
wire [9:0]	VGA_PIXEL_X;
wire [9:0]	VGA_PIXEL_Y;
wire [7:0]	MEM_OUTPUT;
wire			VGA_VSYNC_NEG;
wire			VGA_HSYNC_NEG;
reg			VGA_READ_MEM_EN;

assign GPIO_0_D[5] = VGA_VSYNC_NEG;
assign VGA_RESET = ~KEY[0];

/* WRITE ENABLE */
reg W_EN;

///////* CREATE ANY LOCAL WIRES YOU NEED FOR YOUR PLL *///////
wire PLL_24;
wire PLL_25;
wire PLL_50;

///////* INSTANTIATE YOUR PLL HERE *///////
PLL	pll_inst (
	.inclk0 ( CLOCK_50 ),
	.c0 ( PLL_24 ),
	.c1 ( PLL_25 ),
	.c2 ( PLL_50 )
);

///////* M9K Module *///////
Dual_Port_RAM_M9K mem(
	.input_data(pixel_data_RGB332),
	.w_addr(WRITE_ADDRESS),
	.r_addr(READ_ADDRESS),
	.w_en(W_EN),
	.clk_W(PLL_50),
	.clk_R(PLL_25), // DO WE NEED TO READ SLOWER THAN WRITE??
	.output_data(MEM_OUTPUT)
);

wire [7:0] COLOR;
assign COLOR = (MEM_OUTPUT==4'b0) ? BLACK : ((MEM_OUTPUT==4'b0001) ? WHITE : ((MEM_OUTPUT==4'b0010) ? RED : GREEN));

///////* VGA Module *///////
VGA_DRIVER driver (
	.RESET(VGA_RESET),
	.CLOCK(PLL_25),
	.PIXEL_COLOR_IN(VGA_READ_MEM_EN ? COLOR : BLACK),
	.PIXEL_X(VGA_PIXEL_X),
	.PIXEL_Y(VGA_PIXEL_Y),
	.PIXEL_COLOR_OUT({GPIO_0_D[9],GPIO_0_D[11],GPIO_0_D[13],GPIO_0_D[15],GPIO_0_D[17],GPIO_0_D[19],GPIO_0_D[21],GPIO_0_D[23]}),
   .H_SYNC_NEG(GPIO_0_D[7]),
   .V_SYNC_NEG(VGA_VSYNC_NEG)
);

///////* Image Processor *///////
IMAGE_PROCESSOR proc(
	.ARDUINO_IN({GPIO_1_D[23],GPIO_1_D[21],GPIO_1_D[19],GPIO_1_D[17],GPIO_1_D[15],GPIO_1_D[13],GPIO_1_D[11],GPIO_1_D[9],GPIO_1_D[7],GPIO_1_D[5],GPIO_1_D[3],GPIO_1_D[1],GPIO_1_D[0]}),
	.PIXEL_OUT(pixel_data_RGB332),
	.CLK(PLL_50),
	.VGA_PIXEL_X(X_ADDR),
	.VGA_PIXEL_Y(Y_ADDR)
);


///////* Update Read Address *///////
always @ (VGA_PIXEL_X, VGA_PIXEL_Y) begin
		READ_ADDRESS = (VGA_PIXEL_X + VGA_PIXEL_Y*`SCREEN_WIDTH);
		if(VGA_PIXEL_X>(`SCREEN_WIDTH-1) || VGA_PIXEL_Y>(`SCREEN_HEIGHT-1))begin
				VGA_READ_MEM_EN = 1'b0;
		end
		else begin
				VGA_READ_MEM_EN = 1'b1;
		end
end


assign WRITE_ADDRESS = X_ADDR + Y_ADDR*(`SCREEN_WIDTH);

always @(posedge PLL_50) begin //Make sure that we only write within screen boundaries
		if(X_ADDR>(`SCREEN_WIDTH-1) || Y_ADDR>(`SCREEN_HEIGHT-1)) begin
				W_EN = 1'b0;
		end
		else begin
				W_EN = 1'b1;
		end

end
endmodule 