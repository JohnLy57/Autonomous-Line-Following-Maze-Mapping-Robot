`define SCREEN_WIDTH 270
`define SCREEN_HEIGHT 270
//`define NUM_BARS 3
//`define BAR_HEIGHT 48

`define BLACK 4'b0
`define WHITE 4'b1
`define RED	  4'b010


module IMAGE_PROCESSOR (
	ARDUINO_IN,
	PIXEL_OUT,
	CLK,
	VGA_PIXEL_X,
	VGA_PIXEL_Y
);

//=======================================================
//  PORT declarations
//=======================================================
input wire CLK;
input wire [11:0] ARDUINO_IN;
output reg [9:0] VGA_PIXEL_X = 10'd0;
output reg [9:0] VGA_PIXEL_Y = 10'd0;

output reg	[3:0]	PIXEL_OUT;

wire POS_X;
wire POS_Y;
wire WALLS;
assign POS_X = ARDUINO_IN[3:0];
assign POS_Y = ARDUINO_IN[7:4];
assign WALLS = ARDUINO_IN[11:8];

reg update = 1'b0;
reg reset = 1'b0;
reg init = 1'b1;

reg [3:0] xpos = 4'd0;
reg [3:0] ypos = 4'd0;


always @ (posedge CLK) begin
	if (init == 1'b1) begin
		if(VGA_PIXEL_X<`SCREEN_WIDTH) begin
			VGA_PIXEL_X <= VGA_PIXEL_X+10'd1;
		end
		else if(VGA_PIXEL_Y<`SCREEN_HEIGHT) begin
			VGA_PIXEL_X <= 10'd0;
			VGA_PIXEL_Y <= VGA_PIXEL_Y + 1;
		end
		else begin
			VGA_PIXEL_X <= 10'd0;
			VGA_PIXEL_Y <= 10'd0;
			//init <= 1'b0;
		end
	
		
		if((VGA_PIXEL_X+1) % 30 == 0 || (VGA_PIXEL_Y+1) % 30 == 0) begin
			PIXEL_OUT <= `WHITE;
		end
		else begin 
			if(ARDUINO_IN[0]) begin
				PIXEL_OUT <= `RED;
			end
			else begin
				PIXEL_OUT <= 4'b011;
			end
		end
	end
	
//	else if (update) begin
//		
//		if(ARDUINO_IN[11]) begin //North wall
//		end
//		
//		if(ARDUINO_IN[10])begin //East wall
//		end
//		
//		if(ARDUINO_IN[9])begin //South wall
//		end
//		
//		if(ARDUINO_IN[8])begin //West wall
//		end
//			
//			
//			//4'b1000 //North
//			//4'b0100 //East
//			//4'b0010 //South
//			//4'b0001 //West
//		
//		
//		reset <= 1;
//	end
	
end
//
//always@(ARDUINO_IN, reset) begin
//	if(reset) begin
//		update <= 0;
//	end
//	else begin
//		update <= 1;
//	end
//	
//end

endmodule

