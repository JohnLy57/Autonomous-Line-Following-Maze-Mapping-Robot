`define SCREEN_WIDTH 300
`define SCREEN_HEIGHT 300
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
input wire [12:0] ARDUINO_IN;
output reg [9:0] VGA_PIXEL_X = 10'd0;
output reg [9:0] VGA_PIXEL_Y = 10'd0;

output reg	[3:0]	PIXEL_OUT;

wire [3:0] POS_X;
wire [3:0] POS_Y;
wire [3:0] WALLS;
wire UPDATE;
assign POS_X = ARDUINO_IN[3:0];
assign POS_Y = ARDUINO_IN[7:4];
assign WALLS = ARDUINO_IN[11:8];
assign UPDATE = ARDUINO_IN[12];

reg update = 1'b0;
reg updating = 1'b0;
reg prev_update = 1'b0;

reg init = 1'b1;

reg [9:0] xpos = 10'd0;
reg [9:0] ypos = 10'd0;


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
			init <= 1'b0;
		end
	
		
		if((VGA_PIXEL_X+1) % 30 == 0 || (VGA_PIXEL_Y+1) % 30 == 0) begin
			PIXEL_OUT <= `WHITE;
		end
		else begin 
			PIXEL_OUT <= `BLACK;
		end
	end
	else begin	
				//4'b1000 //North
				//4'b0100 //East
				//4'b0010 //South
				//4'b0001 //West
		
		if(UPDATE)begin
			update <= 1;
		end 
		else begin
			update <= 0;
		end
		
		if(update && !prev_update) begin
			updating <= 1;
		end
		
		if(updating) begin //wall writing code here
			case(WALLS)
				//4'b1000 //North
				4'b1000:	begin
					VGA_PIXEL_Y <= (POS_Y+1)*30-15;
					if(xpos==10'd0) begin	
						VGA_PIXEL_X <= (POS_X+1)*30-15;
						xpos <= (POS_X+1)*30+15;
					end
					else if(VGA_PIXEL_X<xpos) begin
						VGA_PIXEL_X <= VGA_PIXEL_X+1;
					end
					else begin
						xpos <=10'd0;
						updating <= 0;
					end
				end
				//4'b0100 //East
				4'b0100: begin
					VGA_PIXEL_X <= (POS_X+1)*30+15;
					if(ypos==10'd0) begin	
						VGA_PIXEL_Y <= (POS_Y+1)*30-15;
						ypos <= (POS_Y+1)*30+15;
					end
					else if(VGA_PIXEL_Y<ypos) begin
						VGA_PIXEL_Y <= VGA_PIXEL_Y+1;
					end
					else begin
						ypos <=10'd0;
						updating <= 0;
					end
				end
				
				//4'b0010 //South
				4'b0010: begin
					VGA_PIXEL_Y <= (POS_Y+1)*30+15;
					if(xpos==10'd0) begin	
						VGA_PIXEL_X <= (POS_X+1)*30-15;
						xpos <= (POS_X+1)*30+15;
					end
					else if(VGA_PIXEL_X<xpos) begin
						VGA_PIXEL_X <= VGA_PIXEL_X+1;
					end
					else begin
						xpos <=10'd0;
						updating <= 0;
					end
				end
				
				//4'b0001 //West
				4'b0001:
				begin
					VGA_PIXEL_X <= (POS_X+1)*30-15;
					if(ypos==10'd0) begin	
						VGA_PIXEL_Y <= (POS_Y+1)*30-15;
						ypos <= (POS_Y+1)*30+15;
					end
					else if(VGA_PIXEL_Y<ypos) begin
						VGA_PIXEL_Y <= VGA_PIXEL_Y+1;
					end
					else begin
						ypos <=10'd0;
						updating <= 0;
					end
				end
				default: begin end
			endcase
			PIXEL_OUT <= `RED;
		end
		else begin
			VGA_PIXEL_X <= 10'd301; //Causes W_EN = 0;
			VGA_PIXEL_Y <= 10'd301;
			PIXEL_OUT <= `BLACK;
			ypos <=10'd0;
			xpos <=10'd0;
		end
		
		prev_update <= update;	
	end
end


endmodule

