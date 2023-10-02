`timescale 1ns/10ps
module GPSDC(clk, reset_n, DEN, LON_IN, LAT_IN, COS_ADDR, COS_DATA, ASIN_ADDR, ASIN_DATA, Valid, a, D);
input              clk;
input              reset_n;
input              DEN;
input      [23:0]  LON_IN;
input      [23:0]  LAT_IN;
input      [95:0]  COS_DATA;
output     [6:0]   COS_ADDR;
input      [127:0] ASIN_DATA;
output     [5:0]   ASIN_ADDR;
output  reg           Valid;
output  reg   [39:0]  D;
output  reg  [63:0]  a;

localparam S_IDLE0 = 4'b0000;
localparam S_L1    = 4'b1000;
//localparam S_L2    = 4'b000;
localparam S_A	   = 4'b0001;
localparam S_INNER0= 4'b1011;
localparam S_FCOS  = 4'b0011;
localparam S_INNER = 4'b0010;
localparam S_MID   = 4'b0110;
localparam S_FASIN = 4'b0111;
localparam S_INNERS= 4'b1001;
localparam S_D     = 4'b0101;
localparam S_IDLE  = 4'b0100;
reg [3:0] c_state, n_state;
// wire reg declare cao
reg [23:0]LON_IN_A, LON_IN_B;
reg [23:0]LAT_IN_A, LAT_IN_B;
wire [47:0] LAT_w;
reg [95:0] COS_127, COS_0;
reg [127:0] ASIN_63, ASIN_0;
//---------------------------------------
// for mid point
wire [23:0]sel_a1, sel_a2;
wire signed [24:0]angle_diff;
wire signed [15:0]rad;
//wire signed [40:0]ang_mul;
wire signed [32:0]ang_mul;
wire signed [32:0]ang_result;
//wire signed [64:0]ang_result;
wire [63:0]sin2;
wire [127:0]first_result;
reg  [127:0]first_result_r;
wire [191:0] mid_left;
reg  [63:0] a_r;
wire [63:0] a_w;
// for inner
//reg  [47:0] cosA, cosB;
//wire [47:0] inner_result_w;
//wire [95:0] inner_up;
//wire [47:0] inner_down; 
//wire [47:0] sel_y0, sel_y1, sel_x0, sel_x1, sel_x;

reg  [63:0] cosA, cosB;
reg  [63:0] inner_result_r;
wire [63:0] inner_result_w;
wire [127:0] inner_up;
wire [127:0]inner_up_padding;
wire [63:0] inner_down; 
wire [63:0] up_C, up_D;
wire [63:0] sel_y0, sel_y1, sel_x0, sel_x1, sel_x;


// wire reg declare hunry 

// ------------------------------------------------------------
// regs & wire
// ------------------------------------------------------------
// find cos
wire  [47:0] cos_in, cos_val;
reg  [6:0]  p_h, p_l;

// find asin
wire [63:0] asin_in, asin_val;
reg [5:0]  asin_p_h, asin_p_l;

reg [47:0] cos_y0, cos_y1; 
reg [47:0] cos_x0, cos_x1;
reg [63:0] asin_x0, asin_x1;
reg [63:0] asin_y0, asin_y1;
//---------------------------------------
// calculate inner product 
//assign sel_y0 = 48'hF0A3ACB6;
//assign sel_y1 = 48'hF0A5D8F1;
//assign sel_x0 = 48'h18C8D96FA8;
//assign sel_x1 = 48'h18C8DFCE31;
//assign sel_x  = 48'h18c8db0000;

assign inner_down = (sel_x1 - sel_x0);
assign up_C = (sel_x - sel_x0);
assign up_D = (sel_y1 - sel_y0);
assign inner_up = sel_y0 * inner_down + up_C * up_D;
assign inner_up_padding = (c_state == S_INNERS)? inner_up : {inner_up[95:0], 32'd0};
assign inner_result_w = inner_up_padding / inner_down; 

assign sel_y0 = (c_state == S_INNERS)? asin_y0 : cos_y0;
assign sel_y1 = (c_state == S_INNERS)? asin_y1 : cos_y1;
assign sel_x0 = (c_state == S_INNERS)? asin_x0 : cos_x0;
assign sel_x1 = (c_state == S_INNERS)? asin_x1 : cos_x1;
assign sel_x  = (c_state == S_INNERS)? a_r  : LAT_w;

always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  cosB <= 64'd0;
      end else begin
	  if(c_state == S_INNER || c_state == S_INNER0)
          	cosB <= inner_result_w;
	  else
		cosB <= cosB;
      end
end

always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  cosA <= 64'd0;
      end else begin
	  if(c_state == S_INNER)
          	cosA <= cosB;
	  else
		cosA <= cosA;
      end
end


//---------------------------------------
// Mid inner product 
// rad = 16'h477; angle: integer = 8bits; FP = 16bit ; sel : 8+16

assign sel_a1 = (c_state == S_INNER)? LON_IN_B : LAT_IN_B;
assign sel_a2 = (c_state == S_INNER)? LON_IN_A : LAT_IN_A;
assign rad = 16'h477;
assign angle_diff = {1'b0,sel_a1} - {1'b0, sel_a2}; //$signed(sel_a1) - $signed(sel_a2);
assign ang_mul = angle_diff * $signed({9'd0 , 16'h477});//{8'd0 , 16'h477};
// 1bit signed ; 7 bit integer; 32 bit FP;
//assign ang_result = {ang_mul[49], ang_mul>>1}; //>>>1;
assign ang_result = ang_mul / $signed(2); 
//assign ang_result = ang_mul * ang_mul;
// 1bit signed ; 15 bit integer; 64 bit FP; but actual we need FP is enough
assign sin2 = ang_result * ang_result;
//assign sin2 = ang_result / $signed(4);


//cosA and B : int : 0bit; FP = 32bit 
assign first_result = sin2 * cosB;
assign mid_left = first_result_r * cosB;
assign a_w = sin2 + mid_left[191:128] ;

always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  first_result_r <= 128'd0;
      end else begin
	  if(c_state == S_INNER)
          	first_result_r <= first_result; // + first_result[31];
	  else
		first_result_r <= first_result_r;
      end
end


always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  a_r <= 64'd0;
      end else begin
	  if(c_state == S_MID)
          	a_r <= a_w;
	  else
		a_r <= a_r;
      end
end
//---------------------------------------
// Distance  Int: 8bits FP: 32 bits
wire [87:0] D_w;
// C2A532  1 9A27 3878
assign D_w = 24'd12756274 * inner_result_r;
// for distance
always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  inner_result_r <= 64'd0;
      end else begin
	  if(c_state == S_INNER || c_state == S_INNERS)
          	inner_result_r <= inner_result_w;
	  else
		inner_result_r <= inner_result_r;
      end
end
//---------------------------------------
// FSM block
always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  c_state <= S_IDLE0;
      end else begin
          c_state <= n_state;
      end
end

always@(*) begin
	case(c_state)
		S_IDLE0:if(DEN) n_state = S_L1;
			else	n_state = S_IDLE0;
		S_L1:   n_state = S_A;
		S_A:	if(cos_val == cos_y0) n_state = S_INNER0;
			else	 	      n_state = S_A;
		S_INNER0: n_state = S_IDLE;
		S_FCOS: if(cos_val == cos_y0) n_state = S_INNER;
			else	 	      n_state = S_FCOS;
		S_INNER:n_state = S_MID;
		S_MID:  n_state = S_FASIN;
		S_FASIN:if(asin_val == asin_y0) n_state = S_INNERS;
			else	 		n_state = S_FASIN;
		S_INNERS: n_state = S_D;
		S_D:	  n_state = S_IDLE;
		S_IDLE: if(DEN) n_state = S_FCOS;
			else	n_state = S_IDLE;
		default: n_state = S_IDLE;
	endcase
end

//---------------------------------------
// INPUT block
always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  LON_IN_A <= 24'b0;
      end else begin
	  if(DEN)
          	LON_IN_A <= LON_IN_B;
	  else
		LON_IN_A <= LON_IN_A;
      end
end

always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  LAT_IN_A <= 24'b0;
      end else begin
	  if(DEN)
          	LAT_IN_A <= LAT_IN_B;
	  else
		LAT_IN_A <= LAT_IN_A;
      end
end

always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  LON_IN_B <= 24'b0;
      end else begin
	  if(DEN)
          	LON_IN_B <= LON_IN;
	  else
		LON_IN_B <= LON_IN_B;
      end
end

always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  LAT_IN_B <= 24'b0;
      end else begin
	  if(DEN)
          	LAT_IN_B <= LAT_IN;
	  else
		LAT_IN_B <= LAT_IN_B;
      end
end
//---------------------------------------
// OUTPUT block
always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  Valid <= 1'b0;
      end else begin
	  if(c_state == S_D)
          	Valid <= 1'b1;
	  else
		Valid <= 1'b0;
      end
end

always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  D <= 40'b0;
      end else begin
	  if(c_state == S_D)
          	D <= D_w[71:32] ;
	  else
		D <= 40'd0;
      end
end

always@(posedge clk, negedge reset_n) begin
      if(!reset_n) begin
	  a <= 64'b0;
      end else begin
	  if(c_state == S_D)
          	a <= a_r;
	  else
		a <= 64'd0;
      end
end

// ------------------------------------------------------------
// design
// ------------------------------------------------------------
// cos 0, 127
always @(posedge clk or negedge reset_n) begin
	if (!reset_n) 				 COS_0 <= 96'd0;
	else if (c_state == S_IDLE0) COS_0 <= COS_DATA;
	else 						 COS_0 <= COS_0;
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) 		    COS_127 <= 96'd0;
	else if (c_state == S_L1) COS_127 <= COS_DATA;
	else 				    COS_127 <= COS_127;
end

// find cos
wire [7:0] COS_ADDR_w;
assign cos_in   = COS_DATA[95:48];
assign cos_val  = COS_DATA[47:0];
assign COS_ADDR_w = (p_h + p_l);
assign COS_ADDR =  COS_ADDR_w>> 1;
assign LAT_w = {8'd0, LAT_IN_B, 16'd0};
always @(posedge clk or negedge reset_n) begin
	if (!reset_n) p_h <= 7'h00;
	else begin
		case (c_state)
			S_IDLE0: 	 p_h <= (DEN) ? 7'h7f : 7'h00;
			S_L1: 		 p_h <= 7'h7f;
			S_A, S_FCOS: p_h <= (cos_in > LAT_w) ? COS_ADDR : p_h;
			default: 	 p_h <= 7'h7f; // to load cos(x(0, 0)
		endcase 
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) p_l <= 7'h00;
	else begin
		case (c_state)
			S_IDLE0: 	 p_l <= (DEN) ? 7'h7f : 7'h00;
			S_L1: 		 p_l <= 7'h00;
			S_A, S_FCOS: p_l <= (cos_in > LAT_w) ? p_l : COS_ADDR;
			default: 	 p_l <= 7'h00; // to load cos(x(0, 0))
		endcase 
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) cos_y0 <= 48'd0;
	else begin
		case (c_state)
			S_IDLE0: 	 cos_y0 <= cos_val;
			S_IDLE: 	 cos_y0 <= COS_0[47:0];
			S_A, S_FCOS: cos_y0 <= (cos_in > LAT_w) ? cos_y0 : cos_val;
			default: 	 cos_y0 <= cos_y0;
		endcase
	
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) cos_x0 <= 48'd0;
	else begin
		case (c_state)
			S_IDLE0: 	 cos_x0 <= cos_in;
			S_IDLE: 	 cos_x0 <= COS_0[95:48];
			S_A, S_FCOS: cos_x0 <= (cos_in > LAT_w) ? cos_x0 : cos_in;
			default: 	 cos_x0 <= cos_x0;
		endcase
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) cos_y1 <= 48'd0;
	else begin
		case (c_state)
			S_L1: 		 cos_y1 <= cos_val;
			S_IDLE: 	 cos_y1 <= COS_127[47:0];
			S_A, S_FCOS: cos_y1 <= (cos_in > LAT_w) ? cos_val : cos_y1;
			default: 	 cos_y1 <= cos_y1;
		endcase
	
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) cos_x1 <= 48'd0;
	else begin
		case (c_state)
			S_L1: 	     cos_x1 <= cos_in; 
			S_IDLE: 	 cos_x1 <= COS_127[95:48];
			S_A, S_FCOS: cos_x1 <= (cos_in > LAT_w) ? cos_in : cos_x1;
			default: 	 cos_x1 <= cos_x1;
		endcase
	end 
end


// asin 63, 0
always @(posedge clk or negedge reset_n) begin
	if (!reset_n) 				 ASIN_0 <= 128'd0;
	else if (c_state == S_IDLE0) ASIN_0 <= ASIN_DATA;
	else 						 ASIN_0 <= ASIN_0;
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) 		    ASIN_63 <= 128'd0;
	else if (c_state == S_L1)   ASIN_63 <= ASIN_DATA;
	else 				    ASIN_63 <= ASIN_63;
end

// find arcsin
wire [6:0]ASIN_ADDR_w;
assign asin_in   = ASIN_DATA[127:64];
assign asin_val  = ASIN_DATA[63:0];
assign ASIN_ADDR_w = (asin_p_h + asin_p_l);
assign ASIN_ADDR =  ASIN_ADDR_w>> 1;
always @(posedge clk or negedge reset_n) begin
	if (!reset_n) asin_p_h <= 6'h00;
	else begin
		case (c_state)
			S_IDLE0: 	 asin_p_h <= (DEN) ? 6'h3f : 6'h00; 
			S_FASIN: asin_p_h <= (asin_in > a_r) ? ASIN_ADDR : asin_p_h;
			default: 	 asin_p_h <= 6'h3f; // to load cos(x(0, 0)
		endcase 
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) asin_p_l <= 6'h00;
	else begin
		case (c_state)
			S_IDLE0: 	 asin_p_l <= (DEN) ? 6'h3f : 6'h00; 
			S_FASIN: asin_p_l <= (asin_in > a_r) ? asin_p_l : ASIN_ADDR;
			default: 	 asin_p_l <= 6'h00; // to load cos(x(0, 0))
		endcase 
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) asin_y0 <= 128'd0;
	else begin
		case (c_state)
			S_IDLE0: 	 asin_y0 <= asin_val;
			S_IDLE: 	 asin_y0 <= ASIN_0[63:0];
			S_FASIN: asin_y0 <= (asin_in > a_r) ? asin_y0 : asin_val;
			default: 	 asin_y0 <= asin_y0;
		endcase
	
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) asin_x0 <= 128'd0;
	else begin
		case (c_state)
			S_IDLE0: 	 asin_x0 <= asin_in;
			S_IDLE:		 asin_x0 <= ASIN_0[127:64];
			S_FASIN: asin_x0 <= (asin_in > a_r) ? asin_x0 : asin_in;
			default: 	 asin_x0 <= asin_x0;
		endcase
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) asin_y1 <= 128'd0;
	else begin
		case (c_state)
			S_L1: 	 	 asin_y1 <= asin_val;
			S_IDLE: 	 asin_y1 <= ASIN_63[63:0];
			S_FASIN: asin_y1 <= (asin_in > a_r) ? asin_val : asin_y1;
			default: 	 asin_y1 <= asin_y1;
		endcase
	
	end 
end

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) asin_x1 <= 128'd0;
	else begin
		case (c_state)
			S_L1: 		 asin_x1 <= asin_in; 
			S_IDLE: 	 asin_x1 <= ASIN_63[127:64];
			S_FASIN: asin_x1 <= (asin_in > a_r) ? asin_in : asin_x1;
			default: 	 asin_x1 <= asin_x1;
		endcase
	end 
end

endmodule
