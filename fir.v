`timescale 1ns / 1ps

module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    // AXI-lite write
    output  reg                      awready,
    output  reg                      wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
	
	// AXI-lite read
    input   wire                     arvalid,
    output  reg                      arready,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  reg                      rvalid,
    input   wire                     rready,
    output  reg  [(pDATA_WIDTH-1):0] rdata,    

    // AXI-stream -- input from testbench
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                      ss_tready, 

    // AXI-stream -- output to testbench
    input   wire                     sm_tready, 
    output  reg                      sm_tvalid, 
    output  reg  [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                      sm_tlast, 
    
    // bram for tap RAM
    output  reg  [3:0]               tap_WE,
    output  reg                      tap_EN,
    output  reg  [(pDATA_WIDTH-1):0] tap_Di,
    output  reg  [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  reg  [3:0]               data_WE,
    output  reg                      data_EN,
    output  reg  [(pDATA_WIDTH-1):0] data_Di,
    output  reg  [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
// ===============================================
// variables declaratioin
// block level protocol
reg ap_start;
reg ap_idle;
reg ap_done;

reg [9:0] data_length;
reg [11:0] addrTap;

// for AXI-Lite control fsm
reg [1:0] axilr_c_state, axilr_n_state;
localparam  AXILR_IDLE   = 0,
            AXILR_A      = 1,
			AXILR_D		 = 2;

// axi write
reg [1:0] axilw_c_state, axilw_n_state;
localparam  AXILW_IDLE   = 0,
            AXILW_A      = 1,
			AXILW_D		 = 2;

// counter
reg [3:0] save_cnt_c, save_cnt_n;
reg [3:0] get_cnt_c, get_cnt_n;
reg [3:0] cnt_c, cnt_n;
reg [9:0] ans_cnt;

// for FIR calculate
reg ss_tready_d1;
reg signed [31:0] mac_c, mac_n;
reg signed [15:0] tap_Do_c, data_Do_c;
reg cal_valid;

// data bram reset
reg [3:0] dbr_c, dbr_n;
localparam 	DBR_0	= 0,
			DBR_1	= 1,
			DBR_2	= 2,
			DBR_3	= 3,
			DBR_4	= 4,
			DBR_5	= 5,
			DBR_6	= 6,
			DBR_7	= 7,
			DBR_8	= 8,
            DBR_9	= 9,
			DBR_10	= 10,
			DBR_11	= 11,
			DBR_IDLE= 12;
			
// ===============================================
// AXI-Lite FSM
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		axilr_c_state <= AXILR_IDLE;
		axilw_c_state <= AXILW_IDLE;
	end else begin
		axilr_c_state <= axilr_n_state;
		axilw_c_state <= axilw_n_state;
	end
end

always@(*) begin
	// default
	axilw_n_state = axilw_c_state;
	axilr_n_state = axilr_c_state;
	
	// write part
	case(axilw_c_state)
		AXILW_IDLE: if(awvalid == 1) 					axilw_n_state = AXILW_A;
		AXILW_A: 	if(awready == 1 && awvalid == 1)	axilw_n_state = AXILW_D;
		AXILW_D: 	if(wready == 1 && wvalid == 1)		axilw_n_state = AXILW_IDLE;
	endcase
	
	// read part
	case(axilr_c_state)
		AXILR_IDLE:	if(arvalid == 1)					axilr_n_state = AXILR_A;
		AXILR_A:	if(arready == 1 && arvalid == 1)	axilr_n_state = AXILR_D;
		AXILR_D:	if(rvalid == 1 && rready == 1)		axilr_n_state = AXILR_IDLE;
	endcase
end

// AXI-Lite write
always@(*) begin
	awready 	= (axilw_c_state == AXILW_A) ? 1 : 0;
	wready		= (axilw_c_state == AXILW_D) ? 1 : 0;
end

// AXI-Lite read
always@(*) begin
	arready 	= (axilr_c_state == AXILW_A) ? 1 : 0;
	
	// rdata
	if(rready == 1 && rvalid == 1) begin
		if(addrTap != 0) 	rdata = tap_Do;
		else				rdata = {29'b0, ap_idle, ap_done, ap_start};
	end
	else					rdata = 0;
end
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n)		rvalid <= 0;
	else begin
		if(rready == 1 && axilr_n_state == axilr_c_state)	
						rvalid <= 1;
		else 			rvalid <= 0;
	end
end

// data_length
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n)											data_length <= 0;
	else if(wready == 1 && wvalid == 1 && addrTap == 'h10)	data_length <= wdata;
	else													data_length <= data_length;
end

// tap bram
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) 							addrTap <= 0;
	else begin
		if(awready == 1 && awvalid == 1) 		addrTap <= awaddr;
		else if(arready == 1 && arvalid == 1)	addrTap <= araddr;
	end
end

always@(*) begin
	tap_WE = (axilw_c_state == AXILW_D)	? 4'b1111 : 4'b0000;
	
	tap_EN = ((axilw_c_state == AXILW_D)
			||(axilr_c_state == AXILR_D && addrTap != 0)
			||(ap_idle == 0 && dbr_c == DBR_IDLE)) ? 1 : 0;
			
	tap_Di = (wready == 1 && wvalid == 1) ? wdata : 0;
	
	// 'h20 is tap offset 
	if((wready == 1) || (axilr_c_state == AXILR_D && addrTap != 0))
											tap_A = addrTap - 'h20;
	else if(ap_idle == 0 && dbr_c == DBR_IDLE)
											tap_A = (cnt_c - 2) << 2;
	else									tap_A = 0;
end

// counter
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		save_cnt_c 	<= 0;
		get_cnt_c 	<= 0;
		cnt_c		<= 0;
		ans_cnt 	<= 0;
	end else begin
		save_cnt_c 	<= save_cnt_n;
		get_cnt_c 	<= get_cnt_n;
		cnt_c		<= cnt_n;
		
		// ans_cnt
		if(sm_tvalid == 1) 		ans_cnt <= ans_cnt + 1;
		else if(ap_idle == 1)	ans_cnt <= 0;
		else					ans_cnt <= ans_cnt;
					
	end
end

always@(*) begin
	// default
	save_cnt_n 	= save_cnt_c;
	get_cnt_n 	= get_cnt_c;
	cnt_n		= cnt_c;
	
	// cnt_n
	if(cnt_c == 12)							cnt_n = 0;
	else if(ap_idle == 0 && dbr_c == DBR_IDLE)
											cnt_n = cnt_c + 1;
	else if(addrTap == 0 && wdata == 1 && wready == 1 && wvalid == 1)		
											cnt_n = 0;
	
	// save_cnt_n
	if(save_cnt_c == 10 && ss_tready == 1)	save_cnt_n = 0;
	else if(ap_idle == 0 && cnt_c == 0 && dbr_c == DBR_IDLE)
											save_cnt_n = save_cnt_c + 1;
	else if(addrTap == 0 && wdata == 1 && wready == 1 && wvalid == 1)		
											save_cnt_n = 0;
	
	// get_cnt_n
	if(ss_tready_d1 == 1)					get_cnt_n = save_cnt_c;
	else if(get_cnt_c == 10)				get_cnt_n = 0;
	else if(ap_idle == 0 && dbr_c == DBR_IDLE)
											get_cnt_n = get_cnt_c + 1;
	
end

// AXI-Stream read
always@(*) begin
	ss_tready = (cnt_c == 0 && ap_idle == 0 && ans_cnt < 'd600 && ss_tvalid == 1 && dbr_c == DBR_IDLE) ? 1 : 0;
end
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) 	ss_tready_d1 <= 0;
	else 				ss_tready_d1 <= ss_tready;
end

// AXI-Stream write
always@(*) begin
	sm_tdata 	= mac_c;
	sm_tlast	= (ans_cnt == 'd599 && sm_tvalid == 1);	
end

always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n)		sm_tvalid <= 0;
	else if(cnt_c == 0 && cal_valid == 1 && ans_cnt < 'd600)
						sm_tvalid <= 1;
	else				sm_tvalid <= 0;						
end

// Data bram
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) 	dbr_c <= DBR_IDLE;
	else				dbr_c <= dbr_n;
end

always@(*) begin
	case(dbr_c)
		DBR_0	:	dbr_n = DBR_1	;
		DBR_1	:   dbr_n = DBR_2	;
		DBR_2	:   dbr_n = DBR_3	;
		DBR_3	:   dbr_n = DBR_4	;
		DBR_4	:   dbr_n = DBR_5	;
		DBR_5	:   dbr_n = DBR_6	;
		DBR_6	:   dbr_n = DBR_7	;
		DBR_7	:   dbr_n = DBR_8	;
		DBR_8	:   dbr_n = DBR_9	;
		DBR_9	:   dbr_n = DBR_10	;
		DBR_10	:   dbr_n = DBR_11	;
		DBR_11	:   dbr_n = DBR_IDLE;
		DBR_IDLE:	dbr_n = (addrTap == 0 && wdata == 1 && wready == 1 && wvalid == 1) ? DBR_0 : DBR_IDLE;
		default : 	dbr_n = dbr_c;
	endcase
end

always@(*) begin
	data_WE = ((dbr_c != DBR_IDLE) 
			|| (ap_idle == 0 && cnt_c == 0 && dbr_c == DBR_IDLE)) ? 4'b1111 : 4'b0000;
	
	data_EN = ((dbr_c != DBR_IDLE)
			|| (ap_idle == 0 && dbr_c == DBR_IDLE)) ? 1 : 0;
		
	// data_Di
	if(dbr_c != DBR_IDLE)					data_Di = 0;
	else if(ap_idle == 0 && cnt_c == 0 && dbr_c == DBR_IDLE)		
											data_Di = ss_tdata;
	else 									data_Di = 0;
	
	// 'h20 is tap offset 
	if(dbr_c != DBR_IDLE)					data_A = dbr_c << 2;
	else if(ap_idle == 0 && cnt_c == 0 && dbr_c == DBR_IDLE)
											data_A = save_cnt_c << 2;
	else if(ap_idle == 0 && dbr_c == DBR_IDLE)
											data_A = get_cnt_c << 2;
	else									data_A = 0;
	
end

// multiply and accumulate
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n)	mac_c <= 0;
	else			mac_c <= mac_n;
end

// cal_valid flag
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		cal_valid <= 0;
	end else begin
		if(ap_idle == 0 && dbr_c == DBR_IDLE) begin
			if(cnt_c == 2)		cal_valid <= 1;
			else if(cnt_c == 0)	cal_valid <= 0;
			else 				cal_valid <= cal_valid;
		end
		else 					cal_valid <= 0;
	end
end

always@(*) begin
	tap_Do_c 	= tap_Do;
	data_Do_c 	= data_Do;
	
	if(cal_valid) 	mac_n = (tap_Do * data_Do) + mac_c;
	else			mac_n = 0;
end

// ap_start
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		ap_start 	<= 0;
	end else begin
		if(addrTap == 0 && wdata == 1 && wready == 1 && wvalid == 1) begin
			ap_start <= 1;
		end
		else if(ss_tready == 1 && ss_tvalid == 1) begin
			ap_start <= 0;
		end
	end
end

// ap_idle
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		ap_idle	<= 1;
	end else begin
		if(addrTap == 0 && wdata == 1 && wready == 1 && wvalid == 1)	
										ap_idle <= 0;
		else if(sm_tlast == 1) 			ap_idle <= 1;
	end
end

// ap_done
always@(posedge axis_clk or negedge axis_rst_n) begin
	if(!axis_rst_n) begin
		ap_done		<= 0;
	end else begin
		if(sm_tlast == 1)		ap_done <= 1;
		else if(addrTap == 0 && wdata == 1 && wready == 1 && wvalid == 1) 	
								ap_done <= 0;
	end
end
endmodule