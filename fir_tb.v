//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/20/2023 10:38:55 AM
// Design Name: 
// Module Name: fir_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`define CYCLE_TIME 10.0
`define PATNUM_ 2

`include "/home/ubuntu/SOC_LAB/caravel-soc_fpga-lab/lab-fir/fir/rtl/fir.v"
`include "/home/ubuntu/SOC_LAB/caravel-soc_fpga-lab/lab-fir/fir/tb/bram11.v"

module fir_tb
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter Data_Num    = 600
)();

wire                        awready;
wire                        wready;
reg                         awvalid;
reg   [(pADDR_WIDTH-1): 0]  awaddr;
reg                         wvalid;
reg signed [(pDATA_WIDTH-1) : 0] wdata;
wire                        arready;
reg                         rready;
reg                         arvalid;
reg         [(pADDR_WIDTH-1): 0] araddr;
wire                        rvalid;
wire signed [(pDATA_WIDTH-1): 0] rdata;

reg                         ss_tvalid;
reg signed [(pDATA_WIDTH-1) : 0] ss_tdata;
reg                         ss_tlast;
wire                        ss_tready;

reg                         sm_tready;
wire                        sm_tvalid;
wire signed [(pDATA_WIDTH-1) : 0] sm_tdata;
wire                        sm_tlast;

reg                         axis_clk;
reg                         axis_rst_n;

// ram for tap
wire [3:0]               tap_WE;
wire                     tap_EN;
wire [(pDATA_WIDTH-1):0] tap_Di;
wire [(pADDR_WIDTH-1):0] tap_A;
wire [(pDATA_WIDTH-1):0] tap_Do;

// ram for data RAM
wire [3:0]               data_WE;
wire                     data_EN;
wire [(pDATA_WIDTH-1):0] data_Di;
wire [(pADDR_WIDTH-1):0] data_A;
wire [(pDATA_WIDTH-1):0] data_Do;

// ================================================================
// Variable declaration
real CYCLE = `CYCLE_TIME;
integer PATNUM = `PATNUM_;

integer Din, golden, input_data, golden_data;
integer timeout = 1000000;
integer i, k, m;
integer pat;

reg signed [(pDATA_WIDTH-1):0] Din_list[0:(Data_Num-1)];
reg signed [(pDATA_WIDTH-1):0] golden_list[0:(Data_Num-1)];

reg [31:0]  data_length;
reg signed [31:0] coef[0:10]; // fill in coef 

integer error;
integer error_coef;
reg status_error;
// ================================================================

fir u_fir(
	.awready(awready),
	.wready(wready),
	.awvalid(awvalid),
	.awaddr(awaddr),
	.wvalid(wvalid),
	.wdata(wdata),
	.arready(arready),
	.rready(rready),
	.arvalid(arvalid),
	.araddr(araddr),
	.rvalid(rvalid),
	.rdata(rdata),
	.ss_tvalid(ss_tvalid),
	.ss_tdata(ss_tdata),
	.ss_tlast(ss_tlast),
	.ss_tready(ss_tready),
	.sm_tready(sm_tready),
	.sm_tvalid(sm_tvalid),
	.sm_tdata(sm_tdata),
	.sm_tlast(sm_tlast),

	// ram for tap
	.tap_WE(tap_WE),
	.tap_EN(tap_EN),
	.tap_Di(tap_Di),
	.tap_A(tap_A),
	.tap_Do(tap_Do),

	// ram for data
	.data_WE(data_WE),
	.data_EN(data_EN),
	.data_Di(data_Di),
	.data_A(data_A),
	.data_Do(data_Do),

	.axis_clk(axis_clk),
	.axis_rst_n(axis_rst_n)
);
    
// RAM for tap
bram11 tap_RAM (
	.CLK(axis_clk),
	.WE(tap_WE),
	.EN(tap_EN),
	.Di(tap_Di),
	.A(tap_A),
	.Do(tap_Do)
);

// RAM for data: choose bram11 or bram12
bram11 data_RAM(
	.CLK(axis_clk),
	.WE(data_WE),
	.EN(data_EN),
	.Di(data_Di),
	.A(data_A),
	.Do(data_Do)
);

// dump waveform
initial begin
	`ifdef RTL
		$fsdbDumpfile("fir.fsdb");
		$fsdbDumpvars(0, "+mda");
	`elsif GATE
		$fsdbDumpfile("fir_SYN.fsdb");
		$sdf_annotate("fir_SYN.sdf", u_fir);
		$fsdbDumpvars();
	`endif
end

// gen clock
initial begin
	axis_clk = 0;
	forever begin
		#(CYCLE / 2.0) axis_clk = (~axis_clk);
	end
end

// gen asy reset
task reset_task; begin
	force axis_clk = 0;
	axis_rst_n = 1;
    #(2.0) axis_rst_n = 0;
	#(2.0);
    #(10) axis_rst_n = 1;
	#(10) release axis_clk;
end endtask

// initial all input
task init_task; begin
	// input data length
	data_length = 0;
	
	// error counter
	error = 0;
	error_coef = 0;
	status_error = 0;
	pat = 0;
	
	// for check streamOut
	sm_tready = 1;
	
	// coef array
	coef[0]  =  32'd0;
	coef[1]  = -32'd10;
	coef[2]  = -32'd9;
	coef[3]  =  32'd23;
	coef[4]  =  32'd56;
	coef[5]  =  32'd63;
	coef[6]  =  32'd56;
	coef[7]  =  32'd23;
	coef[8]  = -32'd9;
	coef[9]  = -32'd10;
	coef[10] =  32'd0;
	
	// axi-lite init
	awvalid = 0;
	wvalid = 0;
	wdata = 'dx;
	awaddr = 'dx;
	
	arvalid = 0;
	araddr = 'dx;
	rready = 0;
	
	// axi stream
	ss_tvalid = 0;
	ss_tdata = 'dx;
	ss_tlast = 0;
	
	// read input signal and golden output
	Din = $fopen("/home/ubuntu/SOC_LAB/caravel-soc_fpga-lab/lab-fir/fir/tb/samples_triangular_wave.dat","r");
	golden = $fopen("/home/ubuntu/SOC_LAB/caravel-soc_fpga-lab/lab-fir/fir/tb/out_gold.dat","r");
	
	for(m=0; m<Data_Num; m=m+1) begin
		input_data = $fscanf(Din,"%d", Din_list[m]);
		golden_data = $fscanf(golden,"%d", golden_list[m]);
		data_length = data_length + 1;
	end
end endtask

initial begin
	
	init_task;
	reset_task;

	fork
		AXI_Lite_init;
		AXI_Stream_init;
		check_ans;
	join
	
	pat = pat + 1;
	
	while(pat < PATNUM) begin
	
		// let design reset the data bram
		repeat(10) @(posedge axis_clk);	
		$display("---------------------------------------------");
		$display("-----------------Write start-----------------");
		$display("---------------------------------------------");
		config_write(12'h00, 32'h0000_0001);
		
		fork
			AXI_Stream_init;
			check_ans;
		join
		
		pat = pat + 1;
	end
	
	$display("---------------------------------------------");
	$display("----------------Simulation End---------------");
	$display("---------------------------------------------");
	repeat(10) @(posedge axis_clk);	
	$finish;
end

// AXI-Stream part
task AXI_Stream_init; begin
	// wait asy reset
	while(!axis_rst_n) @(posedge axis_clk);
	
	$display("---------------------------------------------");
	$display("---------------Start simulation--------------");
	$display("---------------------------------------------");
	$display("------Start the data input(AXI-Stream)-------");
	$display("---------------------------------------------");
	
	ss_tvalid = 0;
	@(posedge axis_clk);
	
	for(i=0; i<(data_length-1); i=i+1) begin
		ss_tlast = 0;
		ss(Din_list[i]);
		@(posedge axis_clk);
	end

	ss_tlast = 1;
	ss(Din_list[(Data_Num-1)]);
	ss_tlast = 0;
	
	$display("---------------------------------------------");
	$display("-----------------Check Idle------------------");
	$display("---------------------------------------------");
	// check idle = 0, start = 0, done = 0
	config_read_check(12'h00, 32'h00, 32'h0000_000f);
	
	$display("---------------------------------------------");
	$display("--------End the data input(AXI-Stream)-------");
	$display("---------------------------------------------");
end endtask

// check fir calculation result
task check_ans; begin
    while(sm_tvalid !== 1) @(posedge axis_clk);

    for(k=0;k < data_length;k=k+1) begin
        sm(golden_list[k],k);
        @(posedge axis_clk);
    end
	
	$display("---------------------------------------------");
	$display("-----------------Check done------------------");
	$display("---------------------------------------------");
	// check ap_done = 1 (0x00 [bit 1])
	config_read_check(12'h00, 32'h02, 32'h0000_0002); 
	
	$display("---------------------------------------------");
	$display("-----------------Check Idle------------------");
	$display("---------------------------------------------");
	// check ap_idle = 1 (0x00 [bit 2])
	config_read_check(12'h00, 32'h04, 32'h0000_0004); 

	if(error === 0 && error_coef === 0) begin
		$display("---------------------------------------------");
		$display("-----------Congratulations! Pass-------------");
		$display("---------------------------------------------");
	end else begin
		$display("---------------------------------------------");
		$display("-------------Simulation Failed---------------");
		$display("---------------------------------------------");
		$display("Num of error: 		%d", error);
		$display("Num or error coef: 	%d", error_coef);
		$display("Num or error status: 	%d", status_error);
	end
end endtask

// Prevent hang
task hang_check; begin
	while(timeout > 0) begin
		@(posedge axis_clk);
		timeout = timeout - 1;
	end
	$display($time, "Simualtion Hang ....");
	$finish;
end endtask

// AXI-Lite part
task AXI_Lite_init; begin
	
	// wait asy reset
	while(!axis_rst_n) @(posedge axis_clk);
	
	// start axi lite
	$display("---------------------------------------------");
	$display("----Start the coefficient input(AXI-lite)----");
	$display("---------------------------------------------");
	config_write(12'h10, data_length);

	for(k=0; k<Tape_Num; k=k+1) begin
		config_write(12'h20+4*k, coef[k]);
	end

	// read-back and check
	$display(" Check Coefficient ...");
	for(k=0; k<Tape_Num; k=k+1) begin
		config_read_check(12'h20+4*k, coef[k], 32'hffffffff);
	end

	$display(" Tap Coef programming done ...");
	$display("Start FIR");

	@(posedge axis_clk);
	
	$display("---------------------------------------------");
	$display("-----------------Write start-----------------");
	$display("---------------------------------------------");
	config_write(12'h00, 32'h0000_0001);
	
	$display("---------------------------------------------");
	$display("-----End the coefficient input(AXI-lite)-----");
	$display("---------------------------------------------");
end endtask

// AXI-lite protocol
task config_write;
	input [11:0]    addr;
	input [31:0]    data;
	begin
		awvalid <= 1;
		awaddr <= addr;
		while(!awready) @(posedge axis_clk);
		
		awvalid <= 0;
		awaddr <= 'dx;
		@(posedge axis_clk);
		while (!wready) @(posedge axis_clk);
		
		wvalid  <= 1;
		wdata <= data;
		@(posedge axis_clk);
		
        wvalid  <= 0;
        wdata   <= 'dx;
	end
endtask

task config_read_check;
	input [11:0]        addr;
	input signed [31:0] exp_data;
	input [31:0]        mask;
	begin
		arvalid <= 0;
		@(posedge axis_clk);

		arvalid <= 1;
		araddr <= addr;
		while(!arready) @(posedge axis_clk);
		
		arvalid <= 0;
		araddr <= 'dx;
		rready <= 1;
		@(posedge axis_clk);
		while(!rvalid) @(posedge axis_clk);
		
		if((rdata & mask) !== (exp_data & mask)) begin
			$display("ERROR: exp = %d, rdata = %d", exp_data, rdata);
			
			if(addr !== 'h00) 	error_coef = error_coef + 1;
			else 				status_error = status_error + 1;
			
		end else begin
			$display("OK: exp = %d, rdata = %d", exp_data, rdata);
		end
		rready <= 0;
	end
endtask

task ss;
	input  signed [31:0] in1;
	begin
		ss_tvalid <= 1;
		ss_tdata  <= in1;
		while(!ss_tready) @(posedge axis_clk);
	end
endtask

task sm;
	input  signed [31:0] in2; 	// golden data
	input         [31:0] pcnt; // pattern count
	begin

        while(sm_tvalid !== 1) @(posedge axis_clk);
		if(sm_tvalid === 1 && sm_tready === 1) begin
			if(sm_tdata !== in2) begin
				$display("[ERROR] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
				error = error + 1;
			end else begin
				$display("[PASS]  [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
			end
		end
		
	end
endtask

endmodule

