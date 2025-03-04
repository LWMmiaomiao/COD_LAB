`timescale 10ns / 1ns

module simple_cpu(
	input			clk,
	input			rst,

	output	[31:0]	PC,
	input	[31:0]	Instruction,

	output	[31:0]	Address,

	output			MemWrite,
	output	[31:0]	Write_data,
	output 	[ 3:0]	Write_strb,

	output			MemRead,
	input	[31:0]	Read_data
);

	wire 	[31:0]	NPC;
	wire	[31:0]	APC;
	wire	[31:0]	I9n;
	IF IF(
		.clk	(clk		),
		.rst	(rst		),
		.NPC	(NPC		),
		.PC		(PC			),
		.APC	(APC		),
		.EX_I9n	(Instruction),
		.I9n	(I9n		)
	);

	wire			RF_wen;
	wire	[ 4:0]	RF_waddr;
	wire	[31:0]	RF_wdata;

	wire	[ 4:0]	RF_raddr1;
	wire	[31:0]	RF_rdata1;

	wire	[ 4:0]	RF_raddr2;
	wire	[31:0]	RF_rdata2;

RF RF(
	.clk    	(clk     	),
	.wen    	(RF_wen     ),
	.waddr  	(RF_waddr   ),
	.wdata  	(RF_wdata   ),
	.raddr1 	(RF_raddr1  ),
	.rdata1 	(RF_rdata1  ),
	.raddr2 	(RF_raddr2  ),
	.rdata2 	(RF_rdata2  )
);


endmodule
