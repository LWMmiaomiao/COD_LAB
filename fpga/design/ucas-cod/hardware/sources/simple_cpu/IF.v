`timescale 10ns / 1ns

module IF (
	input				clk,
	input				rst,

	input		[31:0]	NPC,
	output	reg	[31:0]	PC,
	output		[31:0]	APC,

	input		[31:0]	EX_I9n,
	output		[31:0]	I9n
);
	always @(posedge clk )
	begin
		if (rst)
			PC <= 32'h0;
		else
			PC <= NPC;	
	end
	assign APC	= PC + 4;

	assign I9n	= EX_I9n;

endmodule