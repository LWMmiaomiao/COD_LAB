`timescale 10ns / 1ns

module IF (
	input				clk,
	input				rst,

	input		[31:0]	APC,
	input		[31:0]	I9n,

	output		[31:0]	JPC,
	output		[31:0]	S_IMM_00,

	output 				Jump,
	output				Branch,
	output				MemRead,
	output				MemtoReg,
	
	output				MemWrite,
	output				RegWrite,

	output		[ 4:0]	RF_waddr,

	output 		[ 4:0]	RF_raddr1,
	input		[31:0]	RF_rdata1,

	output 		[ 4:0]	RF_raddr2,
	input		[31:0]	RF_rdata2,

	output				ALUOp,
	output		[31:0]	ALU_src1,
	output		[31:0]	ALU_src2
);

endmodule