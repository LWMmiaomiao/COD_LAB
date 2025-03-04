`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

// ref: https://nju-projectn.github.io/dlco-lecture-note/exp/03.html#id4
module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output [`DATA_WIDTH - 1:0]  Result,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero

);

	wire [`DATA_WIDTH - 1 : 0]  res_and;
	wire [`DATA_WIDTH - 1 : 0]  res_or;
	wire 						signB;
	wire [`DATA_WIDTH - 1 : 0]  D;
	wire [`DATA_WIDTH - 1 : 0]  C;
	wire 						Co;
	wire [`DATA_WIDTH - 1 : 0]  res_slt;
	wire						Asign;
	wire						Bsign;
	wire						Dsign;
	wire						Csign;

	assign res_and	= A & B;
	assign res_or	= A | B;
	assign signB	= ALUop[2];
	assign D	= {`DATA_WIDTH{signB}} ^ B;
	assign {Co, C}	= (A + D + signB); 
	assign Asign	= A[`DATA_WIDTH-1];
	assign Bsign	= B[`DATA_WIDTH-1];
	assign Dsign	= D[`DATA_WIDTH-1];
	assign Csign	= C[`DATA_WIDTH-1];
	assign CarryOut	= Co ^ signB;
	assign Overflow	= (Asign == Dsign) && (Asign^Csign);
	assign res_slt	= {{`DATA_WIDTH-1{1'b0}},Csign^Overflow};
	assign Result	= ALUop[1] ? 
					(ALUop[0] ? res_or	: res_and):
					(ALUop[0] ? res_slt	: C);
	assign Zero	= (Result == 0);

endmodule
