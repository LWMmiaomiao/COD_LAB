`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module reg_file(
	input                       clk,

	input                       wen,
	input  [`ADDR_WIDTH - 1:0]  waddr,
	input  [`DATA_WIDTH - 1:0]  wdata,

	input  [`ADDR_WIDTH - 1:0]  raddr1,
	output [`DATA_WIDTH - 1:0]  rdata1,

	input  [`ADDR_WIDTH - 1:0]  raddr2,
	output [`DATA_WIDTH - 1:0]  rdata2
);

reg [`DATA_WIDTH - 1:0] regs [(1 << `ADDR_WIDTH) - 1:1];

always @(posedge clk)
begin
	if (wen && waddr != `ADDR_WIDTH'b0)
	begin
		regs[waddr] <= wdata;
	end
end

assign rdata1 = (raddr1 == 0 ? 32'b0 : regs[raddr1]);
assign rdata2 = (raddr2 == 0 ? 32'b0 : regs[raddr2]);
	
endmodule
