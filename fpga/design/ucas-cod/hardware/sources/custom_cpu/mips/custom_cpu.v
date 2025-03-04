`timescale 10ns / 1ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output [31:0] PC,
	output        Inst_Req_Valid,
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,
	output        MemWrite,
	output [31:0] Write_data,
	output [ 3:0] Write_strb,
	output        MemRead,
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15,

	output [69:0] inst_retire
);
	reg [31:0] myPC;
	reg [31:0] Memory_Data_reg;
	reg [31:0] ALUOut;
	reg [31:0] Shift_reg;
	wire [31:0] PC_next;
	wire [31:0] PC_4;
	wire [31:0] Branch_PC;
	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;
	//以下变量声明为Instruction的拆分
	wire [5:0] opcode;
	wire [4:0] rs;
	wire [4:0] rt;
	wire [4:0] rd;
	wire [4:0] shamt;
	wire [5:0] func;
	wire [15:0] immediate;
	wire [31:0] address;
	//以下变量声明为Control部分的输出
	wire RegDst;
	wire Branch;
	wire JUMP;
	wire MemtoReg;
	wire ALUSrc;
	wire RegWrite;
	wire [2:0] ALUop_control;
	wire [2:0] ALUop_R;

	wire [2:0] ALU_operation;//挑选ALUop_control与ALUopn_R作为ALUop信号输入ALU
	wire [31:0] A;
	wire [31:0] B;
	wire Zero;//ALU输出信号
	wire [31:0]ALU_Result;//ALU输出信号
	wire Rtype;
	wire Jtype;//跳转型指令
	wire REGIMM;
	wire Itype;
	wire LUI;
	wire MOV;
	wire MOV_valid;
	wire JR;
	wire CALL;

	wire [31:0] nextPC;
	wire Branch_valid;
	wire [31:0] PC_Branch;
	wire [31:0] PC_Jump;
	wire [31:0] RF_rdata1;
	wire [31:0] RF_rdata2;
	wire [4:0]RF_raddr1;
	wire [4:0]RF_raddr2;
	wire [31:0]	aftershift;

	wire [7:0] LB_data;
	wire [15:0] LH_data;
	wire [31:0] LW_data;
	wire [31:0] LWL_data;
	wire [31:0] LWR_data;
	wire [31:0] MemtoReg_data;

	wire [31:0] ShiftA;
	wire [4:0] ShiftB;
	wire [1:0] Shiftop;
	wire [31:0] ShiftResult;
	wire [31:0] Result;

	wire [3:0] SB_strb;
	wire [3:0] SH_strb;
	wire [3:0] SW_strb;
	wire [3:0] SWL_strb;
	wire [3:0] SWR_strb;
	wire [31:0] SB_data;
	wire [31:0] SH_data;
	wire [31:0] SW_data;
	wire [31:0] SWL_data;
	wire [31:0] SWR_data;

	reg [31:0] Instruction_reg;
	wire PCWriteCond;
	wire PCWrite;
	wire IorD;
	wire IRWrite;
	wire [1:0] PCSource;
	wire [2:0] ALUOp;
	wire [1:0] ALUSrcB;
	wire ALUSrcA;

	reg [9:0] current_state;
	reg [9:0] next_state;
	reg [31:0] EPC;
	reg int_mask;
	wire ERET;
/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/
    assign inst_retire = {RF_wen, RF_waddr, RF_wdata, PC};
// TODO: Please add your custom CPU code here

	localparam  INIT = 10'b0000000001,
				IF	 = 10'b0000000010,
				IW	 = 10'b0000000100,
				ID	 = 10'b0000001000,
				EX	 = 10'b0000010000,
				LD	 = 10'b0000100000,
				ST	 = 10'b0001000000,
				RDW	 = 10'b0010000000,
				WB	 = 10'b0100000000,
				INTR = 10'b1000000000;

	always @(posedge clk) begin
        if (rst) begin
			current_state 	<= INIT;//复位信号有效, 状态机保持INIT初始状态
		end
        else begin
          	current_state 	<= next_state;
		end
    end

	//三段式第二段描述组合逻辑电路, 应该使用阻塞赋值 
	always @(*) begin
        case(current_state)
			INIT : begin
				next_state = IF;
			end
            IF : begin
				if(intr && !int_mask) begin
					next_state = INTR;
				end	
				else if(Inst_Req_Ready) begin
					next_state = IW;
				end
				else begin
					next_state = IF;
				end
			end
			IW : begin
				if(Inst_Valid) begin
					next_state = ID;
				end
				else begin
					next_state = IW;
				end
			end
            ID : begin
                if (Instruction_reg==32'b0) begin//NOP跳回IF
                	next_state = IF;
            	end
                else begin
                    next_state = EX;
                end
            end
            EX : begin
                if (opcode==6'b000010 || Branch || ERET) begin
                    next_state = IF;//J BNE-BEQ-BGEZ-BGTZ-BLEZ-BLTZ
                end
				else if (opcode[5:3]==3'b101) begin
					next_state = ST;
				end
                else if (opcode[5:3]==3'b100) begin//
                    next_state = LD;
                end
                else begin
                    next_state = WB;
                end
            end
			LD : begin
				if (Mem_Req_Ready) begin
					next_state = RDW;
				end
				else begin
					next_state = LD;
				end
			end
			ST : begin
				if (Mem_Req_Ready) begin
					next_state = IF;
				end
				else begin
					next_state = ST;
				end
			end
			RDW : begin
				if (Read_data_Valid) begin
					next_state = WB;
				end
				else begin
					next_state = RDW;
				end
			end
            WB : begin
				next_state = IF;
			end
			INTR : begin
				next_state = IF;
			end
            default: begin
				next_state = IF;
			end
        endcase
    end

    assign Inst_Req_Valid   = current_state[1] && !(intr && !int_mask);//防止中断处理程序仍试图与内存握手
    assign Inst_Ready       = current_state[0] || current_state[2];
    assign MemRead          = current_state[5];
    assign MemWrite         = current_state[6];
    assign Read_data_Ready  = current_state[0] || current_state[7];

	assign PCWriteCond = current_state[4] && Branch;//Branch
	assign PCWrite = current_state[2] && Inst_Valid || current_state[4] && JUMP;//PC+4或JUMP
	assign IorD = !current_state[1];
	assign MemtoReg = current_state[8] && (opcode[5:3]==3'b100);
	assign IRWrite = current_state[2] && Inst_Ready && Inst_Valid;//读入指令
	assign PCSource[0] = current_state[4] && Branch;//ALUout
	assign PCSource[1] = current_state[4] && JUMP;//jump 反之ALUresult
	assign ALUOp = (current_state[2] || current_state[3] || current_state[4] && CALL) ? 3'b010 : ALUop_control;
	//IF计算PC+4, ID计算PC+(signed-extend)immediate<<2, EXE计算A+(signed-extend)immediate
	assign ALUSrcA = (current_state[3] && !Branch) || (current_state[4] && !CALL);
	assign ALUSrcB = {2{current_state[5] || current_state[6] || current_state[3] && Branch}} & 2'b11
					|{2{current_state[4] && (opcode[3] || opcode[5])}} & 2'b10
					|{2{current_state[2] || current_state[4] && CALL}} & 2'b01;
	assign RegWrite = current_state[8];
	assign RegDst = current_state[8] && Rtype;

    always @(posedge clk) begin
        if (IRWrite) begin
            Instruction_reg <= Instruction;
        end
    end

	//实例化寄存器堆
	reg_file registers(
		.clk(clk),
		.waddr(RF_waddr),
		.raddr1(RF_raddr1),
		.raddr2(RF_raddr2),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(RF_rdata1),
		.rdata2(RF_rdata2)
	);
	//先对指令类型与特殊指令进行初步分类与判断
	assign Jtype = (opcode[5:1] == 5'b00001) ? 1 : 0;
	assign Rtype = (opcode[5:0] == 6'b000000) ? 1: 0;
	assign REGIMM = (opcode[5:0] == 6'b000001) ? 1 : 0;
	assign Itype = (opcode[5:2] == 4'b0001) ? 1: 0;
	assign LUI = (opcode[5:0] == 6'b001111) ? 1 : 0;
	assign MOV = (opcode[5:0] == 6'b000000) && (func[5:1] == 5'b00101);
	assign MOV_valid = MOV && (func[0] ^ (RF_rdata2==32'b0));
	assign JR = opcode[5:0] == 6'b000000 && func[5:0] == 6'b001000;
	assign CALL = opcode[5:0] == 6'b000011 || Rtype && (func[5:0] == 6'b001001);
	assign ALUSrc = (opcode[5:3]==3'b001 || opcode[5:3]==3'b100 || opcode[5:3]==3'b101) ? 1 : 0;
	assign Branch = Itype || REGIMM;//BNE-BEQ-BGEZ-BGTZ-BLEZ-BLTZ
	assign JUMP = Rtype && func[5:1] == 5'b00100 || Jtype;
	assign ERET = opcode == 6'b010000;
	assign ALUop_control = (opcode[5:1]==5'b00010) ? 3'b110 :
							((opcode[5:4]==2'b10 || opcode[5:1]==5'b00100) ? 3'b010 :
						 	(opcode[5:2]==4'b0011 ? {opcode[1], 1'b0, opcode[0]} :
						 	(opcode[5:1]==5'b00101 ? {!opcode[0], 2'b11} : 3'b101)));
	//BEQ-BNE-sub L-S-add ADD-add LOGIC-000/001/100/101 SLT-111/110
	//ALUop_control保证与ALUop的操作保持相一致
	//111为slt 010为add 000为and 001为or 100为xor 011为sltu 110为sub
	assign ALUop_R = {3{func[3:2]==2'b00}} & {func[1], 2'b10}
					|{3{func[3:2]==2'b01}} & {func[1], 1'b0, func[0]}
					|{3{func[3:2]==2'b10}} & {!func[0], 2'b11};
	//Rtype对应的ALUop
	assign ALU_operation = (ALUOp == 3'b101) ? ALUop_R : ALUOp;

	//对Instruction的拆分
	assign opcode = Instruction_reg[31:26];
	assign rs = Instruction_reg[25:21];
	assign rt = Instruction_reg[20:16];
	assign rd = Instruction_reg[15:11];
	assign shamt = Instruction_reg[10:6];
	assign func = Instruction_reg[5:0];
	assign immediate = Instruction_reg[15:0];
	assign address = Instruction_reg[25:0];

	//PC更新
	assign Branch_valid = REGIMM & (RF_rdata1[31] ^ rt[0]) 
						| opcode[5:1]==5'b00010 & (Zero ^ opcode[0]) 
						| opcode[5:1]==5'b00011 & (opcode[0] ^ (RF_rdata1[31] | (RF_rdata1==32'b0)));
	//BGEZ且大于等于0  BLTZ且小于0 BEQ且相等 BNE且不等 BGTZ且大于0 BLEZ且小于等于0
	assign PC = myPC;
	assign PC_Jump = (Jtype) ? {PC[31:28], address, 2'b00} : RF_rdata1;//jr/jalr或是j/jal
	assign PC_next = {32{PCSource == 2'b00}} & ALU_Result
					|{32{PCSource == 2'b01}} & ALUOut
                    |{32{PCSource == 2'b10}} & PC_Jump;
	always @(posedge clk) begin
		if(rst==1'b1) begin
			myPC <= 32'b0;	//复位有效
		end
		else if (ERET) begin
			myPC <= EPC;
		end
		else if (PCWrite || Branch_valid && PCWriteCond) begin
			myPC <= PC_next;
 		end
	end

	always @(posedge clk) begin
		if(current_state[9])
			EPC <= PC;
	end
	always @(posedge clk) begin //中断屏蔽字
		if(rst)
			int_mask <= 1'b0;
		else if(current_state[9])
			int_mask <= 1'b1;
		else if(ERET)//ERET
			int_mask <= 1'b0;
	end

	assign RF_raddr1 = rs;
	assign RF_raddr2 = REGIMM ? 5'b0 : rt;//防止BGEZ使用$1寄存器
	assign RF_wen = RegWrite && !JR && !MOV || MOV_valid;
	assign RF_waddr = {5{opcode==6'b000011}} & 5'b11111 | {5{RegDst && !MOV || MOV_valid}} & rd | {5{ALUSrc}} & rt;
	//jal时使用$31, 否则根据RegDst选择使用rd或rt
	assign RF_wdata = MemtoReg ? Memory_Data_reg : Result;//100xxx
	assign A = ALUSrcA ? RF_rdata1 : PC;//无需详细考虑不需要ALU的其他情况, 最后舍去ALU_Result即可
	assign B = {32{ALUSrcB == 2'b00}} & RF_rdata2
            | {32{ALUSrcB == 2'b01}} & 32'b100
	        | {32{ALUSrcB == 2'b10}} & ({32{opcode[2]==1'b1}} & {16'b0, immediate} | {32{opcode[2]==1'b0}} & {{16{immediate[15]}}, immediate})
    	    | {32{ALUSrcB == 2'b11}} & {{14{immediate[15]}}, immediate, 2'b0};

	//实例化ALU单元
	alu ALU_cpu(
		.A(A),
		.B(B),
		.ALUop(ALU_operation),
		.Result(ALU_Result),
		.Overflow(),
		.CarryOut(),
		.Zero(Zero)
	);

    /*内存读写时保持ALUout内容不变*/
	always @(posedge clk) begin
		if (!(current_state[5] || current_state[6] || current_state[7])) begin
        	ALUOut <= ALU_Result;
		end
    end

	assign Result = {32{Rtype && func[5:4]==2'b10 || opcode[5:3]==3'b001 && !LUI}} & ALUOut |
					{32{opcode[5:0] == 6'b000011 || (opcode[5:0] == 6'b000000 && func == 6'b001001)}} & ALUOut |
					{32{Rtype && func[5:3]==3'b000}} & Shift_reg |
					{32{opcode[5:0]==6'b001111}} & {immediate, 16'b0} |
					{32{MOV_valid}} & RF_rdata1;//单周期的PC+8在多周期中放入ALU进行计算
	//不需要从内存中得到的结果, 合并入RF_wdata

	//移位操作
	assign ShiftA = B;
	assign ShiftB = func[2] ? A[4:0] : shamt;
	assign Shiftop = func[1:0];
	shifter Shift_cpu(
		.A(ShiftA),
		.B(ShiftB),
		.Shiftop(Shiftop),
		.Result(ShiftResult)
	);
	always @(posedge clk) begin
		if (!(current_state[5] || current_state[6] || current_state[7])) begin
       		Shift_reg <= ShiftResult;
		end
    end

	//以下对内存读写进行处理
	assign LB_data = {8{ALUOut[1] & ALUOut[0]}} & Read_data[31:24]
				   | {8{ALUOut[1] & !ALUOut[0]}} & Read_data[23:16]
				   | {8{!ALUOut[1] & ALUOut[0]}} & Read_data[15:8]
				   | {8{!ALUOut[1] & !ALUOut[0]}} & Read_data[7:0];
	assign LH_data = (ALUOut[1]) ? Read_data[31:16] : Read_data[15:0];
	assign LWL_data = {32{ALUOut[1] & ALUOut[0]}} & Read_data[31:0]//字最重要部分写入寄存器左侧
				   	| {32{ALUOut[1] & !ALUOut[0]}} & {Read_data[23:0], RF_rdata2[7:0]}
				   	| {32{!ALUOut[1] & ALUOut[0]}} & {Read_data[15:0], RF_rdata2[15:0]}
				   	| {32{!ALUOut[1] & !ALUOut[0]}} & {Read_data[7:0], RF_rdata2[23:0]};
	assign LW_data = Read_data[31:0];
	assign LWR_data = {32{ALUOut[1] & ALUOut[0]}} & {RF_rdata2[31:8], Read_data[31:24]}//字最不重要部分写入寄存器右侧
				   	| {32{ALUOut[1] & !ALUOut[0]}} & {RF_rdata2[31:16], Read_data[31:16]}
				   	| {32{!ALUOut[1] & ALUOut[0]}} & {RF_rdata2[31:24], Read_data[31:8]}
				   	| {32{!ALUOut[1] & !ALUOut[0]}} & Read_data[31:0];
	assign SB_strb = 4'b0001 << (ALUOut[1:0]);
	assign SH_strb = {ALUOut[1], ALUOut[1], !ALUOut[1], !ALUOut[1]};
	assign SW_strb = 4'b1111;
	assign SWL_strb = {ALUOut[1] & ALUOut[0], ALUOut[1], ALUOut[1] | ALUOut[0], 1'b1};//swl与swr互补
	assign SWR_strb = {1'b1,!(ALUOut[1] & ALUOut[0]), !ALUOut[1], (!ALUOut[1]) & (!ALUOut[0])};
	assign Write_strb = {4{MemWrite}} & ({4{opcode[2:0]==3'b000}} & SB_strb
					  					| {4{opcode[2:0]==3'b001}} & SH_strb
										| {4{opcode[2:0]==3'b010}} & SWL_strb
										| {4{opcode[2:0]==3'b011}} & SW_strb
										| {4{opcode[2:0]==3'b110}} & SWR_strb);
	assign SB_data = {32{ALUOut[1:0]==2'b00}} & {{24{1'b0}}, RF_rdata2[7:0]}
				   | {32{ALUOut[1:0]==2'b01}} & {{16{1'b0}}, RF_rdata2[7:0], {8{1'b0}}}
				   | {32{ALUOut[1:0]==2'b10}} & {{8{1'b0}}, RF_rdata2[7:0], {16{1'b0}}}
				   | {32{ALUOut[1:0]==2'b11}} & {RF_rdata2[7:0], {24{1'b0}}};
	assign SH_data = {32{!ALUOut[1]}} & {16'b0,RF_rdata2[15:0]}
				   | {32{ALUOut[1]}} & {RF_rdata2[15:0],16'b0};
	assign SW_data = RF_rdata2;
	assign SWL_data = {32{ALUOut[1:0]==2'b00}} & {{24{1'b0}}, RF_rdata2[31:24]}//寄存器左侧写入字最重要部分
					  | {32{ALUOut[1:0]==2'b01}} & {{16{1'b0}}, RF_rdata2[31:16]}
					  | {32{ALUOut[1:0]==2'b10}} & {{8{1'b0}}, RF_rdata2[31:8]}	
					  | {32{ALUOut[1:0]==2'b11}} & RF_rdata2;	
	assign SWR_data = {32{ALUOut[1:0]==2'b00}} & RF_rdata2//寄存器右侧写入字最不重要部分
					  | {32{ALUOut[1:0]==2'b01}} & {RF_rdata2[23:0], {8{1'b0}}}
					  | {32{ALUOut[1:0]==2'b10}} & {RF_rdata2[15:0], {16{1'b0}}}
					  | {32{ALUOut[1:0]==2'b11}} & {RF_rdata2[7:0], {24{1'b0}}};
	assign Write_data = {32{opcode[2:0]==3'b000}} & SB_data
					  | {32{opcode[2:0]==3'b001}} & SH_data
					  | {32{opcode[2:0]==3'b010}} & SWL_data
					  | {32{opcode[2:0]==3'b011}} & SW_data
					  | {32{opcode[2:0]==3'b110}} & SWR_data;
	assign MemtoReg_data  =   {32{opcode==6'b100000}} & {{24{LB_data[7]}}, LB_data}
							| {32{opcode==6'b100001}} & {{16{LH_data[15]}}, LH_data}
							| {32{opcode==6'b100100}} & {24'b0, LB_data}
							| {32{opcode==6'b100101}} & {16'b0, LH_data}
							| {32{opcode==6'b100010}} & LWL_data
							| {32{opcode==6'b100011}} & LW_data
							| {32{opcode==6'b100110}} & LWR_data;
	
    /*握手成功时MDR内容更新*/
    always @(posedge clk) begin 
        if(current_state[7] && Read_data_Valid && Read_data_Ready) begin
		    Memory_Data_reg <= MemtoReg_data;
        end
	end

	assign Address = {ALUOut[31:2], 2'b00};//内存地址需要相加的已放入ALU运算, 结果即为Address, 末尾保证地址对齐

    reg [31:0] cycle_cnt;//时钟周期
    always @(posedge clk) begin
        if (rst) begin
            cycle_cnt <= 32'd0;
        end
        else begin
            cycle_cnt <= cycle_cnt + 32'd1;
        end
    end
    assign cpu_perf_cnt_0 = cycle_cnt;

    reg [31:0] ID_cnt;//指令个数
    always @(posedge clk) begin
        if (rst) begin
            ID_cnt <= 32'd0;
        end
		else if(current_state[3]) begin
			ID_cnt <= ID_cnt +32'b1;
		end
    end
    assign cpu_perf_cnt_1 = ID_cnt;	

	reg [31:0] MemWrite_cnt;//内存写次数
    always @(posedge clk) begin
        if (rst) begin
            MemWrite_cnt <= 32'd0;
        end
		else if(current_state[6] && Mem_Req_Ready) begin
			MemWrite_cnt <= MemWrite_cnt +32'b1;
		end
    end
    assign cpu_perf_cnt_2 = MemWrite_cnt;	

	reg [31:0] MemRead_cnt;//内存读次数
    always @(posedge clk) begin
        if (rst) begin
            MemRead_cnt <= 32'd0;
        end
		else if(current_state[7] && Read_data_Valid) begin
			MemRead_cnt <= MemRead_cnt +32'b1;
		end
    end
    assign cpu_perf_cnt_3 = MemRead_cnt;

	reg [31:0] WB_cnt;//写回次数
    always @(posedge clk) begin
        if (rst) begin
            WB_cnt <= 32'd0;
        end
		else if(current_state[8]) begin
			WB_cnt <= WB_cnt +32'b1;
		end
    end
    assign cpu_perf_cnt_4 = WB_cnt;	

	reg [31:0] Rtype_cnt;//RType个数
    always @(posedge clk) begin
        if (rst) begin
            Rtype_cnt <= 32'd0;
        end
		else if(current_state[8] && Rtype) begin
			Rtype_cnt <= Rtype_cnt +32'b1;
		end
    end
    assign cpu_perf_cnt_5 = Rtype_cnt;	
endmodule
