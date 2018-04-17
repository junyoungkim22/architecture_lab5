`include "opcodes.v" 
`include "alu.v"
`include "register_file.v"
`include "forwarding_unit.v"
`include "hazard_detection_unit.v"
`include "br_resolve_unit.v"	   

module data_path (
	clk,
	reset_n,
	readM1,
	address1,
	data1,
	readM2,
	writeM2,
	address2,
	data2,
	output_reg,
	instruction,
	PC,
	nextPC,
	signal,
	is_halted,
	num_inst
);

	input clk;
	input reset_n;
	output readM1;
	output [`WORD_SIZE-1:0] address1;
	output readM2;
	output writeM2;
	output [`WORD_SIZE-1:0] address2;
	input [`WORD_SIZE-1:0] data1;
	inout [`WORD_SIZE-1:0] data2;
	output wire [`WORD_SIZE-1:0] output_reg;
	output [`WORD_SIZE-1:0] instruction;
	input [`WORD_SIZE-1:0] PC;
	output [`WORD_SIZE-1:0] nextPC;
	input [`SIG_SIZE-1:0] signal;
	output is_halted;
	output [`WORD_SIZE-1:0] num_inst;


	assign readM1 = 1;

	assign address1 = PC;
	reg [`WORD_SIZE-1:0] num_inst_counter;

	assign num_inst = num_inst_counter;

	//change!
	//assign address2 = 0;
	//assign writeM2 = 0;


	//** IF STAGE **//
	reg [`WORD_SIZE-1:0] IF_ID_ins;
	reg [`WORD_SIZE-1:0] IF_ID_nextPC;
	//** IF STAGE **//

	//** ID STAGE **//
	assign instruction = IF_ID_ins;
	wire [1:0] rs = IF_ID_ins[11:10];
	wire [1:0] rt = IF_ID_ins[9:8];
	wire stall;
	wire [1:0] rd;
	wire isBR = signal[9];

	wire [`WORD_SIZE-1:0] writeData;
	wire [`WORD_SIZE-1:0] readData1;
	wire [`WORD_SIZE-1:0] readData2;
	reg [`WORD_SIZE-1:0] ID_EX_ins;
	reg [`SIG_SIZE-1:0] ID_EX_signal;
	reg [`WORD_SIZE-1:0] ID_EX_readData1;
	reg [`WORD_SIZE-1:0] ID_EX_readData2;
	reg [`WORD_SIZE-1:0] ID_EX_sign_extended;
	reg [1:0] ID_EX_rs;
	reg [1:0] ID_EX_rt;
	reg [1:0] ID_EX_rd;
	wire [`WORD_SIZE-1:0] sign_extended = { {8{IF_ID_ins[7]}}, IF_ID_ins[7:0] };
	wire [`WORD_SIZE-1:0] jmp_target = {PC[15:12], IF_ID_ins[11:0]};

	//change!!
	wire regFileWrite; 

	wire [`WORD_SIZE-1:0] forward_readData1 = readData1;
	wire [`WORD_SIZE-1:0] forward_readData2 = readData2;

	wire bcond;

	br_resolve_unit BR_RES (forward_readData1, forward_readData2, ID_EX_ins, bcond);
	register_file regFile (rs, rt, rd, writeData, regFileWrite, readData1, readData2, !clk, reset_n);
	//** ID STAGE END **//


	//** EX STAGE **//
	wire [3:0] OP = ID_EX_signal[3:0];
	wire [3:0] isLHI = ID_EX_signal[15:12];
	wire [3:0] opcode = ID_EX_ins[15:12];

	//change!!
	wire ALUSrc = ID_EX_signal[5];
	//wire [`WORD_SIZE-1:0] forwardA = ID_EX_readData1;
	wire [`WORD_SIZE-1:0] forwardA;
	wire [`WORD_SIZE-1:0] A = (isLHI == 4'b0001) ? 0 : forwardA;
	wire [`WORD_SIZE-1:0] forwardB;
	//wire [`WORD_SIZE-1:0] forwardB = ID_EX_readData2;
	wire [`WORD_SIZE-1:0] B = ALUSrc ? ID_EX_sign_extended : forwardB;
	wire [`WORD_SIZE-1:0] ALUOut;
	ALU alu(A, B, OP, ALUOut, opcode);

	wire RegDst = ID_EX_signal[11];
	reg [`WORD_SIZE-1:0] EX_MEM_rs;
	reg [`WORD_SIZE-1:0] EX_MEM_rt;
	reg [`WORD_SIZE-1:0] EX_MEM_ALUout;
	reg [1:0] EX_MEM_rd;
	reg [`WORD_SIZE-1:0] EX_MEM_ins;
	reg [`SIG_SIZE-1:0] EX_MEM_sig;
	// ** EX STAGE END **//

	//** MEM STAGE **//
	wire MemRead = EX_MEM_sig[8];
	wire MemWrite = EX_MEM_sig[6];
	assign data2 = MemRead ? `WORD_SIZE'bz : (MemWrite ? EX_MEM_rt : 0);
	assign readM2 = MemRead ? 1 : 0;
	assign writeM2 = MemWrite ? 1 : 0;
	reg [`WORD_SIZE-1:0] MEM_WB_ins;
	reg [`SIG_SIZE-1:0] MEM_WB_sig;
	reg [`WORD_SIZE-1:0] MEM_WB_data;
	reg [`WORD_SIZE-1:0] MEM_WB_ALUout;
	reg [`WORD_SIZE-1:0] MEM_WB_rs;
	reg [`WORD_SIZE-1:0] MEM_WB_rt;
	reg [1:0] MEM_WB_rd;
	assign address2 = EX_MEM_ALUout;
	//** MEM STAGE END **//

	//** WB STAGE **//
	wire MemtoReg = MEM_WB_sig[7];
	wire RegWrite = MEM_WB_sig[4];
	reg [`WORD_SIZE-1:0] WWD_output;
	assign writeData = MemtoReg ? MEM_WB_data : MEM_WB_ALUout;
	assign regFileWrite = RegWrite;
	//assign output_reg = MEM_WB_rs;
	assign output_reg = WWD_output;
	assign rd = MEM_WB_rd;
	//** WB STAGE END **//


	wire [1:0] f_A;
	wire [1:0] f_B;


	forwarding_unit FOW (EX_MEM_sig[4], EX_MEM_rd, MEM_WB_sig[4], MEM_WB_rd, ID_EX_rs, ID_EX_rt, f_A, f_B);
	assign forwardA = (f_A == 2'b10) ? EX_MEM_ALUout : ((f_A ==2'b01) ? writeData : ID_EX_readData1);
	assign forwardB = (f_B == 2'b10) ? EX_MEM_ALUout : ((f_B == 2'b01) ? writeData : ID_EX_readData2);
	wire isJMP = signal[10];
	wire flush = isJMP || (isBR && bcond);

	hazard_detection_unit HAZ(ID_EX_signal[8], RegDst ? ID_EX_rd : ID_EX_rt, rs, rt, IF_ID_ins, stall);

	//change!
	assign nextPC = stall ? PC : ((isBR && bcond) ? jmp_target : (isJMP ? jmp_target : PC + 1));

	initial begin
		IF_ID_ins <= `NOP;
		IF_ID_nextPC <= 0;
		num_inst_counter <= 0;
		IF_ID_ins <= `NOP;
		ID_EX_ins <= `NOP;
		ID_EX_signal <= `SIG_SIZE'b0;
	end


	// ** IF STAGE ** //
	always @ (posedge clk) begin
		if(flush) IF_ID_ins <= `NOP;
		if (!stall && !flush) begin 
			IF_ID_ins <= data1;
			IF_ID_nextPC <= nextPC;
		end
	end
	//** IF STAGE END **//



	// ** ID STAGE ** //
	always @ (posedge clk) begin
		if(!stall) begin
			ID_EX_ins <= IF_ID_ins;
			ID_EX_signal <= signal;
			ID_EX_readData1 <= readData1;
			ID_EX_readData2 <= readData2;
			ID_EX_sign_extended <= sign_extended;
			ID_EX_rs <= rs;
			ID_EX_rt <= rt;
			ID_EX_rd <= IF_ID_ins[7:6];
		end
		if(stall) begin
			ID_EX_ins <= `NOP;
			ID_EX_signal <= `SIG_SIZE'b0;
		end
	end
	// ** ID STAGE END **//


	// ** EX STAGE **//
	always @ (posedge clk) begin
		EX_MEM_rs <= forwardA;
		EX_MEM_rt <= forwardB;
		EX_MEM_rd <= RegDst ? ID_EX_rd : ID_EX_rt;
		EX_MEM_ALUout <= ALUOut;
		EX_MEM_ins <= ID_EX_ins;
		EX_MEM_sig <= ID_EX_signal;
	end
	// ** EX STAGE END **//

	// ** MEM STAGE **//
	always @ (posedge clk) begin
		MEM_WB_ins <= EX_MEM_ins;
		MEM_WB_sig <= EX_MEM_sig;
		MEM_WB_rs <= EX_MEM_rs;
		MEM_WB_rt <= EX_MEM_rt;
		MEM_WB_rd <= EX_MEM_rd;
		MEM_WB_ALUout <= EX_MEM_ALUout;
		MEM_WB_data <= data2;
	end

	// ** WB STAGE **//
	always @ (posedge clk) begin
		if(MEM_WB_sig) num_inst_counter <= num_inst_counter + 1;
		WWD_output = MEM_WB_rs;
	end
	// ** WB STAGE END **//
	
endmodule				
