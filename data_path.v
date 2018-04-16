`include "opcodes.v" 
`include "alu.v"
`include "register_file.v"
//`include "alu_control.v"	   

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
	is_halted
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
	output reg [`WORD_SIZE-1:0] output_reg;
	output [`WORD_SIZE-1:0] instruction;
	input [`WORD_SIZE-1:0] PC;
	output [`WORD_SIZE-1:0] nextPC;
	input [11:0] signal;
	output is_halted;


	assign readM1 = 1;

	assign address1 = PC;

	//change!
	assign address2 = 0;
	assign writeM2 = 0;


	//** IF STAGE **//
	reg [`WORD_SIZE-1:0] IF_ID_ins;
	reg [`WORD_SIZE-1:0] IF_ID_nextPC;
	//** IF STAGE **//

	//** ID STAGE **//
	assign instruction = IF_ID_ins;
	wire [1:0] rs = IF_ID_ins[11:10];
	wire [1:0] rt = IF_ID_ins[9:8];

	//change!!
	wire [1:0] rd = 0;
	//assign rd = (RegDst == 2) ? 2 : (RegDst ? instruction[7:6] :  instruction[9:8]);


	wire [`WORD_SIZE-1:0] writeData;
	wire [`WORD_SIZE-1:0] readData1;
	wire [`WORD_SIZE-1:0] readData2;
	reg [`WORD_SIZE-1:0] ID_EX_ins;
	reg [`WORD_SIZE-1:0] ID_EX_signal;
	reg [`WORD_SIZE-1:0] ID_EX_readData1;
	reg [`WORD_SIZE-1:0] ID_EX_readData2;
	reg [`WORD_SIZE-1:0] ID_EX_sign_extended;
	reg [1:0] ID_EX_rs;
	reg [1:0] ID_EX_rt;
	reg [1:0] ID_EX_rd;
	wire [`WORD_SIZE-1:0] sign_extended = { {8{IF_ID_ins[7]}}, IF_ID_ins[7:0] };
	wire [`WORD_SIZE-1:0] jmp_target = {PC[15:12], IF_ID_ins[11:0]};

	//change!!
	wire regFileWrite = 0; 

	register_file regFile (rs, rt, rd, writeData, regFileWrite, readData1, readData2, clk, reset_n);
	//** ID STAGE END **//


	//** EX STAGE **//
	wire opcode = ID_EX_signal[3:0];
	ALU alu(A, B, OP, ALUOut, opcode, bcond);

	wire isJMP = signal[10];
	wire flush = isJMP;

	//change!
	assign nextPC = isJMP ? jmp_target : PC + 1;

	initial begin
		IF_ID_ins = 0;
		IF_ID_nextPC = 0;
	end

	// ** IF STAGE ** //
	always @ (posedge clk) begin
		if(flush) IF_ID_ins = `NOP;
		else IF_ID_ins <= data1;
		IF_ID_nextPC <= nextPC;
	end

	//** IF STAGE END **//


	// ** ID STAGE ** //

	always @ (posedge clk) begin
		ID_EX_ins <= IF_ID_ins;
		ID_EX_signal <= signal;
		ID_EX_readData1 <= readData1;
		ID_EX_readData2 <= readData2;
		ID_EX_sign_extended <= sign_extended;
		ID_EX_rs <= rs;
		ID_EX_rt <= rt;
		ID_EX_rd <= rd;
	end

	// ** ID STAGE END **//

	// ** EX STAGE **//







	/*

	reg [`WORD_SIZE-1:0] memData;
	wire [3:0] opcode = instruction [15:12];

	//check for specific instructions
	wire isJPR = (opcode == 15) ? (instruction[5:0] == 25 ? 1 : 0) : 0;
	wire isJRL = (opcode == 15) ? (instruction[5:0] == 26 ? 1 : 0) : 0;
	wire isJL = isJPR || isJRL;
	wire isHLT = (opcode == 15) ? (instruction[5:0] ==29 ? 1 : 0) : 0;
	assign is_halted = isHLT;

	//control signals
	wire [1:0] PCSource = signal[15:14];
	wire ALUOp = signal[13];
	wire [1:0] ALUSrcB = signal[12:11];
	wire ALUSrcA = signal[10];
	wire RegWrite = isHLT ? 0 : (isJPR ? 0 : signal[9]);
	wire [1:0] RegDst = signal[8:7];
	wire PCWriteCond = signal[6];
	wire PCWrite = isHLT ? 0 : signal[5];
	wire IorD = signal[4];
	wire MemRead = signal[3];
	wire MemWrite = signal[2];
	wire MemtoReg = signal[1];
	wire IRWrite = isHLT ? 0 : signal[0];

	//only going to use data2 for writing, so make data2 in memory be z
	wire readM2 = 0;

	//inputs and outputs for register file
	wire [1:0] r1 = instruction[11:10];
	wire [1:0] r2 = instruction[9:8];
	wire [1:0] rd;
	assign rd = (RegDst == 2) ? 2 : (RegDst ? instruction[7:6] :  instruction[9:8]);
	wire [`WORD_SIZE-1:0] writeData;
	wire [`WORD_SIZE-1:0] readData1;
	wire [`WORD_SIZE-1:0] readData2;

	//sign-extended instruction[7:0]
	wire [`WORD_SIZE-1:0] sign_extended = { {8{instruction[7]}}, instruction[7:0] };
	wire [`WORD_SIZE-1:0] zero_extended = {8'h00, instruction[7:0]};         //use for ori op

	//opcode
	assign opcode = instruction[15:12];

	//check if instruction if WWD
	wire isWWD = (opcode == 15) ? ((instruction[5:0] == 28) ? 1 : 0) : 0;

	//alu control output
	wire [3:0] alu_control_output;

	//inputs and outputs for regALU
	wire [`WORD_SIZE-1:0] A = ALUSrcA ? readData1 : PC;
	wire [`WORD_SIZE-1:0] beforeB = (ALUSrcB >= 2) ? ((ALUSrcB == 2) ? sign_extended : zero_extended) : ((ALUSrcB == 1) ? 1 : readData2);
	wire [`WORD_SIZE-1:0] B = isWWD ? (PCWrite ? 1 : 0) : (isJL ? 0 : beforeB);
	wire [3:0] OP = ALUOp ? (PCWrite ? 0 : alu_control_output) : (PCWriteCond ? 1 : 0);
	wire [`WORD_SIZE-1:0] ALUOut;
	reg [`WORD_SIZE-1:0] ALUOutReg;

	//output of ALU
	wire [`WORD_SIZE-1:0] calc_address = ALUOutReg;

	//assign writedata of alu
	assign writeData = MemtoReg ? memData : ALUOutReg;


	//jump logic
	wire [`WORD_SIZE-1:0] jumpTarget = {PC[15:12], instruction[11:0]};

	//branch logic
	wire bcond; 

	wire [`WORD_SIZE-1:0] calcPC = (PCSource <= 1) ? ((PCSource) ? ALUOutReg : ALUOut) : jumpTarget;
	wire updatePC = (bcond && PCWriteCond) || PCWrite || (signal[9] && isJPR);

	assign nextPC = updatePC ? (isJPR ? ALUOut : calcPC) : PC;


	assign readM1 = MemRead;
	assign address1 = (!IorD) ? PC : calc_address;
	assign writeM2 = MemWrite ? 1 : 0;
	assign address2 = calc_address;
	assign data2 = readData2;

	//don't write if WWD instruction
	wire regFileWrite = isWWD ? 0 : RegWrite;

	alu_control AC(instruction, alu_control_output);
	register_file regFile (r1, r2, rd, writeData, regFileWrite, readData1, readData2, clk, reset_n);
	ALU alu(A, B, OP, ALUOut, opcode, bcond);

	initial begin
		memData <= 0;
		ALUOutReg <= 0;
		instruction <= 0;
	end

	always @ (posedge clk) begin
		if(!reset_n) begin
			memData <= 0;
			ALUOutReg <= 0;
			instruction <= 0;
			
		end
		else begin
			if(MemRead) begin
				if(!IorD) begin
					if(IRWrite) begin
						instruction <= data1;
					end
				end
				else begin
					if(MemRead) memData <= data1;
				end
			end
			if(RegWrite) begin
				if(isWWD) output_reg = ALUOut;
			end
			if(ALUOp) begin
				if(opcode == `JAL_OP) ALUOutReg <= ALUOut;
				else if(isJRL) ALUOutReg <= ALUOut + 1;
				else ALUOutReg <= ALUOut;
			end
		end
	end
	*/
	
endmodule				
