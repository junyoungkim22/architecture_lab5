`include "opcodes.v"

module hazard_detection_unit (
	ID_EX_signal, 
	ID_EX_rd,  
	IF_ID_rs, 
	IF_ID_rt, 
	IF_ID_signal,
	stall
);
	input [`SIG_SIZE-1:0] ID_EX_signal;
	input [1:0] ID_EX_rd;
	input [1:0] IF_ID_rs;
	input [1:0] IF_ID_rt;
	input [`SIG_SIZE-1-1:0] IF_ID_signal;
	output stall;

	wire ID_EX_MemRead = ID_EX_signal[8];
	wire IF_ID_isBR = IF_ID_signal[9];
	wire IF_ID_isJPR = (IF_ID_signal[15:12] == 3);
	wire isNOP = (!ID_EX_signal);

	//implement cases where IF_ID rs rt are not used

	assign stall = isNOP ? 0 : ((ID_EX_MemRead || IF_ID_isBR || IF_ID_isJPR) ? ((ID_EX_rd == IF_ID_rs || ID_EX_rd == IF_ID_rt) ? 1 : 0) : 0);

endmodule					

