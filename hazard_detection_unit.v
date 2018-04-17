`include "opcodes.v"

module hazard_detection_unit (
	ID_EX_MemRead, 
	ID_EX_rd,  
	IF_ID_rs, 
	IF_ID_rt, 
	IF_ID_ins,
	stall
);
	input ID_EX_MemRead;
	input [1:0] ID_EX_rd;
	input [1:0] IF_ID_rs;
	input [1:0] IF_ID_rt;
	input [`WORD_SIZE-1:0] IF_ID_ins;
	output stall;

	assign stall = ID_EX_MemRead ? ((ID_EX_rd == IF_ID_rs || ID_EX_rd == IF_ID_rt) ? 1 : 0) : 0;

endmodule					

