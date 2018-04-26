`include "opcodes.v"
`define BTB_SIZE 256	//	size of memory is 2^8 words (reduced size)
`define IDX_SIZE 8
						 
module btb(IF_PC, IF_ID_PC, br_target, jmp_target, clk, is_BR, is_JMP, nextPC, reset_n);

	input [`WORD_SIZE-1:0] IF_ID_PC;
	input [`WORD_SIZE-1:0] br_target;
	input [`WORD_SIZE-1:0] jmp_target;
	input clk;
	input is_BR;
	input is_JMP;					   
	input [`WORD_SIZE-1:0] IF_PC;
	output [`WORD_SIZE-1:0] nextPC;
	input reset_n;
	//output reg bcond;

	reg [`BTB_ENTRY_SIZE-1:0] target_table [0:`BTB_SIZE-1];
	reg i;

	wire [`IDX_SIZE-1:0] IF_ID_tag = IF_ID_PC[15:8];
	wire [`IDX_SIZE-1:0] IF_ID_idx = IF_ID_PC[7:0];
	wire [`IDX_SIZE-1:0] IF_tag = IF_PC[15:8];
	wire [`IDX_SIZE-1:0] IF_idx = IF_PC[7:0];

	wire btb_hit = (IF_tag == target_table[IF_idx][`BTB_ENTRY_SIZE-1:`BTB_ENTRY_SIZE-8]);
	assign nextPC = btb_hit ? target_table[IF_idx][`BTB_ENTRY_SIZE-9:0] : IF_PC + 1;

	always @ (posedge clk) begin
		if(!reset_n) begin
			for(i = 0; i < `BTB_SIZE; i = i + 1)
				target_table[i] <= `BTB_ENTRY_SIZE'h110000;
		end
		else begin
			if(is_BR) begin
				target_table[IF_ID_idx] <= {IF_ID_tag, br_target};
			end
			if(is_JMP) begin
				target_table[IF_ID_idx] <= {IF_ID_tag, jmp_target};
			end
		end
	end
	
endmodule


