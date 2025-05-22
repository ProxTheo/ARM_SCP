module Single_Cycle_Computer (
	input clk, reset,
	input [3:0] debug_reg_select,
	output [31:0] debug_reg_out, fetchPC
);

wire PCSrc, RegWrite, MemWrite, MemtoReg, ALUSrc, rotate_control, RdSrc, WdSrc, after_shifter_select;
wire [1:0] ImmSrc, RegSrc;
wire [3:0] ALUControl, ALUFlags;
wire [31:0] INSTRUCTION;


datapath my_datapath (
	.clk(clk),
	.reset(reset),
	.PCSrc(PCSrc),
	.RegWrite(RegWrite),
	.MemWrite(MemWrite),
	.MemtoReg(MemtoReg),
	.ALUSrc(ALUSrc),
	.rotate_control(rotate_control),
	.ImmSrc(ImmSrc),
	.RegSrc(RegSrc),
	.ALUControl(ALUControl),
	.Debug_Source_select(debug_reg_select),
	.Debug_out(debug_reg_out),
	.INSTRUCTION(INSTRUCTION),
	.PC(fetchPC),
	.ALUFlags(ALUFlags),
	.RdSrc(RdSrc),
	.WdSrc(WdSrc),
	.after_shifter_select(after_shifter_select)
	);

Decoder my_controller (
	.clk(clk),
	.Rd(INSTRUCTION[15:12]),
	.ALUFlags(ALUFlags),
	.Cond(INSTRUCTION[31:28]),
	.Op(INSTRUCTION[27:26]),
	.Funct(INSTRUCTION[25:20]),
	.PCSrc(PCSrc),
	.RegWrite(RegWrite),
	.MemWrite(MemWrite),
	.MemtoReg(MemtoReg),
	.ALUSrc(ALUSrc),
	.rotate_control(rotate_control),
	.ImmSrc(ImmSrc),
	.RegSrc(RegSrc),
	.ALUControl(ALUControl),
	.RdSrc(RdSrc),
	.WdSrc(WdSrc),
	.after_shifter_select(after_shifter_select)
	);

endmodule