module datapath #(parameter WIDTH = 32) (
	input clk, reset, PCSrc, RegWrite, MemWrite, MemtoReg, ALUSrc, rotate_control, RdSrc, WdSrc, after_shifter_select,
	input [1:0] ImmSrc, RegSrc,
	input [3:0] ALUControl, Debug_Source_select,
	output wire [WIDTH-1:0] Debug_out, INSTRUCTION, PC,
	output wire [3:0] ALUFlags
);

	wire [WIDTH-1:0] PCNot, PCPlus4, PCPlus8;
	wire [3:0] RA1, RA2, Rd_value;
	wire [WIDTH-1:0] SrcA, SrcB, SrcB1, ALUResult, SrcB_pre, shifter_res;
	wire [WIDTH-1:0] ReadData, WriteData, Result, Result_out;
	wire carry_in;
	wire [1:0] control_s_input; 
	wire [4:0] shamt_input;

	Register_reset #(.WIDTH(WIDTH)) PCreg ( 
		.clk(clk),
		.reset(reset),
		.DATA(PCNot),
		.OUT(PC)
		);
	
	Instruction_memory instruction_memory (
		.ADDR(PC),
		.RD(INSTRUCTION)
		);
	
	Memory data_memory (
		.clk(clk),
		.WE(MemWrite),
		.ADDR(ALUResult),
		.WD(WriteData),
		.RD(ReadData)
	);
	
	Register_file #(.WIDTH(WIDTH)) register_file (
		.clk(clk),
		.write_enable(RegWrite),
		.reset(1'b0),
		.Source_select_0(RA1),
		.Source_select_1(RA2),
		.Debug_Source_select(Debug_Source_select),
		.Destination_select(Rd_value),
		.DATA(Result_out),
		.Reg_15(PCPlus8),
		.out_0(SrcA),
		.out_1(WriteData),
		.Debug_out(Debug_out)	
	);
	
	Mux_2to1 #(.WIDTH(4)) mux_Rd (
	   .select(RdSrc),
	   .input_0(INSTRUCTION[15:12]),
	   .input_1(4'b1110),
	   .output_value(Rd_value)
	);
	
	Mux_2to1 #(.WIDTH(WIDTH)) mux_Wdata (
	   .select(WdSrc),
	   .input_0(Result),
	   .input_1(PCPlus4),
	   .output_value(Result_out)
	  );
	
	shifter #(.WIDTH(WIDTH)) shifter0 (
		.control(control_s_input),
		.shamt(shamt_input),
		.DATA(SrcB_pre),
		.OUT(shifter_res)
	);
	
	Mux_2to1 #(.WIDTH(WIDTH)) mux_SrcB_fin (
	   .select(after_shifter_select),
	   .input_0(shifter_res),
	   .input_1(SrcB_pre),
	   .output_value(SrcB)
	);
	
	ALU #(.WIDTH(WIDTH)) alu0  (
		.control(ALUControl),
		.DATA_A(SrcA),
		.DATA_B(SrcB),
		.OUT(ALUResult),
		.CI(carry_in),
		.CO(ALUFlags[1]),
		.OVF(ALUFlags[0]),
		.N(ALUFlags[3]),
		.Z(ALUFlags[2])
	);
	
	Extender extender (
		.Extended_data(SrcB1),
		.DATA(INSTRUCTION[23:0]),
		.select(ImmSrc)
	);
	
	Adder #(.WIDTH(WIDTH)) PCp4 (
		.DATA_A(PC),
		.DATA_B(32'd4),
		.OUT(PCPlus4)
		);
		
	Adder #(.WIDTH(WIDTH)) PCp8 (
		.DATA_A(PCPlus4),
		.DATA_B(32'd4),
		.OUT(PCPlus8)
		);
	
	Mux_2to1 #(.WIDTH(4)) mux_RA1 (
		.select(RegSrc[0]),
		.input_0(INSTRUCTION[19:16]),
		.input_1(4'b1111),
		.output_value(RA1)
		);
		
	Mux_2to1 #(.WIDTH(4)) mux_RA2 (
		.select(RegSrc[1]),
		.input_0(INSTRUCTION[3:0]),
		.input_1(INSTRUCTION[15:12]),
		.output_value(RA2)
		);
		
	Mux_2to1 #(.WIDTH(WIDTH)) mux_SrcB (
		.select(ALUSrc),
		.input_0(WriteData),
		.input_1(SrcB1),
		.output_value(SrcB_pre)
		);
		
	Mux_2to1 #(.WIDTH(WIDTH)) mux_reg (
		.select(MemtoReg),
		.input_0(ALUResult),
		.input_1(ReadData),
		.output_value(Result)
		);
		
	Mux_2to1 #(.WIDTH(WIDTH)) mux_PC (
		.select(PCSrc),
		.input_0(PCPlus4),
		.input_1(Result),
		.output_value(PCNot)
		);
		
	Mux_2to1 #(.WIDTH(5)) mux_shamt (
		.select(rotate_control),
		.input_0(INSTRUCTION[11:7]),
		.input_1({INSTRUCTION[11:8], 1'b0}),
		.output_value(shamt_input)
		);
		
	Mux_2to1 #(.WIDTH(2)) mux_control_s (
		.select(rotate_control),
		.input_0(INSTRUCTION[6:5]),
		.input_1(2'b11),
		.output_value(control_s_input)
		);
	

endmodule