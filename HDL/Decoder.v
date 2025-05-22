module Decoder (
	input clk,
	input [3:0] Rd, ALUFlags, Cond,
	input [1:0] Op,
	input [5:0] Funct,
	output PCSrc, RegWrite, MemWrite, MemtoReg, ALUSrc, rotate_control, after_shifter_select,
	output reg RdSrc, WdSrc, 
	output [1:0] RegSrc,
	output reg [1:0] ImmSrc,
	output reg [3:0] ALUControl
);

wire DP, MEM, BRANCH;
wire ALUop;
reg NoWrite;
wire [3:0] cmd;
wire DPReg, DPImm, I, S;
wire PCS, RegW, MemW;

// Instruction types
assign DP = ~Op[1] && ~Op[0];
assign MEM = ~Op[1] && Op[0];
assign BRANCH = Op[1] && ~Op[0];

// Decoding of the instruction
assign DPReg = ~I && DP;
assign DPImm = I && DP;
assign cmd = Funct[4:1];
assign I = Funct[5];
assign S = Funct[0];

// Decoding based write and source enables
assign ALUop = DP;
assign RegW = (DP || (MEM && S) || (BRANCH && cmd[3])) && ~(DP && (cmd == 4'b1001));
assign MemtoReg = (MEM && S);
assign MemW = (MEM && ~S);
assign ALUSrc = ~DPReg;
assign RegSrc[0] = BRANCH;
assign RegSrc[1] = (MEM && ~S);
assign PCS = BRANCH || ((Rd == 4'b1111) && RegW) || (DP && (cmd == 4'b1001));
assign after_shifter_select = DP && (cmd == 4'b1001);

// Extender Logic
always @(*) begin
	if (DPImm) begin
		ImmSrc = 2'b00;
	end else if (MEM) begin
		ImmSrc = 2'b01;
	end else if (BRANCH) begin
		ImmSrc = 2'b10;
	end else begin
		ImmSrc = 2'b00;
	end
end

// ALUControl Assignment, with a special case for CMP and BX
always @(*) begin
	if (ALUop) begin
		if (cmd == 4'b1010) begin
			ALUControl = 4'b0010; // SUB for CMP
			NoWrite = 1'b1;
		end else if (cmd == 4'b1001) begin
			ALUControl = 4'b1101; // To Move the Rm to SrcB
			NoWrite = 1'b0;
		end else begin
			ALUControl = cmd;
			NoWrite = 1'b0;
		end
	end else if (BRANCH) begin
		    ALUControl = 4'b0100;
		    NoWrite = 1'b0;
    end else if (MEM) begin
        ALUControl = 4'b0100;
        NoWrite = 1'b0;
	end else begin
		ALUControl = 4'b0000;
		NoWrite = 1'b0;
	end
end

always @(*) begin
    if (BRANCH && cmd[3]) begin
        WdSrc = 1'b1;
        RdSrc = 1'b1;
    end else begin
        RdSrc = 1'b0;
        WdSrc = 1'b0;
    end
end


// RR case 
assign rotate_control = DP && I && (cmd == 4'b1101);

// N, Z, C, V ( Flags order)

reg CondEx;
wire Z, C, V, N;

// Registers with enable for flag setting

Register_en #(.WIDTH(1)) N_reg (
	.clk(clk),
	.en(FlagW[1]),
	.DATA(ALUFlags[3]),
	.OUT(N)
	);

Register_en #(.WIDTH(1)) Z_reg (
	.clk(clk),
	.en(FlagW[1]),
	.DATA(ALUFlags[2]),
	.OUT(Z)
	);

Register_en #(.WIDTH(1)) C_reg (
	.clk(clk),
	.en(FlagW[0]),
	.DATA(ALUFlags[1]),
	.OUT(C)
	);

Register_en #(.WIDTH(1)) V_reg (
	.clk(clk),
	.en(FlagW[0]),
	.DATA(ALUFlags[0]),
	.OUT(V)
	);
	
	
// Conditional Check

always @(*) begin
	case (Cond)
		4'h0: CondEx = Z;
		4'h1: CondEx = ~Z;
		4'h2: CondEx = C;
		4'h3: CondEx = ~C;
		4'h4: CondEx = N;
		4'h5: CondEx = ~N;
		4'h6: CondEx = V;
		4'h7: CondEx = ~V;
		4'h8: CondEx = ~Z && C;
		4'h9: CondEx = Z || ~C;
		4'ha: CondEx = ~(N ^ V);
		4'hb: CondEx = N ^ V;
		4'hc: CondEx = ~Z && ~(N ^ V);
		4'hd: CondEx = Z || (N ^ V);
		4'he: CondEx = 1'b1;
		default: CondEx = 1'b0;
	endcase
end

// Write enables with conditional checks

assign PCSrc = CondEx && PCS;
assign RegWrite = CondEx && RegW && ~NoWrite;
assign MemWrite = CondEx && MemW;

// FlagW logic
wire [1:0] FlagW;

assign FlagW[1] = (DP && S);
assign FlagW[0] = (DP && S) && ((cmd == 4'b0100) || (cmd == 4'b0010) || (cmd == 4'b1010));

endmodule