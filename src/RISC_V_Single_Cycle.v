/******************************************************************
* Description
*	This is the top-level of a RISC-V Microprocessor that can execute the next set of instructions:
*		add
*		addi
* This processor is written Verilog-HDL. It is synthesizabled into hardware.
* Parameter MEMORY_DEPTH configures the program memory to allocate the program to
* be executed. If the size of the program changes, thus, MEMORY_DEPTH must change.
* This processor was made for computer organization class at ITESO.
* Version:
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	16/08/2021
******************************************************************/

module RISC_V_Single_Cycle
#(
	parameter PROGRAM_MEMORY_DEPTH = 64,
	parameter DATA_MEMORY_DEPTH = 256
)

(
	// Inputs
	input clk,
	input reset,
	output [31:0]RegTest_o

);
//******************************************************************/
//******************************************************************/

//******************************************************************/
//******************************************************************/
/* Signals to connect modules*/



/**Control**/
wire alu_src_w;
wire reg_write_w;
wire mem_to_reg_w;
wire mem_write_w;
wire mem_read_w;
wire [2:0] alu_op_w;
wire Branch_w;

/** Program Counter**/
wire [31:0] pc_plus_4_w;
wire [31:0] pc_w;
wire [31:0] pc_plus_jmp_w;
wire [31:0] Next_PC_w;

/** Memory **/
wire [31:0] Read_Mem_Data_w;
wire [31:0] ALU_OR_MEM_w;


/**Register File**/
wire [31:0] read_data_1_w;
wire [31:0] read_data_2_w;

/**Inmmediate Unit**/
wire [31:0] inmmediate_data_w;

/**ALU**/
wire [31:0] alu_result_w;
wire Zero_Flag_w;

/**Multiplexer MUX_DATA_OR_IMM_FOR_ALU**/
wire [31:0] read_data_2_or_imm_w;

/**ALU Control**/
wire [3:0] alu_operation_w;

/**Instruction Bus**/	
wire [31:0] instruction_bus_w;
/**Branch/JAL**/ //wires para controlar los saltos
wire Branch_Flag_w;
wire Jal_Out_w;
wire [31:0] JALR_Res_w;
wire JALR_f;

/** Pipeline Wires **/

wire [31:0] Pipe_PC_w;
wire [31:0] Pipe_Instr_w;

wire [31:0] Pipe2_PC_w;
wire [31:0] Pipe_Rd1_w;
wire [31:0] Pipe_Rd2_w;
wire [31:0] Pipe_Imm_w;

wire [31:0] Pipe_PCJMP_w;
wire [31:0] Pipe_ALURes_w;
wire [31:0] Pipe3_Rd2_w;

wire [31:0] Pipe_RdData_w;
wire [31:0] Pipe4_ALURes_w;

// Pipeline Control Signal Wires
wire [2:0] pipe2_ALUOP_w;

wire pipe2_ALUSrc_w;

wire pipe2_MemWr_w;
wire pipe3_MemWr_w;

wire pipe2_MemRd_w;
wire pipe3_MemRd_w;

wire pipe2_Branch_w;
wire pipe3_Branch_w;

wire pipe2_JALR_w;
wire pipe3_JALR_w;

wire pipe2_RegW_w;
wire pipe3_RegW_w;
wire pipe4_RegW_w;

wire pipe2_MemReg_w;
wire pipe3_MemReg_w;
wire pipe4_MemReg_w;










assign RegTest_o = read_data_1_w;


//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
Control
CONTROL_UNIT
(
	/****/
	.OP_i(Pipe_Instr_w[6:0]),
	/** outputus**/
	.Branch_o(Branch_w),
	.ALU_Op_o(alu_op_w),
	.ALU_Src_o(alu_src_w),
	.Reg_Write_o(reg_write_w),
	.Mem_to_Reg_o(mem_to_reg_w),
	.Mem_Read_o(mem_read_w),
	.JALR_F_o(JALR_f),
	.Mem_Write_o(mem_write_w)
);


PC_Register
PC(
	.clk(clk),
	.reset(reset),
	.Next_PC(Next_PC_w),

	.PC_Value(pc_w)
);


Program_Memory
#(
	.MEMORY_DEPTH(PROGRAM_MEMORY_DEPTH)
)
PROGRAM_MEMORY
(
	.Address_i(pc_w),
	
	.Instruction_o(instruction_bus_w)
);


Adder_32_Bits
PC_PLUS_4
(
	.Data0(pc_w),
	.Data1(4),
	
	.Result(pc_plus_4_w)
);


/*
 Sumador para agregar al PC lo necesario para hacer el salto segun sea el caso
*/
Adder_32_Bits
JUMP_ADDER
(
	.Data0(Pipe2_PC_w),
	.Data1(Pipe_Imm_w),
	
	.Result(pc_plus_jmp_w)
);
// Sumador para obtener el salto de JALR
Adder_32_Bits
JALR_ADDER
(
	.Data0(Pipe_Rd1_w),
	.Data1(Pipe_Imm_w),
	
	.Result(JALR_Res_w)
);

// Mux para ver si el salto es Pc + Imm o viene de un JALR Rs1 +Imm
Multiplexer_2_to_1
#(
	.NBits(32)
)
MUX_JALR_OR_IMM
(
	.Selector_i(pipe3_JALR_w),
	.Mux_Data_0_i(pc_plus_jmp_w),
	.Mux_Data_1_i(JALR_Res_w),
	
	.Mux_Output_o(Jmp_Out_w)

);


/*
Multiplexor para decirdir si el siguiente PC es + 4 o viene de un salto
*/

Multiplexer_2_to_1
#(
	.NBits(32)
)
MUX_PC4_OR_JMP
(
	.Selector_i(Branch_Flag_w),
	.Mux_Data_0_i(pc_plus_4_w),
	.Mux_Data_1_i(Jmp_Out_w),
	
	.Mux_Output_o(Pipe_PCJMP_w)

);

/*
unidad de control de branches que usa el func3 para distinguir el tipo de branch, basicamente es un multiplexor grande que decide si se puede o no
hacer el salto dependiendo del tipo y del resultado de la ALU
*/

Branch_Control
Branch_Control_Unit
(
	.Func_3(Pipe_Instr_w[14:12]),
	.Branch_i(pipe3_Branch_w),
	.ALU_Result(Pipe_ALURes_w),
	
	.Branch_Flag_o(Branch_Flag_w)
	
);


//******************************************************************/


Data_Memory
#(.DATA_WIDTH(32),
.MEMORY_DEPTH (DATA_MEMORY_DEPTH))
Data_Memory_Unit
(
	.clk(clk),
	.Mem_Write_i(pipe3_MemWr_w),
	.Mem_Read_i(pipe3_MemRd_w),
	.Write_Data_i(Pipe3_Rd2_w),
	.Address_i(Pipe_ALURes_w),

	.Read_Data_o(Read_Mem_Data_w)
);

/*
Mux para decidir si al registro se debe escribir el resultado de la ALU o lo que sale de memoria.
*/

Multiplexer_2_to_1
#(
	.NBits(32)
)
MUX_ALU_OR_MEM_OUT
(
	.Selector_i(pipe4_MemReg_w),
	.Mux_Data_0_i(Pipe4_ALURes_w),
	.Mux_Data_1_i(Pipe_RdData_w),
	
	.Mux_Output_o(ALU_OR_MEM_w)

);


Register_File
REGISTER_FILE_UNIT
(
	.clk(clk),
	.reset(reset),
	.Reg_Write_i(pipe4_RegW_w),
	.Write_Register_i(Pipe_Instr_w[11:7]),
	.Read_Register_1_i(Pipe_Instr_w[19:15]),
	.Read_Register_2_i(Pipe_Instr_w[24:20]),
	.Write_Data_i(ALU_OR_MEM_w),
	
	.Read_Data_1_o(read_data_1_w),
	.Read_Data_2_o(read_data_2_w)

);



Immediate_Unit
IMM_UNIT
(  .op_i(Pipe_Instr_w[6:0]),
   .Instruction_bus_i(Pipe_Instr_w),
	
   .Immediate_o(inmmediate_data_w)
);



Multiplexer_2_to_1
#(
	.NBits(32)
)
MUX_DATA_OR_IMM_FOR_ALU
(
	.Selector_i(pipe2_ALUSrc_w),
	.Mux_Data_0_i(Pipe_Rd2_w),
	.Mux_Data_1_i(Pipe_Imm_w),
	
	.Mux_Output_o(read_data_2_or_imm_w)

);


ALU_Control
ALU_CONTROL_UNIT
(
	.funct7_i(Pipe_Instr_w[30]),
	.ALU_Op_i(pipe2_ALUOP_w),
	.funct3_i(Pipe_Instr_w[14:12]),
	
	.ALU_Operation_o(alu_operation_w)

);



ALU
ALU_UNIT
(
	.ALU_Operation_i(alu_operation_w),
	.A_i(Pipe_Rd1_w),
	.B_i(read_data_2_or_imm_w),
	.Pc4(pc_plus_4_w),
	.Zero_o(Zero_Flag_w),
	.ALU_Result_o(alu_result_w)
);


/*
	IF/ID Pipeline ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

Register_Pipeline
IF_ID_PC
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(pc_w),
	
	
	.DataOutput(Pipe_PC_w)
);
Register_Pipeline
IF_ID_Instr
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(instruction_bus_w),
	
	
	.DataOutput(Pipe_Instr_w)
);


/*
	ID/EX Pipeline2 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

Register_Pipeline
ID_EX_PC
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(Pipe_PC_w),
	
	
	.DataOutput(Pipe2_PC_w)
);

Register_Pipeline
ID_EX_RD1
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(read_data_1_w),
	
	
	.DataOutput(Pipe_Rd1_w)
);

Register_Pipeline
ID_EX_RD2
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(read_data_2_w),
	
	
	.DataOutput(Pipe_Rd2_w)
);

Register_Pipeline
ID_EX_IMM
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(inmmediate_data_w),
	
	
	.DataOutput(Pipe_Imm_w)
);

// 		Señales de control
Register_Pipeline
ID_EX_ALUOP
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(alu_op_w),
	
	
	.DataOutput(pipe2_ALUOP_w)
);

Register_Pipeline
ID_EX_ALUSRC
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(alu_src_w),
	
	
	.DataOutput(pipe2_ALUSrc_w)
);

Register_Pipeline
ID_EX_MEMWR
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(mem_write_w),
	
	
	.DataOutput(pipe2_MemWr_w)
);

Register_Pipeline
ID_EX_MEMRD
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(mem_read_w),
	
	
	.DataOutput(pipe2_MemRd_w)
);

Register_Pipeline
ID_EX_BRANCH
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(Branch_w),
	
	
	.DataOutput(pipe2_Branch_w)
);

Register_Pipeline
ID_EX_JALR
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(JALR_f),
	
	
	.DataOutput(pipe2_JALR_w)
);

Register_Pipeline
ID_EX_REGW
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(reg_write_w),
	
	
	.DataOutput(pipe2_RegW_w)
);

Register_Pipeline
ID_EX_MEWMREG
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(mem_to_reg_w),
	
	
	.DataOutput(pipe2_MemReg_w)
);


/*
	Ex/MEM Pipeline3 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

Register_Pipeline
EX_MEM_PC
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(Jmp_Out_w),
	
	
	.DataOutput(Pipe_PCJMP_w)
);

Register_Pipeline
EX_MEM_ALU_RES
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(alu_result_w),
	
	
	.DataOutput(Pipe_ALURes_w)
);

Register_Pipeline
EX_MEM_RD2
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(Pipe_Rd2_w),
	
	
	.DataOutput(Pipe3_Rd2_w)
);

// 		Señales de control (3)
Register_Pipeline
EX_MEM_MEMWR
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(pipe2_MemWr_w),
	
	
	.DataOutput(pipe3_MemWr_w)
);

Register_Pipeline
EX_MEM_MEMRD
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(pipe2_MemRd_w),
	
	
	.DataOutput(pipe3_MemRd_w)
);

Register_Pipeline
EX_MEM_BRANCH
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(pipe2_Branch_w),
	
	
	.DataOutput(pipe3_Branch_w)
);

Register_Pipeline
EX_MEM_JALR
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(pipe2_JALR_w),
	
	
	.DataOutput(pipe3_JALR_w)
);

Register_Pipeline
EX_MEM_REGW
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(pipe2_RegW_w),
	
	
	.DataOutput(pipe3_RegW_w)
);

Register_Pipeline
EX_MEM_MEMREG
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(pipe2_MemReg_w),
	
	
	.DataOutput(pipe3_MemReg_w)
);

/*
	MEM/WB Pipeline4 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
Register_Pipeline
MEM_WB_RDATA
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(Read_Mem_Data_w),
	
	
	.DataOutput(Pipe_RdData_w)
);

Register_Pipeline
MEM_WB_ALU_RES
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(Pipe_ALURes_w),
	
	
	.DataOutput(Pipe4_ALURes_w)
);

// 		Señales de control (4)

Register_Pipeline
MEM_WB_REGW
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(pipe3_RegW_w),
	
	
	.DataOutput(pipe4_RegW_w)
);

Register_Pipeline
MEM_WB_MEMREG
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput(pipe3_MemReg_w),
	
	
	.DataOutput(pipe4_MemReg_w)
);


endmodule

