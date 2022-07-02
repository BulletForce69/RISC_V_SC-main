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
	.OP_i(instruction_bus_w[6:0]),
	/** outputus**/
	.Branch_o(Branch_w),
	.ALU_Op_o(alu_op_w),
	.ALU_Src_o(alu_src_w),
	.Reg_Write_o(reg_write_w),
	.Mem_to_Reg_o(mem_to_reg_w),
	.Mem_Read_o(mem_read_w),
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
Multiplexor para decidir si el salto va a ser rs1 +imm o solo el imm, el selector es el cuarto bit del la instruccion
porque es el que cambia entre jal y jalr.
*/

wire JALR_f;
assign JALR_f = (instruction_bus_w[6:0] == 7'b1100111)? 1'b1:1'b0;


Multiplexer_2_to_1
#(
	.NBits(32)
)
MUX_JALR_OR_IMM
(
	.Selector_i(JALR_f),
	.Mux_Data_0_i(inmmediate_data_w),
	.Mux_Data_1_i(alu_result_w),
	
	.Mux_Output_o(Jal_Out_w)

);

/*
 sumador para agregar al PC lo necesario para hacer el salto segun sea el caso
*/

Adder_32_Bits
JUMP_ADDER
(
	.Data0(pc_w),
	.Data1(Jal_Out_w),
	
	.Result(pc_plus_jmp_w)
);

/*
Multiplexor para decirdir si el siguiente PC es +4 o viene de un salto
*/

Multiplexer_2_to_1
#(
	.NBits(32)
)
MUX_PC4_OR_JMP
(
	.Selector_i(Branch_Flag_w),
	.Mux_Data_0_i(pc_plus_4_w),
	.Mux_Data_1_i(pc_plus_jmp_w),
	
	.Mux_Output_o(Next_PC_w)

);

/*
unidad de control de branches que usa el func3 para distinguir el tipo de branch, basicamente es un multiplexor grande que decide si se puede o no
hacer el salto dependiendo del tipo y del resultado de la ALU
*/

Branch_Control
Branch_Control_Unit
(
	.Func_3(instruction_bus_w[14:12]),
	.Branch_i(Branch_w),
	.ALU_Result(alu_result_w),
	
	.Branch_Flag_o(Branch_Flag_w)
	
);


//******************************************************************/


Data_Memory
Data_Memory_Unit
(
	.clk(clk),
	.Mem_Write_i(mem_write_w),
	.Mem_Read_i(mem_read_w),
	.Write_Data_i(read_data_2_w),
	.Address_i(alu_result_w),

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
	.Selector_i(mem_to_reg_w),
	.Mux_Data_0_i(alu_result_w),
	.Mux_Data_1_i(Read_Mem_Data_w),
	
	.Mux_Output_o(ALU_OR_MEM_w)

);


Register_File
REGISTER_FILE_UNIT
(
	.clk(clk),
	.reset(reset),
	.Reg_Write_i(reg_write_w),
	.Write_Register_i(instruction_bus_w[11:7]),
	.Read_Register_1_i(instruction_bus_w[19:15]),
	.Read_Register_2_i(instruction_bus_w[24:20]),
	.Write_Data_i(ALU_OR_MEM_w),
	.Read_Data_1_o(read_data_1_w),
	.Read_Data_2_o(read_data_2_w)

);



Immediate_Unit
IMM_UNIT
(  .op_i(instruction_bus_w[6:0]),
   .Instruction_bus_i(instruction_bus_w),
   .Immediate_o(inmmediate_data_w)
);



Multiplexer_2_to_1
#(
	.NBits(32)
)
MUX_DATA_OR_IMM_FOR_ALU
(
	.Selector_i(alu_src_w),
	.Mux_Data_0_i(read_data_2_w),
	.Mux_Data_1_i(inmmediate_data_w),
	
	.Mux_Output_o(read_data_2_or_imm_w)

);


ALU_Control
ALU_CONTROL_UNIT
(
	.funct7_i(instruction_bus_w[30]),
	.ALU_Op_i(alu_op_w),
	.funct3_i(instruction_bus_w[14:12]),
	.ALU_Operation_o(alu_operation_w)

);



ALU
ALU_UNIT
(
	.ALU_Operation_i(alu_operation_w),
	.A_i(read_data_1_w),
	.B_i(read_data_2_or_imm_w),
	.Pc4(pc_plus_4_w),
	.Zero_o(Zero_Flag_w),
	.ALU_Result_o(alu_result_w)
);




endmodule

