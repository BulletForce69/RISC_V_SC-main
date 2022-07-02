/******************************************************************
* Description
*	This is the control unit for the ALU. It receves a signal called 
*	ALUOp from the control unit and signals called funct7 and funct3  from
*	the instruction bus.
* Version:
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	16/08/2021
******************************************************************/
module ALU_Control
(
	input funct7_i, 
	input [2:0] ALU_Op_i,
	input [2:0] funct3_i,
	

	output [3:0] ALU_Operation_o

);
//Para indicar a la ALU que operaciones debes realizar
localparam R_Type_ADD 	= 7'b0_000_000;
localparam R_Type_SUB 	= 7'b1_000_000;
localparam R_Type_XOR 	= 7'b0_000_100;
localparam R_Type_OR 	= 7'b0_000_110;
localparam R_Type_AND 	= 7'b0_000_111;
localparam R_Type_SLL 	= 7'b0_000_001;
localparam R_Type_SRL 	= 7'b0_000_101;

localparam I_Type_ADDI 	= 7'bx_001_000;
localparam I_Type_XORI 	= 7'bx_001_100;
localparam I_Type_ORI	= 7'bx_001_110;
localparam I_Type_ANDI	= 7'bx_001_111;
localparam I_Type_SLLI	= 7'b0_001_001;
localparam I_Type_SRLI	= 7'b0_001_101;
localparam I_Type_LD		= 7'bx_001_010;

localparam B_Type_BRCH 	= 7'bx_111_xxx;
localparam S_Type_STR 	= 7'bx_110_010;

localparam J_Type_JAL 	= 7'bx_101_xxx;

localparam U_Type_LUI	= 7'bx_100_xxx;

reg [3:0] alu_control_values;
wire [6:0] selector;

assign selector = {funct7_i, ALU_Op_i, funct3_i};

always@(selector)begin
	casex(selector)
		R_Type_ADD:		alu_control_values = 4'b0000;
		R_Type_SUB:		alu_control_values = 4'b0001;
		R_Type_XOR:		alu_control_values = 4'b0010;
		R_Type_OR:		alu_control_values = 4'b0011;
		R_Type_AND:		alu_control_values = 4'b0100;
		R_Type_SLL:		alu_control_values = 4'b0101;
		R_Type_SRL:		alu_control_values = 4'b0110;
		
		I_Type_ADDI:	alu_control_values = 4'b0000;
		I_Type_XORI:	alu_control_values = 4'b0010;
		I_Type_ORI:		alu_control_values = 4'b0011;
		I_Type_ANDI:	alu_control_values = 4'b0100;
		I_Type_SLLI:	alu_control_values = 4'b0101;
		I_Type_SRLI:	alu_control_values = 4'b0110;
		I_Type_LD:		alu_control_values = 4'b0000;
		
		S_Type_STR: 	alu_control_values = 4'b0000;
		
		B_Type_BRCH:	alu_control_values = 4'b0111;
		
		J_Type_JAL:		alu_control_values = 4'b1000;
		
		U_Type_LUI:		alu_control_values = 4'b1111;
		

		default: alu_control_values = 4'b0000;
	endcase
end


assign ALU_Operation_o = alu_control_values;



endmodule
