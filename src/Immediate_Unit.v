/******************************************************************
* Description
*	This module performs a sign extension operation that is need with
*	in instruction like andi and constructs the immediate constant.
* Version:
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	16/08/2021
******************************************************************/
module Immediate_Unit
(   
	input [6:0] op_i,
	input [31:0]  Instruction_bus_i,
	
   output reg [31:0] Immediate_o
);

localparam I_Type = 7'b0010011;
localparam U_Type = 7'b0110111;
localparam B_Type = 7'b1100011;
localparam S_Type = 7'b0100011;
localparam J_Type = 7'b1101111;
localparam JR_Type= 7'b1100111;


always@(op_i or Instruction_bus_i) begin
	case(op_i)

		I_Type:	Immediate_o = {{20{Instruction_bus_i[31]}},Instruction_bus_i[31:20]};// I format
		U_Type:	Immediate_o = {Instruction_bus_i[31:12],{12{1'b0}}};						// U format
		B_Type: 	Immediate_o = {{19{Instruction_bus_i[31]}},Instruction_bus_i[31],Instruction_bus_i[7],Instruction_bus_i[30:25],Instruction_bus_i[11:8],1'b0};// B format
		S_Type:	Immediate_o = {{20{Instruction_bus_i[31]}},Instruction_bus_i[31:25],Instruction_bus_i[11:7]};// S format
		J_Type:	Immediate_o = {{11{Instruction_bus_i[20]}},Instruction_bus_i[20],Instruction_bus_i[19:12],Instruction_bus_i[11],Instruction_bus_i[30:21],1'b0};	//J format
		JR_Type:	Immediate_o = {{20{Instruction_bus_i[31]}},Instruction_bus_i[31:20]}; //I format for JALR
		default:
			Immediate_o = 0;
	endcase
end


endmodule // signExtend
