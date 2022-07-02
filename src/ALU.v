/******************************************************************
* Description
*	This is an 32-bit arithetic logic unit that can execute the next set of operations:
*		add

* This ALU is written by using behavioral description.
* Version:
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	16/08/2021
******************************************************************/

module ALU 
(
	input [3:0] ALU_Operation_i,
	input signed [31:0] A_i,
	input [31:0] Pc4,
	input signed [31:0] B_i,
	output reg Zero_o,
	output reg [31:0] ALU_Result_o
);

localparam ADD 	= 4'b0000;
localparam SUB 	= 4'b0001;
localparam XOR  	= 4'b0010;
localparam OR  	= 4'b0011;
localparam AND 	= 4'b0100;
localparam SLL 	= 4'b0101;
localparam SRL 	= 4'b0110;
localparam BRCH	= 4'b0111;
localparam JAL		= 4'b1000;
localparam STR 	= 4'b1001;
localparam LUI 	= 4'b1111;

   
   always @ (A_i or B_i or ALU_Operation_i or Pc4)
     begin
		case (ALU_Operation_i)
		ADD: 	ALU_Result_o = A_i + B_i;	
		SUB: 	ALU_Result_o = A_i - B_i;
		AND: 	ALU_Result_o = A_i & B_i;
		OR: 	ALU_Result_o = A_i | B_i;
		XOR: 	ALU_Result_o = A_i ^ B_i;
		SLL: 	ALU_Result_o = A_i << B_i;
		SRL: 	ALU_Result_o = A_i >> B_i;
		BRCH: ALU_Result_o = A_i - B_i;
		JAL:	ALU_Result_o = Pc4;
		LUI: 	ALU_Result_o = B_i;

	
		default:
			ALU_Result_o = 0;
		endcase // case(control)
		
		Zero_o = (ALU_Result_o == 0) ? 1'b1 : 1'b0;
		
     end // always @ (A or B or control)
endmodule // ALU