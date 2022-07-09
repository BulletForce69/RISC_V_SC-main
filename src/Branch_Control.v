module Branch_Control
(
	input [2:0]Func_3,
	input [6:0]Opcode,
	input Branch_i,
	input [31:0]ALU_Result,
	
	output Branch_Flag_o
	
);

reg Branch_Flag;


always@(Func_3 or ALU_Result or Opcode or Branch_i) 
begin
	if(Branch_i == 1'b1)
		if(Opcode == 7'b1101111 || Opcode == 7'b1100111) //Si es un Jal salta por fuerza
			Branch_Flag = 1'b1;
		else
			case(Func_3)//                          

				3'h0:		Branch_Flag = ~(|ALU_Result);//BEQ
							
				3'h1:		Branch_Flag = (|ALU_Result);//BNQ
						
				3'h4:		Branch_Flag = ALU_Result[31];//BLT
				
				3'h5:		Branch_Flag = ~ALU_Result[31];//BGE
			
				
				default:
					Branch_Flag = 1'b0;
				endcase
			
	else
		Branch_Flag = 1'b0;
end

assign Branch_Flag_o = Branch_Flag;

endmodule
