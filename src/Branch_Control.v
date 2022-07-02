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
		if(Opcode == 7'b1101111 || Opcode == 7'b1100111)
			Branch_Flag = 1'b1;
		else
			case(Func_3)//                          

				3'h0:	if(ALU_Result == 31'h0)
							Branch_Flag = 1'b1;
						else
							Branch_Flag = 1'b0;
				3'h1:	if(ALU_Result != 31'h0)
							Branch_Flag = 1'b1;
						else
							Branch_Flag = 1'b0;
				3'h4:	if(ALU_Result < 31'h0)
							Branch_Flag = 1'b1;
						else
							Branch_Flag = 1'b0;
				3'h5:	if(ALU_Result >= 31'h0)
							Branch_Flag = 1'b1;
						else
							Branch_Flag = 1'b0;
			
				
				default:
					Branch_Flag = 1'b0;
				endcase
			
	else
		Branch_Flag = 1'b0;
end

assign Branch_Flag_o = Branch_Flag;

endmodule
