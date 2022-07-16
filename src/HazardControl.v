module HazardControl
(
	input [31:0]Instr,	//Instrucción 
	input [4:0]RegDWB,	//dato que se va a forwardear de Memoria
	input [4:0]RegDMEM,	//Registros a comparar
	input RegWR,
	input BranchF,
	
	output reg FwWB,
	output reg FwMEM,
	output reg UpdateA,
	output reg UpdateB,
	output Flush
	
	
);

reg FF;



always@(Instr or RegDMEM or RegWR or RegDWB or BranchF) 
begin
	if(RegDMEM == Instr[19:15] || RegDWB == Instr[19:15])
			UpdateA = 1'b1;
	else
		UpdateA = 1'b0;
	if(RegDMEM == Instr[24:20] || RegDWB == Instr[24:20])
			UpdateB = 1'b1;
	else
		UpdateB = 1'b0;
	
	//desde MEM
	if((RegDMEM == Instr[19:15] || RegDMEM == Instr[24:20]) && RegWR == 1'b1 && RegDMEM != 4'b0)
		FwMEM = 1'b1;
	else
		FwMEM = 1'b0;
	
	//desde WB
	if((RegDWB == Instr[19:15] || RegDWB == Instr[24:20]) && RegWR == 1'b1 && RegDWB != 4'b0)
		FwWB = 1'b1;
	else
		FwWB = 1'b0;
	
	//Flush Flag si hay un salto
	if(BranchF == 1'b1)
		FF = 1'b1;
	else
		FF = 1'b0;
	
	
end

assign Flush = FF;

endmodule
