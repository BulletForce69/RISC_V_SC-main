module HazardControl
(
	input [31:0]Instr,	//Instrucción 
	input [4:0]RegDWB,	//dato que se va a forwardear de Memoria
	input [4:0]RegDMEM,	//Registros a comparar
	input RegWR,
	
	output reg FwWB,
	output reg FwMEM,
	output reg UpdateA,
	output reg UpdateB,
	output reg Flush
	
);

always@(Instr or RegDEX or RegDMEM or RegWR) 
begin
	if(RegDMEM == Instr[19:15] || RegDWB == Instr[19:15]){
			UpdateA = 1'b1;
	}
	if(RegDMEM == Instr[24:20] || RegDWB == Instr[24:20]){
			UpdateB = 1'b1;
	}
	
	//desde MEM
	if((RegDMEM == Instr[19:15] || RegDMEM == Instr[24:20]) && RegWR == 1'b1 && RegMEM != 4'b0){
		FwMEM = 1'b1;
	}
	//desde WB
	if((RegDWB == Instr[19:15] || RegDWB == Instr[24:20]) && RegWR == 1'b1 && RegMEM != 4'b0){
		FwWB = 1'b1;
	}
	//Flush Flag si hay un salto
	if(Instr[6:0] == 7'b1100011 || Instr[6:0] == 7'b1100111 || Instr[6:0] == 7'b1101111 || Instr[6:0] == 7'b0000011){
		Flush = 1'b1;
	}
	
end

endmodule
