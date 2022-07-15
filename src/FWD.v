module Forwarding
(
	input [31:0]ReadDataWB,	//dato que se va a forwardear de Memoria
	input [31:0]ALUResWB,	//dato que se va a forwardear de Memoria
	input [31:0]ALUResMEM,	//dato que se va a forwardear de Memoria
	input MEMtoREG, //Para saber en caso de que el FW sea desde MEM cual Dato sale
	
	input ForwardWB, 		
	input ForwardMEM,
	/** entradas de un comparador que detecta hazard de datos, 
		es decir si los registros usados en la instrucción no tienen el dato más reciente**/
	input UpdateA,
	input UpdateB,
// entradas para saber cual registro es el que no está actualizado
	output reg [31:0]ForwardData,
	output ForwardA,
	output ForwardB
);
Assign ForwardA = UpdateA;
Assign ForwardB = UpdateB;

always@(ForwardMEM or  ReadDataWB or ForwardWB) 
begin
	if(ForwardMEM == 1'b1 && MEMtoREG == 1'b1){
		ForwardData = ReadDataMEM;
	}
	if(ForwardMEM == 1'b1 && MEMtoREG == 1'b0){
		ForwardData = ALUResMEM;
	}
	if(ForwardWB == 1'b1){
		ForwardData = ALUResWB;
	}
end

endmodule
