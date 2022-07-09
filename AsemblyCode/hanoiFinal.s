#Luis Fernando Casas Ram?rez
#Jose Miguel Castellanos Padilla

.data
Torre1: .word 0
.text
addi s0 zero 5 #N discos
add t2 zero s0
add a0 s0 zero
add a1 s0 zero
lui s1 %hi(Torre1)
addi s1 s1 %lo(Torre1)
#inicalizar memoria con 3 torres y n discos (espacios de 4 en 4)
for: blt t1 s0 loop
beq zero zero AuxDest
loop:
addi t1 t1 1
sw t2 0(s1)
addi t2 t2 -1
addi s1 s1 4
beq zero zero for

AuxDest: #guardamos el apuntador a las torres
add s2 zero s1
addi s1 s1 -4
add s3 zero s2
add t1 zero zero
for2: blt t1 s0 loop2
beq zero zero main
loop2:
addi t1 t1 1
addi s3 s3 4
beq zero zero for2

    
main:
    addi s1 s1 4
    jal ra, Hanoi
    jal zero End

Hanoi:
    addi sp sp -8
    sw ra 0(sp) #guardamos direccion de retorno
    sw a0 4(sp) #guardamos la cantidad de discos
    addi t1 zero 2
    bge a0 t1 recurs
    jal ra MoverDisco
    jal zero return
recurs:
    #add s5 zero a0 #guardar discos antes del -1
    addi a0 a0 -1
    add t5 zero s2  #usamos un temporal para hacer swap1
    add s2 zero s3
    add s3 zero t5
    jal ra Hanoi
    add t5 zero s2  #usamos un temporal para hacer swap1
    add s2 zero s3
    add s3 zero t5
    jal ra MoverDisco
    add t5 zero s2 #usamos un temporal para hacer swap2
    add s2 zero s1
    add s1 zero t5
    jal ra Hanoi
    add t5 zero s2 #usamos un temporal para hacer swap2
    add s2 zero s1
    add s1 zero t5
    jal zero return
MoverDisco:
    addi s1 s1 -4
    lw t6 0(s1)
    sw zero 0(s1)
    sw t6 0(s3)
    addi s3 s3 4
    jalr zero ra 0
            
return:
    lw a0 4(sp)
    lw ra 0(sp)
    addi sp sp 8
    jalr zero ra 0

End: beq zero zero 0