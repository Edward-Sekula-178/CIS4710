        .section .data
input_string:   .asciz "Hello world!"

        # tohost/fromhost are data locations used by Spike
        .align 8
tohost: .byte 0
        .align 8
fromhost: .byte 0

        
        .section .text
        .globl _start
_start:
        # Load the address of the input string into a0
        la a0, input_string

        # Your code here!
        # t3 is lower bound, t4 is upper bound of lower/upper case numbers
        addi t3, x0, 0x061
        addi t4, x0, 0x07B
        # we maintin a pointer in t5
        add t5, a0, x0
        # t1 is the character
        lb t1, 0(t5)
.LOOP:
        beq t1, x0, end_program
        bge t1, t3, .CONDITION
        j .AFTER_CONDITION
.CONDITION:
        blt t1, t4, .UPPER
        j .AFTER_CONDITION
.UPPER:
        addi t1, t1, -32
        sb t1, 0(t5)
        j .AFTER_CONDITION
.AFTER_CONDITION:
        addi t5,t5,0x01
        lb t1, 0(t5)
        j .LOOP
end_program:
        # jump here when you're done
        j end_program

