/*
 * mov_avg.s
 *
 * Created on: 2/2/2026
 * Author: Hitesh B, Hou Linxin
 */
.syntax unified
 .cpu cortex-m4
 .thumb
 .global mov_avg
 .equ N_MAX, 8
 .bss
 .align 4

 .text
 .align 2
@ CG2028 Assignment, Sem 2, AY 2025/26
@ (c) ECE NUS, 2025
@ Write Student 1’s Name here: ABCD (A1234567R)
@ Write Student 2’s Name here: WXYZ (A0000007X)
@ You could create a look-up table of registers here:
@ R0 ...
@ R1 ...
@ write your program from here:
mov_avg:
 PUSH {r2-r11, lr}

@R0,R1 already loaded with N and accel_buff respectively

 MOV R2, R0        @ R2 = N (loop counter)
 MOV R4, #0        @ sum = 0

LOOP:
 LDR R3, [R1], #4  @ load [R1] into R3, post index
 ADD R4, R4, R3    @ sum += value
 SUBS R2, R2, #1   @ decrement loop counter
 BNE LOOP          @ repeat until N values added

 SDIV R0, R4, R0   @ result = sum / N

 POP {r2-r11, pc}
