	.syntax unified
	.cpu cortex-m4
	.thumb

.text
	.global max7219_init
	.global max7219_send
	.global ClearScreen
	.global Display
	.equ RCC_AHB2ENR  , 0x4002104C
	.equ X            , 1008000
	.equ GPIOC_MODER  , 0x48000800
	.equ GPIOC_OTYPER , 0x48000804
	.equ GPIOC_OSPEEDR, 0x48000808
	.equ GPIOC_IDR    , 0x48000810
	.equ GPIOC_PUPDR  , 0x4800080C

    .equ GPIOC_BSRR,    0x48000818
    .equ GPIOC_BRR,     0x48000828

    .equ DECODE_MODE,   0x9
    .equ INTENSITY,     0xA
    .equ SCAN_LIMIT,    0xB
    .equ SHUTDOWN,      0xC
    .equ DISPLAY_TEST,  0xF

    .equ DIN,           0x1     // PC0
    .equ CS,            0x2     // PC1
    .equ CLK,           0x4     // PC2

max7219_send:
	//input parameter: r0 is ADDRESS , r1 is DATA
	PUSH  {r4-r11, LR}
	LDR  r2, =DIN
	LDR  r3, =CS
	LDR  r4, =CLK
	LDR  r5, =GPIOC_BSRR
	LDR  r6, =GPIOC_BRR
	MOV  r7, #16 // Serial-Data counter
	STR  r3, [r6]  // set CS as 0

	LSL  r0, r0, 8
	ADD  r0, r0, r1

SendLoop:
	STR  r4, [r6]   // set CLK as 0
	MOV  r8, #1
	sub  r9, r7, #1
	LSL  r8, r9
	ANDS r8, r8, r0
	CMP  r8, #0
	ite  eq
	STReq  r2, [r6] // set DIN as 0
	STRne  r2, [r5] // set DIN as 1
	STR  r4, [r5]   // CLK rise
	SUBS r7, r7, #1
	CMP  r7, #0
	BGT  SendLoop

	STR  r3, [r5]   // CS rise
	POP  {r4-r11, PC}
max7219_init:
	PUSH  {r4-r11, LR}

	LDR  r0, =SHUTDOWN
	LDR  r1, =#0x1
	BL   max7219_send

	LDR  r0, =DECODE_MODE
	LDR  r1, =#0xFF
	BL   max7219_send

	LDR  r0, =INTENSITY
	LDR  r1, =#0xF
	BL   max7219_send

	LDR  r0, =SCAN_LIMIT
	LDR  r1, =#0x7
	BL   max7219_send

	LDR  r0, =DISPLAY_TEST
	LDR  r1, =#0x0
	BL   max7219_send
	BL  ClearScreen
	POP  {r4-r11, PC}

ClearScreen:
	PUSH  {r4-r11, LR}
	MOV  r4, #1
ClearLoop:

	MOV  r0, r4
	MOV  r1, #0xF
	BL  max7219_send
	ADD  r4, r4 ,#1
	CMP  R4, #8
	BLE ClearLoop

	POP  {r4-r11, PC}


Display:
	PUSH {r4,r5,r6,r7,r8,r9,LR}
	MOV  r6, r0
	MOV  r5, #1 //COUNT
displayloop:
	MOV  r4, #10
	UDIV r7, r6, r4
	MUL  r7, r7, r4
	SUB  r7, r6, r7
	UDIV r6, r6, r4
	MOV  r0, r5
	MOV  r1, r7

	CMP  r5, #3
	it   eq
	addeq r1, r1, 0x80
	BL  max7219_send
	ADD  r5, r5, #1
	CMP  r5, #3
	BLE  displayloop
	CMP  r6, #0
	BNE  displayloop
	POP {r4,r5,r6,r7,r8,r9,LR}
	BX LR
