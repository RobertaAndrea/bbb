@******************************************************************************
@
@ rtc_init.S - Init code routines
@
@******************************************************************************
@
@ Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
@
@
@  Redistribution and use in source and binary forms, with or without
@  modification, are permitted provided that the following conditions
@  are met:
@
@    Redistributions of source code must retain the above copyright
@    notice, this list of conditions and the following disclaimer.
@
@    Redistributions in binary form must reproduce the above copyright
@    notice, this list of conditions and the following disclaimer in the
@    documentation and/or other materials provided with the
@    distribution.
@
@    Neither the name of Texas Instruments Incorporated nor the names of
@    its contributors may be used to endorse or promote products derived
@    from this software without specific prior written permission.
@
@  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
@  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
@  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
@  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
@  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
@  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
@  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
@  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
@  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
@  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
@
@******************************************************************************

@****************************** Global Symbols*********************************
        .global Entry
        .global _stack
        .global __bss_start__
        .global __bss_end__
        .global start_boot
        .global IRQHandler
        .global FIQHandler
        .global AbortHandler
        .global SVC_Handler
        .global UndefInstHandler
        .global INTCCommonIntrHandler
	   .global _init

@************************ Internal Definitions ********************************
@
@ Define the stack sizes for different modes. The user/system mode will use
@ the rest of the total stack size
@
        .set  UND_STACK_SIZE, 0x8
        .set  ABT_STACK_SIZE, 0x8
        .set  FIQ_STACK_SIZE, 0x8
        .set  IRQ_STACK_SIZE, 0x1000
        .set  SVC_STACK_SIZE, 0x8

@
@ to set the mode bits in CPSR for different modes
@
        .set  MODE_USR, 0x10
        .set  MODE_FIQ, 0x11
        .set  MODE_IRQ, 0x12
        .set  MODE_SVC, 0x13
        .set  MODE_ABT, 0x17
        .set  MODE_UND, 0x1B
        .set  MODE_SYS, 0x1F

        .equ  I_F_BIT, 0xC0
        .equ MASK_SVC_NUM, 0x1

@**************************** Code Seection ***********************************
        .text

@
@ This code is assembled for ARM instructions
@
        .code 32

@******************************************************************************
@
@******************************************************************************
@
@ The reset handler in StarterWare is named as 'Entry'.
@ The reset handler sets up the stack pointers for all the modes. The FIQ and
@ IRQ shall be disabled during this. Then clear the BSS sections and finally
@ switch to the function calling the main() function.
@
Entry:


        @STMFD sp!, {lr}                       @ Store link reg
        MOV r7, sp
        MOV r8, lr
        LDR r9, =0xfeedfeed
        CMP r0, r9
        BEQ Diag_jump
        MOV r9, #0

Diag_jump:

@
@ Set up the Stack for Undefined mode
@
        LDR   r0, =_stack                  @ Read the stack address
        MSR   cpsr_c, #MODE_UND|I_F_BIT    @ switch to undef  mode
        MOV   sp,r0                        @ write the stack pointer
        SUB   r0, r0, #UND_STACK_SIZE      @ give stack space
@
@ Set up the Stack for abort mode
@
        MSR   cpsr_c, #MODE_ABT|I_F_BIT    @ Change to abort mode
        MOV   sp, r0                       @ write the stack pointer
        SUB   r0,r0, #ABT_STACK_SIZE       @ give stack space
@
@ Set up the Stack for FIQ mode
@
        MSR   cpsr_c, #MODE_FIQ|I_F_BIT    @ change to FIQ mode
        MOV   sp,r0                        @ write the stack pointer
        SUB   r0,r0, #FIQ_STACK_SIZE       @ give stack space
@
@ Set up the Stack for IRQ mode
@
        MSR   cpsr_c, #MODE_IRQ|I_F_BIT    @ change to IRQ mode
        MOV   sp,r0                        @ write the stack pointer
        SUB   r0,r0, #IRQ_STACK_SIZE       @ give stack space
@
@ Set up the Stack for SVC mode
@
        MSR   cpsr_c, #MODE_SVC|I_F_BIT    @ change to SVC mode
        MOV   sp,r0                        @ write the stack pointer
        SUB   r0,r0, #SVC_STACK_SIZE       @ give stack space
@
@ Set up the Stack for USer/System mode
@
        MSR   cpsr_c, #MODE_SYS|I_F_BIT    @ change to system mode
        MOV   sp,r0                        @ write the stack pointer

@ Invalidate and Enable Branch Prediction
        MOV     r0, #0
        MCR     p15, #0, r0, c7, c5, #6
        ISB
        MRC     p15, #0, r0, c1, c0, #0
        ORR     r0, r0, #0x00000800
        MCR     p15, #0, r0, c1, c0, #0

@
@ Enable Neon/VFP Co-Processor
@
        MRC p15, #0, r1, c1, c0, #2        @ r1 = Access Control Register
        ORR r1, r1, #(0xf << 20)           @ enable full access for p10,11
        MCR p15, #0, r1, c1, c0, #2        @ Access Control Register = r1
        MOV r1, #0
        MCR p15, #0, r1, c7, c5, #4        @ flush prefetch buffer
        MOV r0,#0x40000000
        FMXR FPEXC, r0                     @ Set Neon/VFP Enable bit

Diag_save_lr:
        CMP r9, #0
        BEQ Clear_Bss_Section
        STMFD sp!, {r7, r8}

@
@ Clear the BSS section here
@
Clear_Bss_Section:

        LDR   r0, =__bss_start__             @ Start address of BSS
        LDR   r1, =(__bss_end__ - 0x04)       @ End address of BSS
        MOV   r2, #0
Loop:
        STR   r2, [r0], #4                 @ Clear one word in BSS
        CMP   r0, r1
        BLE   Loop                         @ Clear till BSS end
        B Diag_return_main

@******************************************************************************
@*                  Function Definition of SWI Handler
@******************************************************************************
@
@ The SVC Handler switches to system mode if the SVC number is 1. If the
@ SVC number is different, no mode switching will be done. No other SVC are
@ handled here
@
SVC_Handler:
        STMFD     r13!, {r0-r1, r14}       @ Save context in SVC stack
        SUB       r13, r13, #0x4           @ Adjust the stack pointer
        LDR       r0, [r14, #-4]           @ R0 points to SWI instruction
        BIC       r0, r0, #MASK_SVC_NUM    @ Get the SWI number
        CMP       r0, #458752
        MRSEQ     r1, spsr                 @ Copy SPSR
        ORREQ     r1, r1, #0x1F            @ Change the mode to System
        MSREQ     spsr_cf, r1              @ Restore SPSR
        ADD       r13, r13, #0x4           @ Adjust the stack pointer
        LDMFD     r13!, {r0-r1, pc}^       @ Restore registers from IRQ stack

@******************************************************************************
@*                  Function Definition of IRQ Handler
@******************************************************************************
@
@ The IRQ handler saves the context and jumps to the common handler which calls
@ the handlers of highest priority pending IRQ.
@
IRQHandler:
        SUB       r14, r14, #4             @ Apply lr correction
        STMFD     r13!, {r0-r3, r12, r14}  @ Save context
        MRS       r12, spsr                @ Copy spsr
        VMRS      r1, FPSCR                @ Copy fpscr
        VMRS      r2, FPEXC                @ Copy fpexc
        STMFD     r13!, {r1-r2, r12}       @ Save spsr, fpscr and fpexc
        VSTMDB    r13!, {d0-d7}            @ Save D0-D7 Neon/VFP registers
        VSTMDB    r13!, {d16-d31}          @ Save D16-D31 Neon/VFP registers

        bl        INTCCommonIntrHandler

        VLDMIA    r13!, {d16-d31}          @ Restore D16-D31 Neon/VFP registers
        VLDMIA    r13!, {d0-d7}            @ Restore D0-D7 Neon/VFP registers
        LDMFD     r13!, {r1-r2, r12}       @ Get fpscr, fpexc and spsr
        MSR       spsr, r12                @ Restore spsr
        VMSR      fpscr, r1                @ Restore fpscr
        VMSR      fpexc, r2                @ Restore fpexc

        LDMFD     r13!, {r0-r3, r12, pc}^  @ Restore the context and return

@******************************************************************************
@*                  Function Definition of FIQ Handler
@******************************************************************************
@
@ FIQ is not supported in this version of software.
@
FIQHandler:
        SUBS      pc, r14, #0x4            @ Return to program before FIQ

@******************************************************************************
@*             Function Definition of Abort/Undef Handler
@******************************************************************************
@
@ The Abort handler goes to the C handler of abort mode. Note that the undefined
@ instruction is not handled separately.
@ if nothing is done in the abort mode, the execution enters infinite loop.
@
AbortHandler:
UndefInstHandler:
@
@ Go to infinite loop.
@
loop0:
        B         loop0

Diag_return_main:
    LDR   r10, =start_boot                      @ Get the address of main
    MOV   lr,pc                           @ Dummy return to main
    BX    r10                             @ Branch to main

Diag_return:
    MOV r9, r0
    LDMFD sp, {sp, pc}

@
@ End of the file
@
         .end
