.extern vApplicationIRQHandler
.extern _stack_end
.extern _sys_stack_top
.extern _svc_stack_top
.extern _und_stack_top
.extern _abt_stack_top
.extern _irq_stack_top
.extern _fiq_stack_top

.extern ulICCIAR
.extern ulICCEOIR


.global _start
.type _start, %function
_start:
    /* Initialize general-purpose registers */
    mov r0, #0      /* Initialize r0 to zero */
    mov r1, #1      /* Initialize r1 to one */
    mov r2, #2      /* Initialize r2 to two */
    /* ... (initialize other registers as needed) */

    mrs r0, cpsr
    msr spsr_cxsf, r0
    
    /* system mode */
    ldr sp, =_sys_stack_top   /* User mode stack pointer share with sys mode */
    cps #0x13                 /* Switch to Supervisor mode */
    ldr sp, =_svc_stack_top   /* Supervisor mode stack pointer */
    cps #0x11                 /* Switch to System mode */
    ldr sp, =_fiq_stack_top   /* FIQ mode stack pointer */
    cps #0x12                 /* Switch to IRQ mode */
    ldr sp, =_irq_stack_top   /* IRQ mode stack pointer */
    cps #0x17                 /* Switch to Abort mode */
    ldr sp, =_abt_stack_top   /* Abort mode stack pointer */
    cps #0x1B                 /* Switch to Undefined mode */
    ldr sp, =_und_stack_top   /* Undefined mode stack pointer */

    cps #0x1F                 /* Switch back to System mode */

    b _start_rs
    b .

.global undefined_exception
.type undefined_exception, %function
undefined_exception:
    b .

.global osSysCallHandler
.type osSysCallHandler, %function
osSysCallHandler:
    b .

.global prefetch_exception
.type prefetch_exception, %function
prefetch_exception:
    b .

.global data_abort_exception
.type data_abort_exception, %function
data_abort_exception:
    b .


.global osFiqHandler
.type osFiqHandler, %function
osFiqHandler:
    b .

.global osIrqHandler
.type osIrqHandler, %function
osIrqHandler:
    sub     lr, lr, #4
    push    {{lr}}
    mrs     lr, spsr
    push    {{lr}}

    cps     #0x13

    push    {{r0-r4, r12}}

    ldr     r2, =ulICCIAR
    ldr     r2, [r2]
    ldr     r0, [r2]

    mov     r2, sp
    and     r2, r2, #4
    sub     sp, sp, r2

    push    {{r0-r4, lr}}
    ldr     r1, =vApplicationIRQHandler
    blx     r1
    pop     {{r0-r4, lr}}
    add     sp, sp, r2

    cpsid   i

    ldr     r4, =ulICCEOIR
    ldr     r4, [r4]
    str     r0, [r4]

exit_without_switch:
    pop     {{r0-r4, r12}}
    cps     #0x12
    pop     {{lr}}
    msr     spsr_cxsf, lr
    pop     {{lr}}
    movs    pc, lr
