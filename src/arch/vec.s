.extern _start

.extern undefined_exception
.extern osSysCallHandler
.extern prefetch_exception
.extern data_abort_exception
.extern osIrqHandler
.extern osFiqHandler

.global _vector_tbl

_vector_tbl:
    ldr pc, reset_addr
    ldr pc, undefined_addr
    ldr pc, syscall_addr
    ldr pc, prefetch_addr
    ldr pc, data_abort_addr
    b .
    ldr pc, irq_addr
    ldr pc, fiq_addr

reset_addr:     .word _start
undefined_addr: .word undefined_exception
syscall_addr:   .word osSysCallHandler
prefetch_addr:  .word prefetch_exception
data_abort_addr:.word data_abort_exception

irq_addr:       .word osIrqHandler
fiq_addr:       .word osFiqHandler
padding:        .word 0