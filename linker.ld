OUTPUT_FORMAT("elf32-littlearm")

ENTRY(_start)

MEMORY
{
   ram (rwx) : ORIGIN = 0x0, LENGTH = 8M
}

SECTIONS
{
   .text :
   {
       /* Reset vector section */
       KEEP(*(.text.vec))

       /* Start of .text section */
       *(.text._start)
       PROVIDE(_text_start = .);
       *(.text)

       /* End of .text section */
       *(.text*)
       PROVIDE(_text_end = .);
   } > ram

   .rodata :
   {
       /* Start of .rodata section */
       *(.rodata)

       /* End of .rodata section */
       *(.rodata*)
   } > ram

   /* Define symbols for start and end of .rodata section */
   PROVIDE(_rodata_start = ADDR(.rodata));
   PROVIDE(_rodata_end = ADDR(.rodata) + SIZEOF(.rodata));

   .data :
   {
       /* Start of .data section */
       *(.data)
   } > ram

   PROVIDE(_data_start = ADDR(.data));
   PROVIDE(_data_end = ADDR(.data) + SIZEOF(.data));

   .bss :
   {
       /* Start of .bss section */
       *(.bss)
   } > ram

   /* Stack section */
   stack_top = ORIGIN(ram) + LENGTH(ram);
   stack_bottom = stack_top - 4K; /* Adjust size as needed */
   .stack :
   {
       . = ALIGN(8);
       _stack_start = .;
       . += 1024;
       PROVIDE(_sys_stack_top = .);
       . += 1024;
       PROVIDE(_svc_stack_top = .);
       . += 256;
       PROVIDE(_und_stack_top = .);
       . += 256;
       PROVIDE(_abt_stack_top = .);
       . += 1024;
       PROVIDE(_irq_stack_top = .);
       . += 256;
       PROVIDE(_fiq_stack_top = .);
       . = ALIGN(8);
       _stack_end = .;
   } > ram

}
