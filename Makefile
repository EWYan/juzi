TARGET = ASR_RTOS
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

linker_flags += -C link-arg=--Map=target/armv7r-none-eabi/debug/$(TARGET).map

# qemu paramenters
QEMU := /mnt/c/02_Docs/STUFF_TO/Lab/qemu/build/aarch64-softmmu/qemu-system-aarch64
DTB  := test-arm.dtb

QEMU_DTB_PATH := test-arm.dtb
QEMU_FLAGS := -nographic -machine arm-generic-fdt
QEMU_FLAGS += -dtb $(QEMU_DTB_PATH)
QEMU_FLAGS += -net none -pidfile qemu.pid -chardev stdio,id=con,mux=on
QEMU_FLAGS += -serial chardev:con -mon chardev=con,mode=readline
QEMU_FLAGS += -icount shift=3,align=off,sleep=off -rtc clock=vm
QEMU_FLAGS += -device loader,file=target/armv7r-none-eabi/debug/${TARGET},cpu-num=4
QEMU_FLAGS += -device loader,addr=0xff5e023c,data=0x80008fde,data-len=4
QEMU_FLAGS += -device loader,addr=0xff9a0000,data=0x80000218,data-len=4


all:
	@cargo rustc -- ${linker_flags} --verbose
	@$(OBJDUMP) -D target/armv7r-none-eabi/debug/$(TARGET) > target/armv7r-none-eabi/debug/$(TARGET).asm

qemu: all
	@$(QEMU) $(QEMU_FLAGS)

clean:
	@rm -rf target/

.PHONY: all clean qemu