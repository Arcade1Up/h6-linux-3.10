obj-$(CONFIG_SUNXI_NAND) += nand.o

libnand = libnand.a

$(obj)/$(libnand) : FORCE
ifneq ($(wildcard $(obj)/lib),)
	$(MAKE) -C $(obj) common0
endif
	cp $(obj)/$(basename $(libnand)) $@

nand-objs += nand_interface.o \
	nfd/nand_ota_burn.o \
	nfd/nand_base.o \
	nfd/nand_blk.o \
	nfd/nand_dev.o \
	nfd/nand_class.o \
	nfd/nand_test.o \
	nfd/nand_osal_for_linux.o \
	nfd/nand_lib_cfg.o \
	$(libnand)

.PHONY: common0

common0:
	@make -C ../common0/lib clean 2>/dev/null
	@make -C ../common0/lib install
