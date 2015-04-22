#
# Makefile for the linux kernel.
#

obj-$(CONFIG_PROC_STATIC_FILES) += proc_static_files.o

$(obj)/proc_static_files.o: $(obj)/proc_static_files_table.h

targets += proc_static_files_table.h
$(obj)/proc_static_files_table.h: FORCE
	mkdir -p $(obj)/files
	$(srctree)/scripts/proc_static_files_table.py $(obj)/files >$(obj)/proc_static_files_table.h
