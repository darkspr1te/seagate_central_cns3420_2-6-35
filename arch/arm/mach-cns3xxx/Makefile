#
# Makefile for the linux kernel.
#

obj-$(CONFIG_ARCH_CNS3XXX)		+= core.o pm.o clock.o devices.o lm.o
obj-$(CONFIG_MACH_CNS3420VB)		+= cns3420vb.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_AMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_PCIEPORTBUS)		+= pcie.o
obj-$(CONFIG_CNS3XXX_RAID)		+= rdma.o rdma_sysfs.o raid5_dataplane.o
obj-$(CONFIG_CPU_FREQ_CNS3XXX)		+= cpufreq.o
obj-$(CONFIG_CNS3XXX_DMAC)              += dmac.o
obj-$(CONFIG_CNS3XXX_DMAC_M2MTEST)      += dmac_m2mtest.o
obj-$(CONFIG_CNS3XXX_PCM_LEGERITY_2PHONE_DEMO) += demo_pcm_legerity_config_2phone.o
obj-$(CONFIG_CNS3XXX_EXT_INTR_DEMO)	+= ext_intr.o

