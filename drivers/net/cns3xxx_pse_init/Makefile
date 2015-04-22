################################################################################
#
# 
#   Copyright (c) 2008 Cavium Networks 
# 
# This program is free software; you can redistribute it and/or modify it 
# under the terms of the GNU General Public License as published by the Free 
# Software Foundation; either version 2 of the License, or (at your option) 
# any later version.
# 
# This program is distributed in the hope that it will be useful, but WITHOUT 
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for 
# more details.
# 
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 59 
# Temple Place - Suite 330, Boston, MA  02111-1307, USA.
# 
# The full GNU General Public License is included in this distribution in the
# file called LICENSE.
# 
# Contact Information:
# Star semiconduction Linux Support <support@starsemi.com>
#
################################################################################

#
# Makefile for the CNS3XXX PSE Initialize part
#

#obj-y :=
#obj-m :=

obj-$(CONFIG_CNS3XXX_PSE_INIT) += cns3xxx_pse_init.o
#if CONFIG_RTL8367_ONE_LEG || CONFIG_RTL8367 is y
ifeq (y, $(filter $(CONFIG_RTL8367_ONE_LEG) $(CONFIG_RTL8367), y))

EXTRA_CFLAGS += -Idrivers/net/cns3xxx_pse_init/rtl8367/ -D_LITTLE_ENDIAN
cns3xxx_pse_init-objs := cns3xxx_phy.o pse_init.o \
rtl8367/rtl8370_asicdrv_led.o        rtl8367/rtl8370_asicdrv_rma.o \
rtl8367/rtl8370_asicdrv_acl.o 			 rtl8367/rtl8370_asicdrv_lut.o            rtl8367/rtl8370_asicdrv_rrcp.o \
rtl8367/rtl8370_asicdrv.o            rtl8367/rtl8370_asicdrv_meter.o          rtl8367/rtl8370_asicdrv_scheduling.o \
rtl8367/rtl8370_asicdrv_cputag.o     rtl8367/rtl8370_asicdrv_mib.o            rtl8367/rtl8370_asicdrv_specialCongest.o \
rtl8367/rtl8370_asicdrv_dot1x.o      rtl8367/rtl8370_asicdrv_mirror.o         rtl8367/rtl8370_asicdrv_storm.o \
rtl8367/rtl8370_asicdrv_eee.o        rtl8367/rtl8370_asicdrv_misc.o           rtl8367/rtl8370_asicdrv_svlan.o \
rtl8367/rtl8370_asicdrv_eeelldp.o    rtl8367/rtl8370_asicdrv_oam.o            rtl8367/rtl8370_asicdrv_trunking.o \
rtl8367/rtl8370_asicdrv_fc.o         rtl8367/rtl8370_asicdrv_phy.o            rtl8367/rtl8370_asicdrv_unknownMulticast.o \
rtl8367/rtl8370_asicdrv_green.o      rtl8367/rtl8370_asicdrv_port.o           rtl8367/rtl8370_asicdrv_vlan.o \
rtl8367/rtl8370_asicdrv_igmp.o       rtl8367/rtl8370_asicdrv_portIsolation.o  rtl8367/smi.o \
rtl8367/rtl8370_asicdrv_inbwctrl.o   rtl8367/rtl8370_asicdrv_qos.o \
rtl8367/rtl8370_asicdrv_interrupt.o  rtl8367/rtl8370_asicdrv_rldp.o 					rtl8367/rtk_api.o
else
cns3xxx_pse_init-objs := cns3xxx_phy.o pse_init.o
endif

ifeq (y, $(filter $(CONFIG_CNS3XXX_PSE_SW_PHY_AUTO_POLLING), y))
cns3xxx_pse_init-objs += cns3xxx_sw_auto_polling.o
endif

#cns3xxx_pse_init.o
#obj-$(CONFIG_RTL8367) += rtl8367/
#EXTRA_CFLAGS += -Idrivers/net/cns3xxx/rtl8367/
#vega_main.o

#include $(TOPDIR)/Rules.make
