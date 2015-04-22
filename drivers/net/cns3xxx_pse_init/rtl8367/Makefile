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
# Makefile for the Star GSW ethernet driver
#

#obj-y :=
#obj-m :=

obj-$(CONFIG_RTL8367) += rtl8367.o
rtl8367-objs := rtl8370_asicdrv_led.o            rtl8370_asicdrv_rma.o \
rtl8370_asicdrv_acl.o rtl8370_asicdrv_lut.o            rtl8370_asicdrv_rrcp.o \
rtl8370_asicdrv.o            rtl8370_asicdrv_meter.o          rtl8370_asicdrv_scheduling.o \
rtl8370_asicdrv_cputag.o     rtl8370_asicdrv_mib.o            rtl8370_asicdrv_specialCongest.o \
rtl8370_asicdrv_dot1x.o      rtl8370_asicdrv_mirror.o         rtl8370_asicdrv_storm.o \
rtl8370_asicdrv_eee.o        rtl8370_asicdrv_misc.o           rtl8370_asicdrv_svlan.o \
rtl8370_asicdrv_eeelldp.o    rtl8370_asicdrv_oam.o            rtl8370_asicdrv_trunking.o \
rtl8370_asicdrv_fc.o         rtl8370_asicdrv_phy.o            rtl8370_asicdrv_unknownMulticast.o \
rtl8370_asicdrv_green.o      rtl8370_asicdrv_port.o           rtl8370_asicdrv_vlan.o \
rtl8370_asicdrv_igmp.o       rtl8370_asicdrv_portIsolation.o  smi.o \
rtl8370_asicdrv_inbwctrl.o   rtl8370_asicdrv_qos.o \
rtl8370_asicdrv_interrupt.o  rtl8370_asicdrv_rldp.o rtk_api.o

EXTRA_CFLAGS += -Idrivers/net/cns3xxx_pse_init/rtl8367/ -D_LITTLE_ENDIAN
