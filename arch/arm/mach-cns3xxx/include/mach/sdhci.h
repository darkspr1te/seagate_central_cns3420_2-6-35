/*******************************************************************************
 *
 *  arch/arm/mach-cns3xxx/include/mach/sdhci.h
 *
 *  Scott Shu
 *
 *  Copyright (c) 2011 Cavium
 *  Copyright (c) 2009 Cavium Networks
 *
 *  SDHCI platform data definitions
 *
 *  This file is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, Version 2, as
 *  published by the Free Software Foundation.
 *
 *  This file is distributed in the hope that it will be useful,
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 *  NONINFRINGEMENT.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this file; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA or
 *  visit http://www.gnu.org/licenses/.
 *
 *  This file may also be available under a different license from Cavium.
 *  Contact Cavium for more information
 *
 ******************************************************************************/

#ifndef	_CNS3XXX_SDHCI_H_
#define	_CNS3XXX_SDHCI_H_

struct platform_device;

struct cns3xxx_sdhci_platdata {
	unsigned int	max_width;
	unsigned int	host_caps;
	char		**clocks;

	struct sdhci_host *sdhci_host;
};
#endif
