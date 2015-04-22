# Edit this file as appropriate for your MSD.
# The new file should be committed to the MSD branch.

PV = "2.6.35"
PR = "r1"

COMPATIBLE_MACHINE = "(whitney-econa-1bay|whitney-econa-2bay|whitney-econa-4bay|cirrus-v1)"
DESCRIPTION = "Linux kernel for the Econa-base Whitney NAS platform"

FILESDIR = "${FILE_DIRNAME}"

DEF_KERNEL_URI = "${KERNELORG_MIRROR}/pub/linux/kernel/v2.6/linux-${PV}.tar.bz2"
DEF_KERNEL_CONFIG = "configs/${MACHINE}_defconfig"
DEF_KERNEL_PATCH_SERIES = "${FILESDIR}/patches/series"

# the following variables are used when
# KERNEL_SOURCE_DIRECTORY or KERNEL_SOURCE_URI is set
DEF_EXTERNAL_CONFIG = ".config"
DEF_EXTERNAL_PATCH_SERIES = ""

inherit mvl_kernel
