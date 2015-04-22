#ifndef LINUX_VERSION_H
#define LINUX_VERSION_H

#include <linux/version.h>


#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,31)
#define LINUX31
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
#define LINUX35
#endif

#endif

