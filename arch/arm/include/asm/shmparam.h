#ifndef _ASMARM_SHMPARAM_H
#define _ASMARM_SHMPARAM_H

/*
 * This should be the size of the virtually indexed cache/ways,
 * or page size, whichever is greater since the cache aliases
 * every size/ways bytes.
 */
#ifdef CONFIG_PAGE_SIZE_64K
#define	SHMLBA	(1 * PAGE_SIZE)
#else
#define	SHMLBA	(4 * PAGE_SIZE)		 /* attach addr a multiple of this */
#endif

/*
 * Enforce SHMLBA in shmat
 */
#define __ARCH_FORCE_SHMLBA

#endif /* _ASMARM_SHMPARAM_H */
