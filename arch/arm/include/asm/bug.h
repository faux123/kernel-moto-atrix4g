#ifndef _ASMARM_BUG_H
#define _ASMARM_BUG_H


#ifdef CONFIG_BUG
#ifdef CONFIG_DEBUG_BUGVERBOSE
extern void __bug(const char *file, int line);

/* give file/line information */
#define BUG()	do { __bug(__FILE__, __LINE__);  *(int *)1 = 0; } while (1)

#else

/* this just causes an oops */
#define BUG()		do { *(int *)0 = 0; } while (1)

#endif

#define HAVE_ARCH_BUG
#endif

#include <asm-generic/bug.h>

#endif
