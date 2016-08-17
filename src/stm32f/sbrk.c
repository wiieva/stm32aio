/* Support files for GNU libc.  Files in the system namespace go here.
   Files in the C namespace (ie those that do not start with an
   underscore) go in .c.  */

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>

#undef errno
extern int errno;

#define MAX_STACK_SIZE 0x200
#define VTOR 0xE000ED08

  register char * stack_ptr asm("sp");

//caddr_t _sbrk		_PARAMS ((int));

caddr_t _sbrk_r(void *reent,int incr)
{
	(void)reent;
	extern char end asm("end");
	static char *heap_end;
        char *prev_heap_end;

        if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;

	if (heap_end + incr > stack_ptr)
	{
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}
