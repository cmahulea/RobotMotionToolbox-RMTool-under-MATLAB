/*
 * getrusage
 * Implementation according to:
 * The Open Group Base Specifications Issue 6
 * IEEE Std 1003.1, 2004 Edition
 */

/*
 *  THIS SOFTWARE IS NOT COPYRIGHTED
 *
 *  This source code is offered for use in the public domain. You may
 *  use, modify or distribute it freely.
 *
 *  This code is distributed in the hope that it will be useful but
 *  WITHOUT ANY WARRANTY. ALL WARRANTIES, EXPRESS OR IMPLIED ARE HEREBY
 *  DISCLAIMED. This includes but is not limited to warranties of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  Contributed by:
 *  Ramiro Polla <ramiro@lisha.ufsc.br>
 */

#include "resource.h"
#include <stdint.h>
#include <errno.h>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

static inline void FILETIME_to_timeval(struct timeval *tv, FILETIME *ft)
{
    int64_t fulltime = ((int64_t)ft->dwHighDateTime<<32|ft->dwLowDateTime);
    fulltime /= 10; /* 100-ns -> us */

    tv->tv_sec  = fulltime / 1000000;
    tv->tv_usec = fulltime % 1000000;
}

int __cdecl getrusage(int who, struct rusage *r_usage)
{
    HANDLE proc;
    FILETIME dummy;
    FILETIME KernelTime;
    FILETIME UserTime;

    if(!r_usage) {
        errno = EFAULT;
        return -1;
    }

    if(who != RUSAGE_SELF) {
        errno = EINVAL;
        return -1;
    }

    proc = GetCurrentProcess();
    GetProcessTimes(proc, &dummy, &dummy, &KernelTime, &UserTime);

    FILETIME_to_timeval(&r_usage->ru_stime, &KernelTime);
    FILETIME_to_timeval(&r_usage->ru_utime, &UserTime  );

    return 0;
}
