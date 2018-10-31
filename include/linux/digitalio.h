#ifndef __LINUX_DIGITALIO_H
#define __LINUX_DIGITALIO_H

#include <linux/ioctl.h>

struct digitalio_waitforinputchange {
        int timeout;
        unsigned mask;
        unsigned intcap;
};

#define DIGITALIO_MAGIC 0xC5
#define DIGITALIO_IOCBASE 0x00
#define DIGITALIO_WAITFORINPUTMASK   _IOWR(DIGITALIO_MAGIC, DIGITALIO_IOCBASE    , struct digitalio_waitforinputchange *)
#define DIGITALIO_GETINPUTMASK       _IOR (DIGITALIO_MAGIC, DIGITALIO_IOCBASE + 1, int *)
#define DIGITALIO_GETLASTINPUT       _IOWR(DIGITALIO_MAGIC, DIGITALIO_IOCBASE + 2, struct digitalio_waitforinputchange *)

#endif 
