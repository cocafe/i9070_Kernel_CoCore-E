/*
* ST-Ericsson ETF driver
*
* Copyright (c) ST-Ericsson SA 2011
*
* License terms: Redistribution and use in source and binary forms, with or
* without modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*   - Neither the name of ST-Ericsson S.A. nor the names of its contributors
*     may be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
/****************************************************************************
* driver.h
*
* This module serves as the lower interface header file for the adapter module
* and all driver's module.
*
****************************************************************************/

#ifndef _DRIVER_H
#define _DRIVER_H

#include "stddefs.h"

struct ST90tdsDev_s {
	int dummy;
};

typedef struct ST90tdsDev_s Dev_t;

/* Endian conversion. Use the linux ones*/
#ifndef le16_to_cpu
#define le16_to_cpu(_x) ((uint16)(_x))
#define le32_to_cpu(_x) ((uint32)(_x))
#define le64_to_cpu(_x) ((uint64)(_x))
#endif

#ifndef cpu_to_le16
#define cpu_to_le16(_x) ((uint16)(_x))
#define cpu_to_le32(_x) ((uint32)(_x))
#define cpu_to_le64(_x) ((uint64)(_x))
#endif

#define MSEC_2_USEC(x) ((x)*1000)
/* POINT1SEC_2_USEC is 0.1 sec */
#define POINT1SEC_2_USEC(x) ((x)*100000)
#define SEC_2_USEC(x) ((x)*1000000)

#define WDEV_DRV_E_SCHEDULE_READ 0x01
#define WDEV_DRV_E_SCHEDULE_SEND 0x02
#define WDEV_E_READ_COMPLETE 0x01
#define WDEV_E_SEND_COMPLETE 0x02
#define WDEV_E_SOFT_BOOT_COMPLETE 0x03
#define WDEV_E_ADAPTER_STARTING 0x04

/*bits 2 to 12*/
#define IOCTL_NUMBER(x) ((x>>2)&0x7FF)

#endif /* _DRIVER_H */
