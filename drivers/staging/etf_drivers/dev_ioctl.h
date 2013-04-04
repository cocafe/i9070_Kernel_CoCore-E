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
* dev_ioctl.h
*
* This file defines the ioctls called for character device
*
****************************************************************************/

#ifndef _DEV_IOCTL_H
#define _DEV_IOCTL_H

#include <linux/ioctl.h>
#define MAJOR_NUM 100

#define IOCTL_ST90TDS_ADAP_READ      _IOWR(MAJOR_NUM, 1, char*)
#define IOCTL_ST90TDS_ADAP_WRITE     _IOWR(MAJOR_NUM, 2, char*)
#define IOCTL_ST90TDS_REQ_CNF        _IOWR(MAJOR_NUM, 3, char*)
#define IOCTL_ST90TDS_AHA_REG_READ   _IOWR(MAJOR_NUM, 4, char*)
#define IOCTL_ST90TDS_AHA_REG_WRITE  _IOWR(MAJOR_NUM, 5, char*)
#define IOCTL_ST90TDS_CUSTOM_CMD     _IOWR(MAJOR_NUM, 6, char*)
#define IOCTL_ST90TDS_REQ_CNF_READ   _IOWR(MAJOR_NUM, 7, char*)
#define IOCTL_ST90TDS_REQ_CNF_SIZE   _IOWR(MAJOR_NUM, 8, char*)
#define IOCTL_ST90TDS_WRITE_CNF_SIZE _IOWR(MAJOR_NUM, 9, char*)
#define IOCTL_ST90TDS_ADAP_READ_SIZE _IOWR(MAJOR_NUM, 10, char*)
/*similarly define other ioctls*/
/* to detect chip version */
#define IOCTL_ST90TDS_CHIP_VERSION   _IOWR(MAJOR_NUM, 11, char*)

#define DEVICE_NAME "char_dev"

#endif /* _DEV_IOCTL_H */
