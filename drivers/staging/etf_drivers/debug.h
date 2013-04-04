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
* debug.h
*
* This module serves as the lower interface header file for the adapter module
* and all driver's module.
*
****************************************************************************/

#ifndef _DEBUG_H
#define _DEBUG_H

extern int debug_level;

#define DBG_NONE    0x00000000
#define DBG_SBUS    0x00000001
#define DBG_MESSAGE 0x00000002
#define DBG_FUNC    0x00000004
#define DBG_ERROR   0x00000008
#define DBG_ALWAYS  DBG_ERROR

#define DEBUG(f, args... ) do{if(f & debug_level) printk(args);}while(0)

#endif /* _DEBUG_H */
