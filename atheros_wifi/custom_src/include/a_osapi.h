//------------------------------------------------------------------------------
// Copyright (c) 2017 Qualcomm Technologies, Inc.
// All rights reserved.
// Redistribution and use in source and binary forms, with or without modification, are permitted (subject to the limitations in the disclaimer below) 
// provided that the following conditions are met:
// Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, 
// this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// Neither the name of Qualcomm Technologies, Inc. nor the names of its contributors may be used to endorse or promote products derived 
// from this software without specific prior written permission.
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
// BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
// OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//
// Modified by Arm
//==============================================================================
#ifndef _A_OSAPI_H_
#define _A_OSAPI_H_

/* PORT_NOTE: Include any System header files here to resolve any types used
 *  in A_CUSTOM_DRIVER_CONTEXT */ 
#include <string.h>

#include "a_config.h"
#include "a_types.h"
#include <athdefs.h>
#include <cust_netbuf.h>
#include <osal.h>

#include "cmsis_compiler.h"

#define MEM_TYPE_ATHEROS_PERSIST_RX_PCB 0x2001

/* there exist within the common code a few places that make use of the 
 * inline option.  This macro can be used to properly declare inline for the compiler 
 * or it can be used to disable inlining be leaving the macro blank.
 */
#ifndef INLINE
#define INLINE              __INLINE
#endif
/* PREPACK and/or POSTPACK are used with compilers that require each data structure
 * that requires single byte alignment be declared individually. these macros typically
 * would be used in place of athstartpack.h and athendpack.h which would be used with 
 * compilers that allow this feature to be invoked through a one-time pre-processor command.
 */
#ifndef PREPACK_STRUCT
#define PREPACK_STRUCT      __PACKED_STRUCT
#endif
#ifndef PREPACK_UNION
#define PREPACK_UNION       __PACKED_UNION
#endif
 
#ifndef FIELD_PACKED
#define FIELD_PACKED        __PACKED
#endif
#ifndef FIELD_PREPACKED
#define FIELD_PREPACKED
#endif
#ifndef PREPACK
#define PREPACK             __PACKED
#endif /* PREPACK */
#ifndef POSTPACK
#define POSTPACK            __PACKED
#endif /* POSTPACK */

#ifndef min
#define min(a,b) ((a) < (b))? (a) : (b)
#endif

#ifndef max
#define max(a,b) ((a) > (b))? (a) : (b)
#endif


/* unaligned little endian access */
#define A_LE_READ_2(p)                                              \
        ((QOSAL_UINT16)(                                                \
        (((QOSAL_UINT8 *)(p))[0]) | (((QOSAL_UINT8 *)(p))[1] <<  8)))

#define A_LE_READ_4(p)                                              \
        ((QOSAL_UINT32)(                                                \
        (((QOSAL_UINT8 *)(p))[0]      ) | (((QOSAL_UINT8 *)(p))[1] <<  8) | \
        (((QOSAL_UINT8 *)(p))[2] << 16) | (((QOSAL_UINT8 *)(p))[3] << 24)))
/*
 * Endianes macros - Used to achieve proper translation to/from wifi chipset endianess.
 * these macros should be directed to OS macros or functions that serve this purpose. the
 * naming convention is the direction of translation (BE2CPU == BigEndian to native CPU) 
 * appended with the bit-size (BE2CPU16 == 16 bit operation BE2CPU32 == 32 bit operation
 */   


#if DRIVER_CONFIG_ENDIANNESS == A_LITTLE_ENDIAN
#define A_BE2CPU16(x)   ((QOSAL_UINT16)(((x)<<8)&0xff00) | (QOSAL_UINT16)(((x)>>8)&0x00ff))
#define A_BE2CPU32(x)   ((((x)<<24)&0xff000000) | (((x)&0x0000ff00)<<8) | (((x)&0x00ff0000)>>8) | (((x)>>24)&0x000000ff))
#define A_LE2CPU16(x)   (x)
#define A_LE2CPU32(x)   (x)
#define A_CPU2BE16(x)   ((QOSAL_UINT16)(((x)<<8)&0xff00) | (QOSAL_UINT16)(((x)>>8)&0x00ff))
#define A_CPU2BE32(x)   ((((x)<<24)&0xff000000) | (((x)&0x0000ff00)<<8) | (((x)&0x00ff0000)>>8) | (((x)>>24)&0x000000ff))
#define A_CPU2LE32(x)   (x)
#define A_CPU2LE16(x)   (x)
#define A_ENDIAN A_LITTLE_ENDIAN
#elif DRIVER_CONFIG_ENDIANNESS == A_BIG_ENDIAN
#define A_BE2CPU16(x)   (x)
#define A_BE2CPU32(x)   (x)
#define A_LE2CPU16(x)   ((QOSAL_UINT16)(((x)<<8)&0xff00) | (QOSAL_UINT16)(((x)>>8)&0x00ff))
#define A_LE2CPU32(x)   ((((x)<<24)&0xff000000) | (((x)&0x0000ff00)<<8) | (((x)&0x00ff0000)>>8) | (((x)>>24)&0x000000ff))
#define A_CPU2BE16(x)   (x)
#define A_CPU2BE32(x)   (x)
#define A_CPU2LE32(x)   ((((x)<<24)&0xff000000) | (((x)&0x0000ff00)<<8) | (((x)&0x00ff0000)>>8) | (((x)>>24)&0x000000ff))
#define A_CPU2LE16(x)   ((QOSAL_UINT16)(((x)<<8)&0xff00) | (QOSAL_UINT16)(((x)>>8)&0x00ff))
#define A_ENDIAN A_BIG_ENDIAN
#else
#error DRIVER_CONFIG_ENDIANNESS is not properly defined!!!
#endif

/* A_MEM -- macros that should be mapped to OS/STDLIB equivalent functions */
#define A_MEMCPY(dst, src, len)         memcpy((QOSAL_UINT8 *)(dst), (src), (len))
#define A_MEMZERO(addr, len)            memset((addr), 0, (len))
#define A_MEMCMP(addr1, addr2, len)     memcmp((addr1), (addr2), (len))

extern QOSAL_UINT32 g_totAlloc;
extern QOSAL_UINT32 g_poolid;

#define DRIVER_STATE_INVALID  0
#define DRIVER_STATE_INIT     1
#define DRIVER_STATE_RUN      2
#define DRIVER_STATE_SHUTDOWN 3
extern QOSAL_UINT8 g_driverState;

extern A_VOID   qosal_malloc_init(A_VOID);
extern A_VOID*  qosal_malloc(A_UINT32 size);
extern A_STATUS qosal_free(A_VOID* addr);
/* Definitions used for ID param in calls to A_MALLOC and A_FREE */
//		NAME					VALUE 	DESCRIPTION
#define MALLOC_ID_CONTEXT 		1	// persistent allocation for the life of Driver
#define MALLOC_ID_NETBUFF 		2	// buffer used to perform TX/RX SPI transaction
#define MALLOC_ID_NETBUFF_OBJ 	3	// NETBUF object
#define MALLOC_ID_TEMPORARY     4   // other temporary allocation
/* Macros used for heap memory allocation and free */
#define A_MALLOCINIT()                  qosal_malloc_init()
#define A_MALLOC(size, id)              qosal_malloc(size)
#define A_FREE(addr, id)                qosal_free(addr)

#ifdef A_PRINTF_ENABLED
#define A_PRINTF(args...)               printf(args)
#else
#define A_PRINTF(args...)
#endif

/* Mutual Exclusion */
typedef MUTEX_STRUCT                    A_MUTEX_T;
/* use default mutex attributes */
#define A_MUTEX_INIT(mutex)             (((A_OK != qosal_mutex_init((mutex)/*, NULL*/)))? A_ERROR:A_OK)                      
    
#define A_MUTEX_ACQUIRE(mutex)             (((A_OK != qosal_mutex_acquire((mutex), 0)))? A_ERROR:A_OK)
#define A_MUTEX_RELEASE(mutex)           (((A_OK != qosal_mutex_release((mutex))))? A_ERROR:A_OK)
#define A_IS_MUTEX_VALID(mutex)         A_TRUE  /* okay to return true, since A_MUTEX_DELETE does nothing */
#define A_MUTEX_DELETE(mutex)           (((A_OK != qosal_mutex_destroy((mutex))))? A_ERROR:A_OK)

/*
 * A_MDELAY - used to delay specified number of milliseconds.
 */
#define A_MDELAY(msecs)                 qosal_time_delay((msecs))


#if DRIVER_CONFIG_DISABLE_ASSERT
#define A_ASSERT(expr)
#else /* DRIVER_CONFIG_DISABLE_ASSERT */
extern QOSAL_VOID *p_Global_Cxt;    
extern void assert_func(const char *, QOSAL_UINT32);
#define A_ASSERT(expr) if(!(expr)) do{ assert_func(__FUNCTION__, __LINE__);}while(0)
#endif /* DRIVER_CONFIG_DISABLE_ASSERT */

/* UNUSED_ARGUMENT - used to solve compiler warnings where arguments to functions are not used
 * within the function. 
 */
#define UNUSED_ARGUMENT(arg) ((void)arg)

#include <cust_context.h>



#endif /* _OSAPI_MQX_H_ */
