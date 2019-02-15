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
//==============================================================================

#ifndef _A_TYPES_H_
#define _A_TYPES_H_

#include "stdint.h"
#include "stdbool.h"

#define _PTR_   *
#define TRUE    true
#define FALSE   false
typedef uint8_t uchar;

#define int_8_ptr   int8_t*
#define uint_8_ptr  uint8_t*
#define uint_32_ptr uint32_t*
#define char_ptr    int8_t*

typedef int8_t      QOSAL_INT8;
typedef int16_t     QOSAL_INT16;
typedef int32_t     QOSAL_INT32;
typedef uint8_t     QOSAL_UINT8;
typedef uint16_t    QOSAL_UINT16;
typedef uint32_t    QOSAL_UINT32;

/* NOTE: QOSAL_BOOL is a type that is used in various WMI commands and events.
 * as such it is a type that is shared between the wifi chipset and the host.
 * It is required for that reason that QOSAL_BOOL be treated as a 32-bit/4-byte type.
 */
typedef int32_t     QOSAL_BOOL;
typedef char        QOSAL_CHAR;
typedef uint8_t     QOSAL_UCHAR;
typedef void        QOSAL_VOID;
typedef uint32_t    QOSAL_ULONG;
typedef int32_t     QOSAL_LONG;

typedef void*       pointer;
#define QOSAL_CONST const

typedef int8_t      int_8;
typedef int16_t     int_16;
typedef int32_t     int_32;

typedef uint8_t     uint_8;
typedef uint16_t    uint_16;
typedef uint32_t    uint_32;

typedef QOSAL_BOOL  A_BOOL;

typedef void        A_VOID;
typedef char        A_CHAR;


typedef int8_t      A_INT8;
typedef int16_t     A_INT16;
typedef int32_t     A_INT32;

typedef uint8_t     A_UINT8;
typedef uint16_t    A_UINT16;
typedef uint32_t    A_UINT32;
typedef uint32_t    A_ULONG;

#define A_TRUE      true
#define A_FALSE     false

#endif /* _A_TYPES_H_ */
