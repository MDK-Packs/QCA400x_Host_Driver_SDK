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
#include <a_config.h>
#include <a_types.h>
#include <a_osapi.h>
#include <common_api.h>
#include <hcd_api.h>
#include <spi_hcd_if.h>
#include <targaddrs.h>
#include <AR6002/hw2.0/hw/mbox_host_reg.h>
#include "atheros_wifi_api.h"

/* prototypes */
static A_STATUS Driver_SetAddressWindowRegister(QOSAL_VOID *pCxt, QOSAL_UINT32 RegisterAddr, QOSAL_UINT32 Address);

/* Compile the 4BYTE version of the window register setup routine,
 * This mitigates host interconnect issues with non-4byte aligned bus requests, some
 * interconnects use bus adapters that impose strict limitations.
 * Since diag window access is not intended for performance critical operations, the 4byte mode should
 * be satisfactory even though it generates 4X the bus activity. */

#ifdef USE_4BYTE_REGISTER_ACCESS

    /* set the window address register (using 4-byte register access ). */
/*****************************************************************************/
/*  Driver_SetAddressWindowRegister - Utility function to set the window 
 *	 register. This is used for diagnostic reads and writes.
 *      QOSAL_VOID *pCxt - the driver context. 
 *		QOSAL_UINT32 RegisterAddr - The window register address.
 *		QOSAL_UINT32 Address - the target address. 
 *****************************************************************************/
static A_STATUS 
Driver_SetAddressWindowRegister(QOSAL_VOID *pCxt, QOSAL_UINT32 RegisterAddr, QOSAL_UINT32 Address)
{
    A_STATUS status;
    QOSAL_UINT8 addrValue[4];
    QOSAL_INT32 i;
	A_NETBUF_DECLARE req;
	QOSAL_VOID *pReq = (QOSAL_VOID*)&req;
	
        /* write bytes 1,2,3 of the register to set the upper address bytes, the LSB is written
         * last to initiate the access cycle */
	Address = A_CPU2LE32(Address);
	
    for (i = 1; i <= 3; i++) {
            /* fill the buffer with the address byte value we want to hit 4 times*/
        addrValue[0] = ((QOSAL_UINT8 *)&Address)[i];
        addrValue[1] = addrValue[0];
        addrValue[2] = addrValue[0];
        addrValue[3] = addrValue[0];

		A_NETBUF_CONFIGURE(pReq, (QOSAL_VOID*)addrValue, 0, 4, 4);	
		ATH_SET_PIO_EXTERNAL_WRITE_OPERATION(pReq, RegisterAddr+i, A_FALSE, 4);
		
		if(A_OK != (status = Hcd_DoPioExternalAccess(pCxt, pReq))){
			break;
		}
    }

    if (status != A_OK) {        
        return status;
    }

	A_NETBUF_CONFIGURE(pReq, (QOSAL_VOID*)&Address, 0, 4, 4);	
	ATH_SET_PIO_EXTERNAL_WRITE_OPERATION(pReq, RegisterAddr, A_TRUE, 4);
	status = Hcd_DoPioExternalAccess(pCxt, pReq);
	
	return status;
}


#else
    
/*****************************************************************************/
/*  Driver_SetAddressWindowRegister - Utility function to set the window 
 *	 register. This is used for diagnostic reads and writes.
 *      QOSAL_VOID *pCxt - the driver context. 
 *		QOSAL_UINT32 RegisterAddr - The window register address.
 *		QOSAL_UINT32 Address - the target address. 
 *****************************************************************************/
static A_STATUS 
Driver_SetAddressWindowRegister(QOSAL_VOID *pCxt, QOSAL_UINT32 RegisterAddr, QOSAL_UINT32 Address)
{
    A_STATUS status;
	A_NETBUF_DECLARE req;
	QOSAL_VOID *pReq = (QOSAL_VOID*)&req;
	
	Address = A_CPU2LE32(Address);
	
	do{
		A_NETBUF_CONFIGURE(pReq, (((QOSAL_UCHAR *)(&Address))+1), (sizeof(QOSAL_UINT32)-1));	
		ATH_SET_PIO_EXTERNAL_WRITE_OPERATION(pReq, RegisterAddr+1, A_TRUE, (sizeof(QOSAL_UINT32)-1));
		
		if(A_OK != (status = Hcd_DoPioExternalAccess(pCxt, pReq))){
			break;
		}
		
		A_NETBUF_CONFIGURE(pReq, ((QOSAL_UCHAR *)(&Address)), sizeof(QOSAL_UINT8));	
		ATH_SET_PIO_EXTERNAL_WRITE_OPERATION(pReq, RegisterAddr, A_TRUE, sizeof(QOSAL_UINT8));
		
		if(A_OK != (status = Hcd_DoPioExternalAccess(pCxt, pReq))){
			break;
		}
	}while(0);

	return status;
}

#endif

/*****************************************************************************/
/*  Driver_ReadRegDiag - Reads four bytes of data from the specified chip
 *	 device address.
 *      QOSAL_VOID *pCxt - the driver context. 
 *		QOSAL_UINT32 address - the device chip address to start the read.
 *		QOSAL_UCHAR *data - the start of data destination buffer.
 *****************************************************************************/
A_STATUS
Driver_ReadRegDiag(QOSAL_VOID *pCxt, QOSAL_UINT32 *address, QOSAL_UINT32 *data)
{
    A_STATUS status;
	A_NETBUF_DECLARE req;
	QOSAL_VOID *pReq = (QOSAL_VOID*)&req;
	
	A_NETBUF_CONFIGURE(pReq, data, 0, sizeof(QOSAL_UINT32), sizeof(QOSAL_UINT32));
	
	do{
        /* set window register to start read cycle */
    	if(A_OK != (status = Driver_SetAddressWindowRegister(pCxt,
                                             WINDOW_READ_ADDR_ADDRESS,
                                             *address))){
        	break;
        }

		ATH_SET_PIO_EXTERNAL_READ_OPERATION(pReq, WINDOW_DATA_ADDRESS, A_TRUE, sizeof(QOSAL_UINT32));
		
		if(A_OK != (status = Hcd_DoPioExternalAccess(pCxt, pReq))){
			break;
		}
	}while(0);    

    return status;
}

/*****************************************************************************/
/*  Driver_WriteRegDiag - Writes four bytes of data to the specified chip
 *	 device address.
 *      QOSAL_VOID *pCxt - the driver context. 
 *		QOSAL_UINT32 address - the device chip address to start the write.
 *		QOSAL_UCHAR *data - the start of data source buffer.
 *****************************************************************************/
A_STATUS
Driver_WriteRegDiag(QOSAL_VOID *pCxt, QOSAL_UINT32 *address, QOSAL_UINT32 *data)
{
    A_STATUS status;

	A_NETBUF_DECLARE req;
	QOSAL_VOID *pReq = (QOSAL_VOID*)&req;
	
	A_NETBUF_CONFIGURE(pReq, data, 0, sizeof(QOSAL_UINT32), sizeof(QOSAL_UINT32));
	
	
	ATH_SET_PIO_EXTERNAL_WRITE_OPERATION(pReq, WINDOW_DATA_ADDRESS, A_TRUE, sizeof(QOSAL_UINT32));
	
	do{
		if(A_OK != (status = Hcd_DoPioExternalAccess(pCxt, pReq))){
			break;
		}
        /* set window register, which starts the write cycle */
	    if(A_OK != (status = Driver_SetAddressWindowRegister(pCxt,
	                                           WINDOW_WRITE_ADDR_ADDRESS,
	                                           *address))){
	    	break;                                       
	    }
	}while(0);
	
	return status;                                           
}

/*****************************************************************************/
/*  Driver_ReadDataDiag - Reads a buffer of data starting from address. Length
 * 	 of data is specified by length. Data is read from a contiguous chip
 *	 address memory/register space.
 *      QOSAL_VOID *pCxt - the driver context. 
 *		QOSAL_UINT32 address - the device chip address to start the read.
 *		QOSAL_UCHAR *data - the start of data destination buffer.
 *		QOSAL_UINT32 length - the length of data in bytes.
 *****************************************************************************/
A_STATUS
Driver_ReadDataDiag(QOSAL_VOID *pCxt, QOSAL_UINT32 address,
                    QOSAL_UCHAR *data, QOSAL_UINT32 length)
{
    QOSAL_UINT32 count;
    A_STATUS status = A_OK;

    for (count = 0; count < length; count += 4, address += 4) {
        if ((status = Driver_ReadRegDiag(pCxt, &address,
                                         (QOSAL_UINT32 *)&data[count])) != A_OK)
        {
            break;
        }
    }

    return status;
}

/*****************************************************************************/
/*  Driver_WriteDataDiag - Writes a buffer of data starting at address. Length
 * 	 of data is specified by length. Data is written to a contiguous chip
 *	 address memory/register space.
 *      QOSAL_VOID *pCxt - the driver context. 
 *		QOSAL_UINT32 address - the device chip address to start the write.
 *		QOSAL_UCHAR *data - the start of data source buffer.
 *		QOSAL_UINT32 length - the length of data in bytes.
 *****************************************************************************/
A_STATUS
Driver_WriteDataDiag(QOSAL_VOID *pCxt, QOSAL_UINT32 address,
                    QOSAL_UCHAR *data, QOSAL_UINT32 length)
{
    QOSAL_UINT32 count;
    A_STATUS status = A_OK;

    for (count = 0; count < length; count += 4, address += 4) {
        if ((status = Driver_WriteRegDiag(pCxt, &address,
                                         (QOSAL_UINT32 *)&data[count])) != A_OK)
        {
            break;
        }
    }

    return status;
}


#define REG_DUMP_COUNT_AR4100   60
#define REG_DUMP_COUNT_AR4002   60
#define REGISTER_DUMP_LEN_MAX   60

/*****************************************************************************/
/*  Driver_DumpAssertInfo - Collects assert information from chip and writes
 *	 it to stdout.
 *      QOSAL_VOID *pCxt - the driver context. 
 *		QOSAL_UINT32 *pData - buffer to store UINT32 results
 *		QOSAL_UINT16 *pLength - IN/OUT param to store length of buffer for IN and
 *		 length of contents for OUT.     
 *****************************************************************************/
A_STATUS
Driver_DumpAssertInfo(QOSAL_VOID *pCxt, QOSAL_UINT32 *pData, QOSAL_UINT16 *pLength)
{
	A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
    QOSAL_UINT32 address;
    QOSAL_UINT32 regDumpArea = 0;
    A_STATUS status = A_ERROR;   
    QOSAL_UINT32 regDumpCount = 0;
    
		
    do {
		if(*pLength < REGISTER_DUMP_LEN_MAX){
			break;
		}
		/* clear it */
		*pLength = 0;
            /* the reg dump pointer is copied to the host interest area */
        address = HOST_INTEREST_ITEM_ADDRESS(hi_failure_state);
        address = TARG_VTOP(address);

        if (pDCxt->targetType == TARGET_TYPE_AR4100 || 
        	pDCxt->targetType == TARGET_TYPE_AR400X) {
            regDumpCount = REG_DUMP_COUNT_AR4100;
        } else {
            A_ASSERT(0); /* should never happen */
#if DRIVER_CONFIG_DISABLE_ASSERT            
            break;      
#endif            
        }

            /* read RAM location through diagnostic window */
        if(A_OK != (status = Driver_ReadRegDiag(pCxt, &address, &regDumpArea))){
        	A_ASSERT(0); /* should never happen */
#if DRIVER_CONFIG_DISABLE_ASSERT            
            break;      
#endif          
        }

		regDumpArea = A_LE2CPU32(regDumpArea);

        if (regDumpArea == 0) {
                /* no reg dump */
            break;
        }

        regDumpArea = TARG_VTOP(regDumpArea);

            /* fetch register dump data */
        if(A_OK != (status = Driver_ReadDataDiag(pCxt,
                                     regDumpArea,
                                     (QOSAL_UCHAR *)&pData[0],
                                     regDumpCount * (sizeof(QOSAL_UINT32))))){
        	A_ASSERT(0); /* should never happen */
#if DRIVER_CONFIG_DISABLE_ASSERT            
            break;      
#endif                                      
        }      
        
        *pLength = regDumpCount;              
    } while (0);


	return status;
}
