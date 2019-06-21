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
#include <a_config.h>
#include <a_types.h>
#include <a_osapi.h>
#include <common_api.h>
#include <custom_wlan_api.h>
#include <htc.h>
#include <atheros_wifi_api.h>

#include "QCA400x.h"

#if (!ENABLE_STACK_OFFLOAD)

int32_t
Custom_Api_Send(QCA400x_WiFi *pDev, A_NATIVE_NETBUF * pcb_ptr, uint32_t size, uint32_t frags, uint32_t flags);

/*****************************************************************************/
/*  Custom_DeliverFrameToNetworkStack - Called by API_RxComplete to 
 *   deliver a data frame to the network stack. This code will perform 
 *   platform specific operations.
 *      QOSAL_VOID *pCxt - the driver context. 
 *      QOSAL_VOID *pReq - the packet.
 *****************************************************************************/
QOSAL_VOID 
Custom_DeliverFrameToNetworkStack(QOSAL_VOID *pCxt, QOSAL_VOID *pReq)
{
    A_NETBUF* a_netbuf_ptr = (A_NETBUF*)pReq;
	QOSAL_UINT32 len;		
	A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
	QCA400x_WiFi * pDev = (QCA400x_WiFi *)Custom_Api_GetDriverCxt(pDCxt->devId);
    ATH_PROMISCUOUS_CB prom_cb = (ATH_PROMISCUOUS_CB)(GET_DRIVER_CXT(pCxt)->promiscuous_cb);
    
    if(a_netbuf_ptr) {    	        
    	len = A_NETBUF_LEN(pReq);
//    	_DCACHE_INVALIDATE_MBYTES(A_NETBUF_DATA(pReq), len);
        
        if(pDCxt->promiscuous_mode){				    	
	    	if(prom_cb){
	    		/* send frame to callback function */
				a_netbuf_ptr->native.FRAG[0].LENGTH = len;
		    	a_netbuf_ptr->native.FRAG[0].FRAGMENT = A_NETBUF_DATA(pReq);
	    		prom_cb((QOSAL_VOID*)&(a_netbuf_ptr->native));			
	    	}else{
	    		A_NETBUF_FREE(pReq);
	    	}
        }else if (pDev->FrameReceived_cb != NULL){
        	// Call user Callback function
        	a_netbuf_ptr->native.FRAG[0].LENGTH = len;
	    	a_netbuf_ptr->native.FRAG[0].FRAGMENT = A_NETBUF_DATA(pReq);	    			    		    		    	
    		pDev->FrameReceived_cb(a_netbuf_ptr);
    	}else{    	
            /* failed to find a receiver for this data packet. */
    		A_NETBUF_FREE(pReq);
    	}    
    }   
}

/*****************************************************************************/
/*  Custom_Api_Send - Entry point for WiFi driver interface to send a packet.
 *      QCA400x_WiFi         pDev - pointer to QCA400x_WiFi structure
 *      A_NATIVE_NETBUF *    pcb_ptr - the packet object.
 *      uint_32              size - the length in bytes of pcb_ptr.
 *      uint_32              frags - the number of fragments in pcb_ptr.
 *      uint_32              flags - optional flags for transmit.
 *****************************************************************************/
int32_t Custom_Api_Send (QCA400x_WiFi *pDev, A_NATIVE_NETBUF * pcb_ptr, uint32_t size, uint32_t frags, uint32_t flags)
{ 
    int_32 error = A_OK;
    A_NETBUF* a_netbuf_ptr;
    UNUSED_ARGUMENT(flags);
    UNUSED_ARGUMENT(size);

    /* create an atheros pcb and continue or fail. */
    do{    	
    	/* provide enough space in the top buffer to store the 14 byte 
    	 * header which will be copied from its position in the original
    	 * buffer. this will allow wmi_dix_2_dot3() to function properly.
    	 */
		if((a_netbuf_ptr = (A_NETBUF*)A_NETBUF_ALLOC(sizeof(ATH_MAC_HDR))) == NULL){    	
    		error = A_ERROR;
    		break;
    	}
    
		a_netbuf_ptr->num_frags = (QOSAL_UINT8)frags;   
		/* HACK: copy the first part of the fragment to the new buf in order to allow 
		 * wmi_dix_2_dot3() to function properly. */
		A_ASSERT(pcb_ptr->FRAG[0].LENGTH >= sizeof(ATH_MAC_HDR));
		A_NETBUF_PUT_DATA(a_netbuf_ptr, (QOSAL_UINT8*)pcb_ptr->FRAG[0].FRAGMENT, sizeof(ATH_MAC_HDR));
		/*(QOSAL_CHAR*)pcb_ptr->FRAG[0].FRAGMENT += sizeof(ATH_MAC_HDR);*/
                pcb_ptr->FRAG[0].FRAGMENT = (QOSAL_UCHAR*)((QOSAL_UINT32)pcb_ptr->FRAG[0].FRAGMENT + sizeof(ATH_MAC_HDR));
		pcb_ptr->FRAG[0].LENGTH -= sizeof(ATH_MAC_HDR);

    	//ensure there is enough headroom to complete the tx operation
    	if (A_NETBUF_HEADROOM(a_netbuf_ptr) < sizeof(ATH_MAC_HDR) + sizeof(ATH_LLC_SNAP_HDR) +
            		sizeof(WMI_DATA_HDR) + HTC_HDR_LENGTH + WMI_MAX_TX_META_SZ) {
            error = A_ERROR;            
    		break;   		
        }
    	//carry the original around until completion.
    	//it is freed by A_NETBUF_FREE  
    	a_netbuf_ptr->native_orig = pcb_ptr;       
    	
  		if(A_OK != Api_DataTxStart((void *)pDev->MAC_CONTEXT_PTR, (void *)a_netbuf_ptr)){
    		error = A_ERROR;
    		break;
    	}
    	    
   	}while(0);
           
	if(error != A_OK){
		/* in the event of api failure, the original pcb should be returned to the caller un-touched
		 * and the a_netbuf_ptr should be freed */
		a_netbuf_ptr->native_orig = NULL;
		A_NETBUF_FREE((QOSAL_VOID*)a_netbuf_ptr);
	}           
           
    return error;
}

#endif
