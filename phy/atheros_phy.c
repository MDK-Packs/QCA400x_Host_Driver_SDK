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
#include <wlan_config.h>
#include <a_types.h>
#include <a_osapi.h>
#include <driver_cxt.h>
#include "mqx.h"
#include "bsp.h"
#include "bsp_prv.h"
#include "enet.h"
#include "enetprv.h"
#include "atheros_phy.h"
#if BSPCFG_ENABLE_ATHEROS_WIFI

static boolean phy_atheros_wifi_discover_addr(ENET_CONTEXT_STRUCT_PTR enet_ptr);
static boolean phy_atheros_wifi_init(ENET_CONTEXT_STRUCT_PTR enet_ptr);
static uint_32 phy_atheros_wifi_get_speed(ENET_CONTEXT_STRUCT_PTR enet_ptr);
static boolean phy_atheros_wifi_get_link_status(ENET_CONTEXT_STRUCT_PTR enet_ptr);

const ENET_PHY_IF_STRUCT ATHEROS_PHY_IF = {
    phy_atheros_wifi_discover_addr,
    phy_atheros_wifi_init,
    phy_atheros_wifi_get_speed,
    phy_atheros_wifi_get_link_status
};
  
  

/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : phy_atheros_wifi_discover_addr
*  Returned Value : none
*  Comments       :
*    Scan possible PHY addresses looking for a valid device
*
*END*-----------------------------------------------------------------*/

static boolean phy_atheros_wifi_discover_addr
   (
       ENET_CONTEXT_STRUCT_PTR	   enet_ptr
   )
{ 
	UNUSED_ARGUMENT(enet_ptr);
    return TRUE;
} 


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : phy_atheros_wifi_init
*  Returned Value : boolean
*  Comments       :
*    Wait for PHY to automatically negotiate its configuration
*
*END*-----------------------------------------------------------------*/

static boolean phy_atheros_wifi_init
   (
       ENET_CONTEXT_STRUCT_PTR	   enet_ptr
   )
{ 
	UNUSED_ARGUMENT(enet_ptr);
   return TRUE;
}  


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : phy_atheros_wifi_get_speed
*  Returned Value : uint_32 - connection speed
*  Comments       :
*    Determine connection speed.
*
*END*-----------------------------------------------------------------*/

static uint_32 phy_atheros_wifi_get_speed
   (
       ENET_CONTEXT_STRUCT_PTR	   enet_ptr
   )
{ 
	UNUSED_ARGUMENT(enet_ptr);
   return 54;
} 

  
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name  : phy_atheros_wifi_get_link_status
*  Returned Value : TRUE if link active, FALSE otherwise
*  Comments       : 
*    Get actual link status.
*
*END*-----------------------------------------------------------------*/

static boolean phy_atheros_wifi_get_link_status
   (
       ENET_CONTEXT_STRUCT_PTR	   enet_ptr
   )
{ 
    boolean res = FALSE;
    QOSAL_VOID *pCxt;
    
    if((QOSAL_VOID*)enet_ptr->MAC_CONTEXT_PTR != NULL){
    	pCxt = (QOSAL_VOID*)enet_ptr->MAC_CONTEXT_PTR;
    	if(QOSAL_TRUE == GET_DRIVER_COMMON(pCxt)->conn[enet_ptr->PARAM_PTR->ENET_IF->PHY_NUMBER].isConnected){       
            res = TRUE;
        }        
    }
  
    return res;
} 
#endif /*BSPCFG_ENABLE_ATHEROS_WIFI*/

/*EOF*/    
