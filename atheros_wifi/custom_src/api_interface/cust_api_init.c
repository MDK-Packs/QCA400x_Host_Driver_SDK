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
#include "a_osapi.h"
#include "QCA400x.h"
#include "common_api.h"

void *p_Global_Cxt = NULL;

static int32_t Custom_Api_Readmii    (QCA400x_WiFi * pDev, uint32_t val,  uint32_t * p_val, uint32_t val2);
static int32_t Custom_Api_Writemii   (QCA400x_WiFi * pDev, uint32_t val1, uint32_t   val2 , uint32_t val3);
static int32_t Custom_Api_Join       (QCA400x_WiFi * pDev, MCB_STRUCT * p_mcb);
static int32_t Custom_Api_Rejoin     (QCA400x_WiFi * pDev);
static int32_t Custom_Api_Initialize (QCA400x_WiFi * pDev);
static int32_t Custom_Api_Shutdown   (QCA400x_WiFi * pDev);
extern int32_t Custom_Api_Send       (QCA400x_WiFi * pDev, void * pcb_ptr, uint32_t size, uint32_t frags, uint32_t flags);
extern int32_t Custom_Api_Mediactl   (uint8_t device_id, uint32_t command_id, void * inout_param);

const MAC_IF ATHEROS_WIFI_IF = {
    Custom_Api_Initialize,
    Custom_Api_Shutdown,
    Custom_Api_Send,
    Custom_Api_Readmii,
    Custom_Api_Writemii,
    Custom_Api_Join,
    Custom_Api_Rejoin,
    Custom_Api_Mediactl
};

static int32_t Custom_Api_Readmii(QCA400x_WiFi * pDev, uint32_t val, uint32_t * p_val, uint32_t val2)

{
    /* NOTHING TO DO HERE */
    UNUSED_ARGUMENT(pDev);
    UNUSED_ARGUMENT(val);
    UNUSED_ARGUMENT(p_val);
    UNUSED_ARGUMENT(val2);
//    while(1){};
    return 0;
}

static int32_t Custom_Api_Writemii(QCA400x_WiFi * pDev, uint32_t val1, uint32_t val2 , uint32_t val3)
{
    /* NOTHING TO DO HERE */
    UNUSED_ARGUMENT(pDev);
    UNUSED_ARGUMENT(val1);
    UNUSED_ARGUMENT(val2);
    UNUSED_ARGUMENT(val3);
//    while(1){};
    return 0;
}

static int32_t Custom_Api_Join (QCA400x_WiFi * pDev, MCB_STRUCT * p_mcb)
{
    /* NOTHING TO DO HERE */
    UNUSED_ARGUMENT(pDev);
    UNUSED_ARGUMENT(p_mcb);
//    while(1){};
    return 0;
}

static int32_t Custom_Api_Rejoin(QCA400x_WiFi * pDev)
{
    /* NOTHING TO DO HERE */
    UNUSED_ARGUMENT(pDev);
    //while(1){};
    return 0;
}

/*****************************************************************************/
/*  Custom_Api_Initialize - Entry point initialize the Driver.
 *
 * RETURNS: A_OK on success or A_ERROR otherwise.
 *****************************************************************************/
static int32_t Custom_Api_Initialize(QCA400x_WiFi * pDev)
{

    // Only one instance is supported
    if (pDev == NULL)
    {
        // Invalid pDev
        return A_ERROR;
    }

    // Only one instance is supported
    if (p_Global_Cxt != NULL)
    {
        // Already initialized
        return A_OK;
    }

    A_STATUS status = A_OK;


    do{
        // allocate the driver context and assign it to the enet_ptr mac_param
        if(NULL == (p_Global_Cxt = (QOSAL_VOID*)A_MALLOC(sizeof(A_CUSTOM_DRIVER_CONTEXT), MALLOC_ID_CONTEXT))){
            status = A_ERROR;
            break;
        }

        A_MEMZERO(p_Global_Cxt, sizeof(A_CUSTOM_DRIVER_CONTEXT));
        // alloc the driver common context
        if(NULL == (GET_DRIVER_CXT(p_Global_Cxt)->pCommonCxt = A_MALLOC(sizeof(A_DRIVER_CONTEXT), MALLOC_ID_CONTEXT))){
            status = A_ERROR;
            break;
        }
                
        pDev->MAC_CONTEXT_PTR = (pointer)p_Global_Cxt;
        /* initialize backwards pointers */
        GET_DRIVER_CXT(p_Global_Cxt)->pUpperCxt[0] = pDev;
//      GET_DRIVER_CXT(p_Global_Cxt)->pDriverParam = enet_ptr->PARAM_PTR->MAC_PARAM;
        /* create the 2 driver events. */    
     
        /* Api_InitStart() will perform any additional allocations that are done as part of 
        * the common_driver initialization */
        if(A_OK != (status = Api_InitStart(p_Global_Cxt))){
            status = A_ERROR;
            break;
        }
        /* CreateDriverThread is a custom function to create or restart the driver thread.
        * the bulk of the driver initialization is handled by the driver thread.
        */
#if DRIVER_CONFIG_MULTI_TASKING	 
        if(A_OK != (status = Driver_CreateThread(p_Global_Cxt))){
            status = A_ERROR;
            break;
        }         
#else
        Driver_Init(p_Global_Cxt);
#endif        
        /* Api_InitFinish waits for wmi_ready event from chip. */
        if(A_OK != Api_InitFinish(p_Global_Cxt)){
            status = A_ERROR;
            break;
        }

        g_driverState = DRIVER_STATE_RUN;

        Api_WMIInitFinish(p_Global_Cxt);
    }while(0);

    if(status != A_OK)
    {
        if(p_Global_Cxt != NULL)
        {
            A_FREE(GET_DRIVER_COMMON(p_Global_Cxt), MALLOC_ID_CONTEXT);
            A_FREE(p_Global_Cxt, MALLOC_ID_CONTEXT);
        }
    }
   
#if 0
if(g_totAlloc){
    A_PRINTF("init alloc: %d\n", g_totAlloc);	
    //for more information one can implement _mem_get_free() to 
    //determine how much free memory exists in the system pool.
}
#endif
 
    Api_BootProfile(p_Global_Cxt, BOOT_PROFILE_DRIVE_READY);

    return status;
}


/*****************************************************************************/
/*  Custom_Api_Shutdown - Entry point to shutdown the Driver.
 *
 * RETURNS: ENET_OK on success or ENET_ERROR otherwise. 
 *****************************************************************************/
static int32_t Custom_Api_Shutdown(QCA400x_WiFi * pDev)
{
    A_CUSTOM_DRIVER_CONTEXT *pCxt = pDev->MAC_CONTEXT_PTR;

    if(pCxt != NULL){
        Api_DeInitStart(pCxt);
#if DRIVER_CONFIG_MULTI_TASKING
        Driver_DestroyThread(pCxt);
#else
        Driver_DeInit(pCxt);
#endif
        Api_DeInitFinish(pCxt);

        if(NULL != GET_DRIVER_COMMON(pCxt)){
            A_FREE(GET_DRIVER_COMMON(pCxt), MALLOC_ID_CONTEXT);
        }

        A_FREE(pCxt, MALLOC_ID_CONTEXT);
        p_Global_Cxt = NULL;
        pDev->MAC_CONTEXT_PTR = NULL;
    }

    return (int32_t) A_OK;
}

/*****************************************************************************/
/*  Custom_Api_GetDriverCxt
 *
 * RETURNS: return driver context based on device ID
 *****************************************************************************/
void* Custom_Api_GetDriverCxt(uint8_t device_id)
{
    void * dev_ptr = NULL;

    // Only one QCA400x module supported
    if (device_id == 0) {
      if (p_Global_Cxt != NULL) {
        dev_ptr = GET_DRIVER_CXT(p_Global_Cxt)->pUpperCxt[0];
      }
    }

    return(dev_ptr);
}
