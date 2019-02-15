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

//------------------------------------------------------------------------------
// Modified by Arm
//------------------------------------------------------------------------------
// Defines used for driver configuration (at compile time):
//   - Defined in WiFi_QCA400x_Config.h
//------------------------------------------------------------------------------
//
//   WIFI_QCA400x_SPI_DRV_NUM: defines index of SPI driver used
//     - default value:    8
//   WIFI_QCA400x_SPI_TIMEOUT: defines maximum wait on SPI to become free
//     - default value:    3000
//   WIFI_QCA400x_SPI_COMPLETED_TIMEOUT: defines maximum wait for SPI transfer to complete
//     - default value:    3000
//
// Notes:
// This driver uses SPI for communicating with the module, however there are
// 2 pins that are not handled by SPI peripheral, and they are:
//  - SPI_INTR  = pending interrupt signal                 (input)
//  - CHIP_PWDN = Power down control signal (active low)   (output)
//
//
// To drive CHIP_PWDN pin you need to implement following function in WiFi_QCA400x_HW.c:
//  - WiFi_QCA400x_Pin_PWDN
//
// Following function is called from a WiFi_QCA400x_Pin_INTR function in WiFi_QCA400x_HW.c
//  - WiFi_QCA400x_Pin_INTR_cb


#include <a_config.h>
#include <a_types.h>
#include <a_osapi.h>
#include <driver_cxt.h>
#include <common_api.h>
#include <custom_wlan_api.h>
#include <hif_internal.h>
#include "spi_hcd_if.h"
#include <targaddrs.h>
#include "AR6002/hw2.0/hw/mbox_host_reg.h"
#include <atheros_wifi.h>
#include <stdlib.h> 
#include "atheros_wifi_api.h"

#include "custom_spi_api.h"
#include "Driver_SPI.h"

// Configuration definitions
#include "QCA400x_Config.h"
#define _SPI_Driver_(n)  Driver_SPI##n
#define  SPI_Driver_(n) _SPI_Driver_(n)
extern ARM_DRIVER_SPI    SPI_Driver_(WIFI_QCA400x_SPI_DRV_NUM);
#define SPIdrv         (&SPI_Driver_(WIFI_QCA400x_SPI_DRV_NUM))

#define EVENT_SPI_COMPLETED                  (1)
static osSemaphoreId_t  SPI_semaphore       = NULL;
static osEventFlagsId_t SPI_completed_event = NULL;

// WiFi_QCA400x_Pin_PWDN must be implemented by the user in WiFi_QCA400x_HW.c
extern void WiFi_QCA400x_Pin_PWDN (bool pwdn);

// Called from a WiFi_QCA400x_Pin_INTR function in WiFi_QCA400x_HW.c
void WiFi_QCA400x_Pin_INTR_cb (QCA400x_WiFi *pDev) {
  if (pDev->MAC_CONTEXT_PTR) {
    HW_InterruptHandler (pDev->MAC_CONTEXT_PTR);
  }
}

static void SPI_Callback (uint32_t event) {
  if (event & ARM_SPI_EVENT_TRANSFER_COMPLETE) {
    osEventFlagsSet(SPI_completed_event, EVENT_SPI_COMPLETED);
  }
}

QOSAL_VOID 
Custom_Hcd_EnableDisableSPIIRQ(QOSAL_VOID *pCxt, QOSAL_BOOL enable)
{

}

/*****************************************************************************/
/* Custom_Bus_InOutBuffer - This is the platform specific solution to 
 *  transfer a buffer on the SPI bus.  This solution is always synchronous
 *  regardless of sync param. The function will use the MQX fread and fwrite
 *  as appropriate.
 *      QOSAL_VOID * pCxt - the driver context.
 *      QOSAL_UINT8 *pBuffer - The buffer to transfer.
 *      QOSAL_UINT16 length - the length of the transfer in bytes.
 *      QOSAL_UINT8 doRead - 1 if operation is a read else 0.
 *      QOSAL_BOOL sync - TRUE is synchronous transfer is required else FALSE.
 *****************************************************************************/
A_STATUS 
Custom_Bus_InOutBuffer(QOSAL_VOID *pCxt,
                       QOSAL_UINT8 *pBuffer,
                       QOSAL_UINT16 length,
                       QOSAL_UINT8 doRead,
                       QOSAL_BOOL sync)
{
  int32_t  drv_ret;
  uint32_t evt_ret;
  A_STATUS status = A_ERROR;

  if (osSemaphoreAcquire(SPI_semaphore, WIFI_QCA400x_SPI_TIMEOUT) == osOK) {
    if(doRead) {
      drv_ret = SPIdrv->Receive(pBuffer, length);
    } else {
      drv_ret = SPIdrv->Send(pBuffer, length);
    }
    if (drv_ret == ARM_DRIVER_OK) {
      evt_ret = osEventFlagsWait(SPI_completed_event, EVENT_SPI_COMPLETED, osFlagsWaitAll, WIFI_QCA400x_SPI_COMPLETED_TIMEOUT);
      if (((evt_ret & 0x80000000) == 0) && ((evt_ret & EVENT_SPI_COMPLETED) == EVENT_SPI_COMPLETED)) {
        status = A_OK;
      }
    }
    osSemaphoreRelease(SPI_semaphore);
  }

  return status;
}


/*****************************************************************************/
/* Custom_Bus_Start_Transfer - This function is called by common layer prior
 *  to starting a new bus transfer. This solution merely sets up the SPI 
 *  mode as a precaution.
 *      QOSAL_VOID * pCxt - the driver context.     
 *      QOSAL_BOOL sync - TRUE is synchronous transfer is required else FALSE.
 *****************************************************************************/
A_STATUS 
Custom_Bus_StartTransfer(QOSAL_VOID *pCxt, QOSAL_BOOL sync)
{
  UNUSED_ARGUMENT(pCxt);
  UNUSED_ARGUMENT(sync);

  return A_OK;
}


/*****************************************************************************/
/* Custom_Bus_Complete_Transfer - This function is called by common layer prior
 *  to completing a bus transfer. This solution calls fflush to de-assert 
 *  the chipselect.
 *      QOSAL_VOID * pCxt - the driver context.     
 *      QOSAL_BOOL sync - TRUE is synchronous transfer is required else FALSE.
 *****************************************************************************/
A_STATUS 
Custom_Bus_CompleteTransfer(QOSAL_VOID *pCxt, QOSAL_BOOL sync)
{
  UNUSED_ARGUMENT(pCxt);
  UNUSED_ARGUMENT(sync);

  return A_OK;
}



/*****************************************************************************/
/* Custom_Bus_InOut_Token - This is the platform specific solution to 
 *  transfer 4 or less bytes in both directions. The transfer must be  
 *  synchronous. This solution uses the MQX spi ioctl to complete the request.
 *      QOSAL_VOID * pCxt - the driver context.
 *      QOSAL_UINT32 OutToken - the out going data.
 *      QOSAL_UINT8 DataSize - the length in bytes of the transfer.
 *      QOSAL_UINT32 *pInToken - A Buffer to hold the incoming bytes. 
 *****************************************************************************/
A_STATUS 
Custom_Bus_InOutToken(QOSAL_VOID *pCxt,
                           QOSAL_UINT32        OutToken,
                           QOSAL_UINT8         DataSize,
                           QOSAL_UINT32    *pInToken) 
{
  uint32_t ret;
  A_STATUS status = A_ERROR;

  if (DataSize > 4) {
    DataSize = 4;
  }

  if (osSemaphoreAcquire(SPI_semaphore, WIFI_QCA400x_SPI_TIMEOUT) == osOK) {
    if (SPIdrv->Transfer((uint8_t *)&OutToken, (uint8_t *)&pInToken[0],DataSize) == ARM_DRIVER_OK) {
      ret = osEventFlagsWait(SPI_completed_event, EVENT_SPI_COMPLETED, osFlagsWaitAll, WIFI_QCA400x_SPI_COMPLETED_TIMEOUT);
      if (((ret & 0x80000000) == 0) && ((ret & EVENT_SPI_COMPLETED) == EVENT_SPI_COMPLETED)) {
        status = A_OK;
      }
    }
    osSemaphoreRelease(SPI_semaphore);
  }

  return status;
}

A_STATUS 
Custom_HW_Init(QOSAL_VOID *pCxt)
{
  int32_t  ret;
  A_STATUS status = A_ERROR;
  A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);

  SPI_completed_event = osEventFlagsNew (NULL);

  if (SPI_completed_event != NULL) {
    SPI_semaphore = osSemaphoreNew (1, 0, NULL);
    if (SPI_semaphore != NULL) {
      pDCxt->spi_hcd.PowerUpDelay = 1;
      pDCxt->spi_hcd.SpiHWCapabilitiesFlags = (HW_SPI_FRAME_WIDTH_8 | HW_SPI_NO_DMA | HW_SPI_INT_EDGE_DETECT);
      pDCxt->spi_hcd.OperationalClock = 1000000;
    
      ret = SPIdrv->Initialize(SPI_Callback);
      if (ret == ARM_DRIVER_OK) {
        ret =  SPIdrv->PowerControl(ARM_POWER_FULL);
      }
      if (ret == ARM_DRIVER_OK) {
        ret = SPIdrv->Control (ARM_SPI_MODE_MASTER |
                               ARM_SPI_CPOL1_CPHA1 |
                               ARM_SPI_DATA_BITS(8)|
                               ARM_SPI_MSB_LSB     |
                               ARM_SPI_SS_MASTER_HW_OUTPUT,
                               1000000);
      }
     if (ret == ARM_DRIVER_OK) {
         status = A_OK;
      }
      osSemaphoreRelease(SPI_semaphore);
    }
  }
  return status;
}

A_STATUS 
Custom_HW_DeInit(QOSAL_VOID *pCxt)
{
  int32_t  ret;
  A_STATUS status = A_ERROR;

  if (osSemaphoreAcquire(SPI_semaphore, WIFI_QCA400x_SPI_TIMEOUT) == osOK) {
    ret = SPIdrv->Control(ARM_SPI_MODE_INACTIVE, 0);
    if (ret == ARM_DRIVER_OK) {
      ret =  SPIdrv->PowerControl(ARM_POWER_OFF);
    }
    if (ret == ARM_DRIVER_OK) {
      ret =  SPIdrv->Uninitialize();
    }
    osSemaphoreRelease(SPI_semaphore);
    if (ret == ARM_DRIVER_OK) {
      status = A_OK;
      osSemaphoreDelete(SPI_semaphore);
      osEventFlagsDelete(SPI_completed_event);
    }
  }

  return status;
}

QOSAL_VOID 
Custom_HW_PowerUpDown(QOSAL_VOID *pCxt, QOSAL_UINT32 powerUp) {
  WiFi_QCA400x_Pin_PWDN (powerUp);
}

#if 0   //TBD
QOSAL_VOID Custom_HW_RegisterInterruptHandler(CUSTOM_HW_INTR_CB *callback, QOSAL_VOID *pContext);
QOSAL_VOID Custom_HW_RegisterSuspendHandler(CUSTOM_HW_SUSPEND_CB *suspendCallback, QOSAL_VOID *pSuspContext);
QOSAL_VOID Custom_HW_RegisterResumeHandler(CUSTOM_HW_RESUME_CB *resumeCallback, QOSAL_VOID *pSuspContext);

QOSAL_VOID Custom_HW_RegisterInterruptHandler(CUSTOM_HW_INTR_CB *callback, QOSAL_VOID *pContext)
{
  //TBD
}

QOSAL_VOID Custom_HW_RegisterSuspendHandler(CUSTOM_HW_SUSPEND_CB *suspendCallback, QOSAL_VOID *pSuspContext) 
{
  //TBD
}

QOSAL_VOID Custom_HW_RegisterResumeHandler(CUSTOM_HW_RESUME_CB *resumeCallback, QOSAL_VOID *pSuspContext)
{
  //TBD
}

#endif  //#if 0 TBD
