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

//#include <a_config.h>
//#include <a_types.h>
//#include <a_osapi.h>
//#include <driver_cxt.h>
//#include <common_api.h>
//#include <custom_wlan_api.h>
//#include <wmi_api.h>
//#include <bmi.h>
//#include <htc.h>
//#include <wmi_host.h>
//#if ENABLE_P2P_MODE
//#include <wmi.h>
//#include "p2p.h"
//#endif
//#include "mqx.h"
//#include "bsp.h"
//#include "enet.h"
//#include "enetprv.h"
//#include "atheros_wifi.h"
//#include "enet_wifi.h"
//#include "atheros_wifi_api.h"
//#include "atheros_wifi_internal.h"
//#include "atheros_stack_offload.h"
//#include "dset_api.h"
//#include "common_stack_offload.h"
//#include "qcom_api.h"

#include "QCA400x.h"
#include "atheros_wifi_api.h"
#include "atheros_stack_offload.h"
#include "driver_cxt.h"
#include "qcom_api.h"

//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
extern A_STATUS ath_ioctl_handler(void *pCxt, ATH_IOCTL_PARAM_STRUCT_PTR param_ptr);
extern void* Custom_Api_GetDriverCxt(uint8_t device_id);


/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : custom_qapi
* Returned Value : A_ERROR on error else A_OK
* Comments       : Custom part of QAPI layer. This function will implement the 
*                  custom portion of different QAPIs (OS specific). 
*                  PORTING NOTE: Engineer should rewrite this function based on 
*                  the OS framework.
*END------------------------------------------------------------------*/
A_STATUS custom_qapi(uint8_t device_id, uint32_t cmd, void* param) {

  A_STATUS                  error = A_ERROR;
  QCA400x_WiFi            * pDev  = NULL;
  A_CUSTOM_DRIVER_CONTEXT * pCxt  = NULL;
  A_DRIVER_CONTEXT        * pDCxt = NULL;

  WMI_SET_PASSPHRASE_CMD passCmd;

  pDev = (QCA400x_WiFi *)(Custom_Api_GetDriverCxt(device_id));
  if (pDev == NULL) {
    return A_ERROR;
  }

  pCxt = pDev->MAC_CONTEXT_PTR;
  if (pCxt == NULL) {
    return A_ERROR;
  }

  pDCxt = GET_DRIVER_COMMON((A_CUSTOM_DRIVER_CONTEXT *)(pDev->MAC_CONTEXT_PTR));
  if (pDCxt == NULL) {
    return A_ERROR;
  }

  switch(cmd){
//  case ATH_CHIP_STATE:
//    error = chip_state_ctrl(wifiDev, *(((ATH_IOCTL_PARAM_STRUCT*)param)->data));
//    if(error != ENET_OK){
//      config_dump_target_assert_info(enet_ptr->MAC_CONTEXT_PTR);
//    }
//    break;

  case ATH_SET_SCAN:
    pDCxt->extended_scan = 0;
    error = (A_STATUS)(scan_setup(pCxt, pDCxt->pWmiCxt, (WMI_START_SCAN_CMD *)NULL));
    break;

  case ATH_GET_SCAN_RESULTS:
    wait_scan_done(pCxt, pDCxt->pWmiCxt);
    ((ATH_SCAN_LIST*)param)->scan_list = pDCxt->pScanOut;
    ((ATH_SCAN_LIST*)param)->num_scan_entries = pDCxt->scanOutCount;
    error = A_OK;
    break;

  case ATH_SET_SSID:
    if (((ATH_IOCTL_PARAM_STRUCT *)param)->length < 33) {
      pDCxt->conn[device_id].ssidLen = strlen((char *)(((ATH_IOCTL_PARAM_STRUCT *)param)->data));
      if (pDCxt->conn[device_id].ssidLen) {
        A_MEMCPY(pDCxt->conn[device_id].ssid, ((ATH_IOCTL_PARAM_STRUCT *)param)->data, (uint32_t)pDCxt->conn[device_id].ssidLen);
      }
    } else {
      error = A_EINVAL;
    }
    error = A_OK;
    break;

  case ATH_GET_ESSID:
    ((ATH_IOCTL_PARAM_STRUCT*)param)->length = pDCxt->conn[device_id].ssidLen;
    A_MEMCPY(((ATH_IOCTL_PARAM_STRUCT*)param)->data, pDCxt->conn[device_id].ssid,(uint32_t)pDCxt->conn[device_id].ssidLen + 1);
    error = A_OK;
    break;

  case ATH_SET_COMMIT:
    if (pDCxt->conn[device_id].ssidLen == 0){
      /* a zero length ssid + a COMMIT command is interpreted as a
       * request from the caller to disconnect.
       */
      pDCxt->securityType = WLAN_AUTH_NONE;
      error = Api_DisconnectWiFi(pCxt);
    } else {
      error = Api_ConnectWiFi(pCxt);
    }
    pDCxt->wps_in_progress = A_FALSE;
    break;

  case ATH_GET_POWER:
    if(pDCxt->userPwrMode == REC_POWER){
      *((uint32_t *)param) = 1;
    }else{
      *((uint32_t *)param) = 0;
    }
    error = A_OK;
    break;

  case ATH_SET_SEC_TYPE:
    pDCxt->securityType = *((uint8_t *)param);

    switch(*(QOSAL_UINT32*)param) {
    case WLAN_AUTH_NONE:
      pDCxt->conn[device_id].wpaAuthMode = NONE_AUTH;
      pDCxt->conn[device_id].wpaPairwiseCrypto = NONE_CRYPT;
      pDCxt->conn[device_id].wpaGroupCrypto = NONE_CRYPT;
      pDCxt->conn[device_id].dot11AuthMode = OPEN_AUTH;
      pDCxt->conn[device_id].connectCtrlFlags &= ~CONNECT_DO_WPA_OFFLOAD;
      error = A_OK;
      break;
    case WLAN_AUTH_WPA_PSK:
      /* FIXME: need t allow WPA with AES */
      pDCxt->conn[device_id].wpaAuthMode = WPA_PSK_AUTH;
      pDCxt->conn[device_id].dot11AuthMode = OPEN_AUTH;
      /* by ignoring the group cipher the wifi can connect to mixed mode AP's */
      pDCxt->conn[device_id].connectCtrlFlags |= CONNECT_DO_WPA_OFFLOAD | CONNECT_IGNORE_WPAx_GROUP_CIPHER;
      error = A_OK;
      break;
    case WLAN_AUTH_WPA2_PSK:
      /* FIXME: need to allow WPA2 with TKIP */
      pDCxt->conn[device_id].wpaAuthMode = WPA2_PSK_AUTH;
      pDCxt->conn[device_id].dot11AuthMode = OPEN_AUTH;
      /* by ignoring the group cipher the wifi can connect to mixed moe AP's */
      pDCxt->conn[device_id].connectCtrlFlags |= CONNECT_DO_WPA_OFFLOAD | CONNECT_IGNORE_WPAx_GROUP_CIPHER;
      error = A_OK;
      break;
     case WLAN_AUTH_WEP:
       pDCxt->conn[device_id].wpaAuthMode = NONE_AUTH;
       pDCxt->conn[device_id].wpaPairwiseCrypto = WEP_CRYPT;
       pDCxt->conn[device_id].wpaGroupCrypto = WEP_CRYPT;
       /* allow either form of auth for WEP */
       pDCxt->conn[device_id].dot11AuthMode = (OPEN_AUTH | SHARED_AUTH); 
       pDCxt->conn[device_id].connectCtrlFlags &= ~CONNECT_DO_WPA_OFFLOAD;
       error = A_OK;
       break;
    default:
     error = A_ERROR;
    }
    break;
  case ATH_SET_OP_MODE:
    error = A_OK;
    switch (*(uint32_t *)param) {
      case QCOM_WLAN_DEV_MODE_STATION:
        pDCxt->conn[device_id].networkType = INFRA_NETWORK;
        break;
      case QCOM_WLAN_DEV_MODE_ADHOC:
        pDCxt->conn[device_id].networkType = ADHOC_NETWORK;
        break;
#if ENABLE_AP_MODE
      case QCOM_WLAN_DEV_MODE_AP:
        pDCxt->conn[device_id].networkType = AP_NETWORK;
        break;
#endif
      default:
        error = A_ERROR;
        break;
    }
    break;

  case ATH_GET_OP_MODE:
    error = A_OK;
    switch(pDCxt->conn[device_id].networkType) {
      case INFRA_NETWORK:
        *(uint32_t*)param = QCOM_WLAN_DEV_MODE_STATION;
        break;
      case ADHOC_NETWORK:
        *(uint32_t*)param = QCOM_WLAN_DEV_MODE_ADHOC;
        break;
#if ENABLE_AP_MODE
      case AP_NETWORK:
        *(uint32_t*)param = QCOM_WLAN_DEV_MODE_AP;
        break;
#endif
      default:
        error = A_ERROR;
        break;
      }
    break;

  case ATH_SET_PASSPHRASE:
    /* must have already set SSID prior to this call */
    if(((ATH_IOCTL_PARAM_STRUCT*)param)->length > WMI_PASSPHRASE_LEN) {
      error = A_EINVAL;
    } else if (pDCxt->conn[device_id].ssidLen == 0) {
      error = A_ERROR;
    } else {
      passCmd.ssid_len = (uint8_t)pDCxt->conn[device_id].ssidLen;
      A_MEMCPY(passCmd.ssid, pDCxt->conn[device_id].ssid, (QOSAL_UINT32)pDCxt->conn[device_id].ssidLen);
      passCmd.passphrase_len = (uint8_t)(((ATH_IOCTL_PARAM_STRUCT *)param)->length);
      A_MEMCPY(passCmd.passphrase, ((ATH_IOCTL_PARAM_STRUCT *)param)->data, passCmd.passphrase_len);
      error = wmi_cmd_start(pDCxt->pWmiCxt, &passCmd, WMI_SET_PASSPHRASE_CMDID, sizeof(WMI_SET_PASSPHRASE_CMD));
    }
    break;

  default: 
    error = ath_ioctl_handler(pCxt, ((ATH_IOCTL_PARAM_STRUCT *)param));
  }
  return error;
}
