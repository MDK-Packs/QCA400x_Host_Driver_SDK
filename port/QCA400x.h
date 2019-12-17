#ifndef __QCA400X_H__
#define __QCA400X_H__

#include "stdint.h"
#include "a_osapi.h"
#include "QCA400x_Config.h"

// Joined multicast groups
typedef struct _mcb_struct {
  uint8_t GROUP[6];
  uint8_t reserved[2];
  uint32_t HASH;
  struct _mcb_struct *NEXT;
} MCB_STRUCT, *MCB_STRUCT_PTR;

#if WIFI_QCA400x_MODE_PASSTHROUGH
// Frame received callback
typedef void (*FrameReceived)(A_NETBUF *a_netbuf_ptr);
#endif

// QCA400x WiFi
typedef struct _QCA400x_WiFi {
  A_CUSTOM_DRIVER_CONTEXT *MAC_CONTEXT_PTR;
  uint8_t                  ADDRESS[6];
  uint8_t                  reserved[2];
#if WIFI_QCA400x_MODE_PASSTHROUGH
  FrameReceived            FrameReceived_cb;
#endif
} QCA400x_WiFi, *QCA400x_WiFi_PTR;

typedef struct {
  int32_t (*INIT)     (QCA400x_WiFi *);
  int32_t (*STOP)     (QCA400x_WiFi *);
  int32_t (*SEND)     (QCA400x_WiFi *, void *,   uint32_t,   uint32_t,   uint32_t);
  int32_t (*PHY_READ) (QCA400x_WiFi *, uint32_t, uint32_t *, uint32_t);
  int32_t (*PHY_WRITE)(QCA400x_WiFi *, uint32_t, uint32_t,   uint32_t);
  int32_t (*JOIN)     (QCA400x_WiFi *, MCB_STRUCT *);
  int32_t (*REJOIN)   (QCA400x_WiFi *);
  int32_t (*MEDIACTL) (uint8_t, uint32_t, void *);
} MAC_IF, *MAC_IF_PTR;

#endif
