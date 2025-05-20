/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// Enable device stack
#define CFG_TUD_ENABLED         1

// Default is max speed that hardware controller could support with on-chip PHY
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT        0
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

// Enable 2 CDC ports
#define CFG_TUD_CDC             2

// CDC FIFO size of TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE  512
#define CFG_TUD_CDC_TX_BUFSIZE  512

// CDC Endpoint transfer buffer size
// Note: For USB Full Speed, endpoint transfer size should be 64 bytes
#define CFG_TUD_CDC_EP_BUFSIZE  64

// Default control endpoint size
#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE  64
#endif

// Enable string descriptor
#define CFG_TUD_DESC_STR        1

// Enable auto-generated USB descriptors
#define CFG_TUD_DESC_AUTO       1

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
