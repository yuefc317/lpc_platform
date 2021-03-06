/***********************************************************************
 * $Id::$
 *
 * Project: Embedded Artists LPC3250 OEM Board definitions
 *
 * Description:
 *     This file contains board specific information such as the
 *     chip select wait states, and other board specific information.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/

#ifndef EA3250_PRV_H
#define EA3250_PRV_H


// Enable the following define to setup for RMII mode
#define USE_PHY_RMII

// PHY address (configured via PHY ADRx pins)
#define PHYDEF_PHYADDR           0x0001

// DP83848 Ethernet Phy Extended Register
#define DP83848_PHY_STATUS	0x10

// Maximum ethernet frame size, maximum RX and TX packets
#define ENET_MAXF_SIZE             1536
#define ENET_MAX_TX_PACKETS        16
#define ENET_MAX_RX_PACKETS        16



#endif /* EA3250_PRV_H */
