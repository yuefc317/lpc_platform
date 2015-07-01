/*===========================================================================
FILE:
   QMI.c

DESCRIPTION:
   Qualcomm QMI driver code

FUNCTIONS:
   Generic QMUX functions
      ParseQMUX
      FillQMUX

   Generic QMI functions
      GetTLV
      ValidQMIMessage
      GetQMIMessageID

   Fill Buffers with QMI requests
      QMICTLGetClientIDReq
      QMICTLReleaseClientIDReq
      QMICTLReadyReq
      QMIWDSSetEventReportReq
      QMIWDSGetPKGSRVCStatusReq
      QMIDMSGetMEIDReq

   Parse data from QMI responses
      QMICTLGetClientIDResp
      QMICTLReleaseClientIDResp
      QMIWDSEventResp
      QMIDMSGetMEIDResp

Copyright (c) 2011, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Code Aurora Forum nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
===========================================================================*/

//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------
#include <linux/string.h>
#include "QMI.h"

#ifdef SIMCOM_NETWORK_MANAGER
#include "GobiNetworkManager.h"
#endif /* SIMCOM_NETWORK_MANAGER */
/*=========================================================================*/
// Get sizes of buffers needed by QMI requests
/*=========================================================================*/

/*===========================================================================
METHOD:
   QMUXHeaderSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMUXHeaderSize( void )
{
   return sizeof( sQMUX );
}

/*===========================================================================
METHOD:
   QMICTLGetClientIDReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLGetClientIDReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMICTLGetClientIDReqSize( void )
{
   return sizeof( sQMUX ) + 10;
}

/*===========================================================================
METHOD:
   QMICTLReleaseClientIDReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLReleaseClientIDReq

RETURN VALUE:
   u16 - size of header
===========================================================================*/
u16 QMICTLReleaseClientIDReqSize( void )
{
   return sizeof( sQMUX ) + 11;
}

/*===========================================================================
METHOD:
   QMICTLReadyReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLReadyReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMICTLReadyReqSize( void )
{
   return sizeof( sQMUX ) + 6;
}

/*===========================================================================
METHOD:
   QMIWDSSetEventReportReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIWDSSetEventReportReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIWDSSetEventReportReqSize( void )
{
   return sizeof( sQMUX ) + 15;
}

/*===========================================================================
METHOD:
   QMIWDSGetPKGSRVCStatusReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIWDSGetPKGSRVCStatusReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIWDSGetPKGSRVCStatusReqSize( void )
{
   return sizeof( sQMUX ) + 7;
}

#ifdef SIMCOM_NETWORK_MANAGER

/*===========================================================================
METHOD:
   QMIWDSStartNetworkInterfaceReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIWDSStartNetworkInterfaceReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIWDSStartNetworkInterfaceReqSize( struct sGobiNMParam *pGobiNMParam)
  {
    int result = sizeof( sQMUX ) + 7;

    if (pGobiNMParam == 0)
      {
        return result;
      }

    if (pGobiNMParam->apn)
      {
        result += 3 + strlen(pGobiNMParam->apn);
      }

    if (pGobiNMParam->username)
      {
        result += 3 + strlen(pGobiNMParam->username);
      }

    if (pGobiNMParam->passwd)
      {
        result += 3 + strlen(pGobiNMParam->passwd);
      }

    if (pGobiNMParam->auth_type != AUTH_PROTOCOL_UNSPECIFIED)
      {
        result += 4;
      }

    if (pGobiNMParam->ip_type != IP_TYPE_UNSPECIFIED)
      {
        result += 4;
      }

    return result;
  }

/*===========================================================================
METHOD:
   QMIWDSStopNetworkInterfaceReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIWDSStopNetworkInterfaceReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIWDSStopNetworkInterfaceReqSize( void )
{
   return sizeof( sQMUX ) + 14;
}

#endif /* SIMCOM_NETWORK_MANAGER */

/*===========================================================================
METHOD:
   QMIDMSGetMEIDReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIDMSGetMEIDReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIDMSGetMEIDReqSize( void )
{
   return sizeof( sQMUX ) + 7;
}

/*=========================================================================*/
// Generic QMUX functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   ParseQMUX (Public Method)

DESCRIPTION:
   Remove QMUX headers from a buffer

PARAMETERS
   pClientID       [ O ] - On success, will point to Client ID
   pBuffer         [ I ] - Full Message passed in
   buffSize        [ I ] - Size of pBuffer

RETURN VALUE:
   int - Positive for size of QMUX header
         Negative errno for error
===========================================================================*/
int ParseQMUX(
   u16 *    pClientID,
   void *   pBuffer,
   u16      buffSize )
{
   sQMUX * pQMUXHeader;

   if (pBuffer == 0 || buffSize < 12)
   {
      return -ENOMEM;
   }

   // QMUX Header
   pQMUXHeader = (sQMUX *)pBuffer;

   if (pQMUXHeader->mTF != 1
   ||  pQMUXHeader->mLength != buffSize - 1
   ||  pQMUXHeader->mCtrlFlag != 0x80 )
   {
      return -EINVAL;
   }

   // Client ID
   *pClientID = (pQMUXHeader->mQMIClientID << 8)
              + pQMUXHeader->mQMIService;

   return sizeof( sQMUX );
}

/*===========================================================================
METHOD:
   FillQMUX (Public Method)

DESCRIPTION:
   Fill buffer with QMUX headers

PARAMETERS
   clientID        [ I ] - Client ID
   pBuffer         [ O ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer (must be at least 6)

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int FillQMUX(
   u16      clientID,
   void *   pBuffer,
   u16      buffSize )
{
   sQMUX * pQMUXHeader;

   if (pBuffer == 0 ||  buffSize < sizeof( sQMUX ))
   {
      return -ENOMEM;
   }

   // QMUX Header
   pQMUXHeader = (sQMUX *)pBuffer;

   pQMUXHeader->mTF = 1;
   pQMUXHeader->mLength = buffSize - 1;
   pQMUXHeader->mCtrlFlag = 0;

   // Service and Client ID   
   pQMUXHeader->mQMIService = clientID & 0xff;
   pQMUXHeader->mQMIClientID = clientID >> 8;

   return 0;
}

/*=========================================================================*/
// Generic QMI functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   GetTLV (Public Method)

DESCRIPTION:
   Get data bufffer of a specified TLV from a QMI message

   QMI Message shall NOT include SDU
   
PARAMETERS
   pQMIMessage    [ I ] - QMI Message buffer
   messageLen     [ I ] - Size of QMI Message buffer
   type           [ I ] - Desired Type
   pOutDataBuf    [ O ] - Buffer to be filled with TLV
   messageLen     [ I ] - Size of QMI Message buffer

RETURN VALUE:
   u16 - Size of TLV for success
         Negative errno for error
===========================================================================*/
u16 GetTLV(
   void *   pQMIMessage,
   u16      messageLen,
   u8       type,
   void *   pOutDataBuf,
   u16      bufferLen )
{
   u16 pos;
   u16 tlvSize = 0;
   u16 cpyCount;

   if (pQMIMessage == 0 || pOutDataBuf == 0)
   {
      return -ENOMEM;
   }

   for (pos = 4; 
        pos + 3 < messageLen; 
        pos += tlvSize + 3)
   {
      tlvSize = *(u16 *)(pQMIMessage + pos + 1);
      if (*(u8 *)(pQMIMessage + pos) == type)
      {
         if (bufferLen < tlvSize)
         {
            return -ENOMEM;
         }

         /* replacement memcpy
            memcpy( pOutDataBuf,
                    pQMIMessage + pos + 3,
                    tlvSize ); */

         for (cpyCount = 0; cpyCount < tlvSize; cpyCount++)
         {
            *((char*)(pOutDataBuf + cpyCount)) = *((char*)(pQMIMessage + pos + 3 + cpyCount));
         }

         return tlvSize;
      }
   }

   return -ENOMSG;
}

/*===========================================================================
METHOD:
   ValidQMIMessage (Public Method)

DESCRIPTION:
   Check mandatory TLV in a QMI message

   QMI Message shall NOT include SDU

PARAMETERS
   pQMIMessage    [ I ] - QMI Message buffer
   messageLen     [ I ] - Size of QMI Message buffer

RETURN VALUE:
   int - 0 for success (no error)
         Negative errno for error
         Positive for QMI error code
===========================================================================*/
int ValidQMIMessage(
   void *   pQMIMessage,
   u16      messageLen )
{
   char mandTLV[4];

   if (GetTLV( pQMIMessage, messageLen, 2, &mandTLV[0], 4 ) == 4)
   {
      // Found TLV
      if (*(u16 *)&mandTLV[0] != 0)
      {
         return *(u16 *)&mandTLV[2];
      }
      else
      {
         return 0;
      }
   }
   else
   {
      return -ENOMSG;
   }
}

/*===========================================================================
METHOD:
   GetQMIMessageID (Public Method)

DESCRIPTION:
   Get the message ID of a QMI message

   QMI Message shall NOT include SDU

PARAMETERS
   pQMIMessage    [ I ] - QMI Message buffer
   messageLen     [ I ] - Size of QMI Message buffer

RETURN VALUE:
   int - Positive for message ID
         Negative errno for error
===========================================================================*/
int GetQMIMessageID(
   void *   pQMIMessage,
   u16      messageLen )
{
   if (messageLen < 2)
   {
      return -ENODATA;
   }
   else
   {
      return *(u16 *)pQMIMessage;
   }
}

/*=========================================================================*/
// Fill Buffers with QMI requests
/*=========================================================================*/

/*===========================================================================
METHOD:
   QMICTLGetClientIDReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Get Client ID Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID
   serviceType     [ I ] - Service type requested

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLGetClientIDReq(
   void *   pBuffer,
   u16      buffSize,
   u8       transactionID,
   u8       serviceType )
{
   if (pBuffer == 0 || buffSize < QMICTLGetClientIDReqSize() )
   {
      return -ENOMEM;
   }

   // QMI CTL GET CLIENT ID
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))= 0x00;
   // Transaction ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 2) = 0x0022;
   // Size of TLV's
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 4) = 0x0004;
      // QMI Service Type
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 6)  = 0x01;
      // Size
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 7) = 0x0001;
      // QMI svc type
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 9)  = serviceType;

   // success
   return sizeof( sQMUX ) + 10;
}

/*===========================================================================
METHOD:
   QMICTLReleaseClientIDReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Release Client ID Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID
   clientID        [ I ] - Service type requested

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLReleaseClientIDReq(
   void *   pBuffer,
   u16      buffSize,
   u8       transactionID,
   u16      clientID )
{
   if (pBuffer == 0 || buffSize < QMICTLReleaseClientIDReqSize() )
   {
      return -ENOMEM;
   }

   // QMI CTL RELEASE CLIENT ID REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1 ) = transactionID;
   // Message ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 2) = 0x0023;
   // Size of TLV's
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 4) = 0x0005;
      // Release client ID
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 6)  = 0x01;
      // Size
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 7) = 0x0002;
      // QMI svs type / Client ID
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 9)  = clientID;

   // success
   return sizeof( sQMUX ) + 11;
}

/*===========================================================================
METHOD:
   QMICTLReadyReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Get Version Info Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLReadyReq(
   void *   pBuffer,
   u16      buffSize,
   u8       transactionID )
{
   if (pBuffer == 0 || buffSize < QMICTLReadyReqSize() )
   {
      return -ENOMEM;
   }

   // QMI CTL GET VERSION INFO REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 2) = 0x0021;
   // Size of TLV's
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 4) = 0x0000;

   // success
   return sizeof( sQMUX ) + 6;
}

/*===========================================================================
METHOD:
   QMIWDSSetEventReportReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI WDS Set Event Report Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIWDSSetEventReportReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID )
{
   if (pBuffer == 0 || buffSize < QMIWDSSetEventReportReqSize() )
   {
      return -ENOMEM;
   }

   // QMI WDS SET EVENT REPORT REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 3) = 0x0001;
   // Size of TLV's
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 5) = 0x0008;
      // Report channel rate TLV
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 7)  = 0x11;
      // Size
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 8) = 0x0005;
      // Stats period
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 10)  = 0x01;
      // Stats mask
      *(u32 *)(pBuffer + sizeof( sQMUX ) + 11)  = 0x000000ff;

   // success
   return sizeof( sQMUX ) + 15;
}

/*===========================================================================
METHOD:
   QMIWDSGetPKGSRVCStatusReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI WDS Get PKG SRVC Status Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIWDSGetPKGSRVCStatusReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID )
{
   if (pBuffer == 0 || buffSize < QMIWDSGetPKGSRVCStatusReqSize() )
   {
      return -ENOMEM;
   }

   // QMI WDS Get PKG SRVC Status REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 3) = 0x0022;
   // Size of TLV's
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 5) = 0x0000;

   // success
   return sizeof( sQMUX ) + 7;
}

#ifdef SIMCOM_NETWORK_MANAGER

/*===========================================================================
METHOD:
   QMIWDSStartNetworkInterfaceReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI WDS Start Network Interface Status Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIWDSStartNetworkInterfaceReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID,
   struct sGobiNMParam* pGobiNMParam)
{
  int paramSize = 0;

  if (pBuffer == 0 || buffSize < QMIWDSStartNetworkInterfaceReqSize(pGobiNMParam) )
  {
      return -SIMCOM_ERR_NO_MEMORY;
  }

  // Request
  *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
  // Transaction ID
  *(u16 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
  // Message ID
  *(u16 *)(pBuffer + sizeof( sQMUX ) + 3) = 0x0020;

  if (pGobiNMParam == 0)
  {
      // Size of TLV's
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 5) = 0x0000;
      return sizeof( sQMUX ) + 7 + paramSize;
  }

  if (pGobiNMParam->apn)
  {
      int string_len = strlen(pGobiNMParam->apn);
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 7) = 0x14;
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 8) = string_len;
      memcpy(pBuffer + sizeof( sQMUX ) + 10, pGobiNMParam->apn, string_len);
      paramSize += 3 + string_len;
  }

  if (pGobiNMParam->username)
  {
      int string_len = strlen(pGobiNMParam->username);
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 7 + paramSize) = 0x17;
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 8 + paramSize) = string_len;
      memcpy(pBuffer + sizeof( sQMUX ) + 10 + paramSize, pGobiNMParam->username, string_len);
      paramSize += 3 + string_len;
  }

  if (pGobiNMParam->passwd)
  {
      int string_len = strlen(pGobiNMParam->passwd);
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 7 + paramSize) = 0x18;
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 8 + paramSize) = string_len;
      memcpy(pBuffer + sizeof( sQMUX ) + 10 + paramSize, pGobiNMParam->passwd, string_len);
      paramSize += 3 + string_len;
  }

  if (pGobiNMParam->auth_type != AUTH_PROTOCOL_UNSPECIFIED)
  {
      u8 value = 0;
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 7 + paramSize) = 0x16;
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 8 + paramSize) = 0x0001;
      switch (pGobiNMParam->auth_type)
      {
        case AUTH_PROTOCOL_NONE:
          value = 0x00;
          break;
        case AUTH_PROTOCOL_PAP_ONLY:
          value = 0x01;
          break;
        case AUTH_PROTOCOL_CHAP_ONLY:
          value = 0x02;
          break;
        case AUTH_PROTOCOL_CHAP_PAP:
          value = 0x03;
          break;
        default:
          return -SIMCOM_ERR_INTERNEL;
        }
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 10 + paramSize) = value;
      paramSize += 4;
    }

  if (pGobiNMParam->ip_type != IP_TYPE_UNSPECIFIED)
  {
      u8 value = 0;
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 7 + paramSize) = 0x19;
      *(u16 *)(pBuffer + sizeof( sQMUX ) + 8 + paramSize) = 0x0001;
      switch (pGobiNMParam->ip_type)
        {
        case IP_TYPE_IPV4:
          value = 0x04;
          break;
        case IP_TYPE_IPV6:
          value = 0x06;
          break;
        case IP_TYPE_UNSPECIFIED:
          value = 0x08;
          break;
        default:
          return -SIMCOM_ERR_INTERNEL;
        }
      *(u8 *)(pBuffer + sizeof( sQMUX ) + 10 + paramSize) = value;
      paramSize += 4;
  }
  // success
  *(u16 *)(pBuffer + sizeof( sQMUX ) + 5) = paramSize;
  return sizeof( sQMUX ) + 7 + paramSize;
}


/*===========================================================================
METHOD:
   QMIWDSStartNetworkInterfaceResp (Public Method)

DESCRIPTION:
   Parse the QMI WDS Start Network Interface Resp

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer
   pSessionID           [ O ] - Packet Data Handle
                                The handle identifying the call instance
                                providing packet service.
                                The packet data handle must be retained
                                by the control point and specified in the
                                STOP_NETWORK_INTERFACE
                                message issued when the control point is
                                finished with the packet data session.
   sessionIDSize        [ I ] - Size of pSessionID buffer (at least 14)

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMIWDSStartNetworkInterfaceResp(
   void *   pBuffer,
   u16      buffSize,
   void *   pSessionID,
   int      sessionIDSize )
{
   int result;

   // Ignore QMUX and SDU
   u8 offset = sizeof( sQMUX ) + 3;

   if (pBuffer == 0 || buffSize < offset || sessionIDSize < 4)
   {
      return -SIMCOM_ERR_NO_MEMORY;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x20)
   {
      return -SIMCOM_ERR_NOT_MATCHED_RESP;
   }

   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
     if (result == -ENOMEM)
         return -SIMCOM_ERR_NO_MEMORY;
     else
     {
         return -QMIErrToSIMCOMErr(result);
     }
   }

   result = GetTLV( pBuffer, buffSize, 0x01, (void*)pSessionID, 4 );
   if (result != 4)
   {
      return -SIMCOM_ERR_INTERNEL;
   }

   return SIMCOM_SUCCESS;
}

/*===========================================================================
METHOD:
   QMIWDSStopNetworkInterfaceReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI WDS Start Network Interface Status Request

PARAMETERS
   pBuffer           [ 0 ] - Buffer to be filled
   buffSize          [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID
   sessionID        [ I ]  - Packet Data Handle
					The handle identifying the call instance
					providing packet service.
					The packet data handle must be retained
					by the control point and specified in the
					STOP_NETWORK_INTERFACE
					message issued when the control point is
					finished with the packet data session.

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIWDSStopNetworkInterfaceReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID,
   u32      sessionID)
{
   if (pBuffer == 0 || buffSize < QMIWDSStopNetworkInterfaceReqSize() )
   {
      return -SIMCOM_ERR_NO_MEMORY;
   }

   // QMI WDS Get PKG SRVC Status REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 3) = 0x0021;
   // Size of TLV's
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 5) = 0x0007;

   *(u8 *)(pBuffer + sizeof( sQMUX ) + 7) = 0x01;
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 8) = 0x0004;
   *(u32 *)(pBuffer + sizeof( sQMUX ) + 10) = sessionID;
   // success
   return sizeof( sQMUX ) + 14;
}

/*===========================================================================
METHOD:
   QMIWDSStopNetworkInterfaceResp (Public Method)

DESCRIPTION:
   Parse the QMI WDS Stop Network Interface Resp

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMIWDSStopNetworkInterfaceResp(
   void *   pBuffer,
   u16      buffSize)
{
   int result;

   // Ignore QMUX and SDU
   u8 offset = sizeof( sQMUX ) + 3;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -SIMCOM_ERR_NO_MEMORY;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x21)
   {
      return -SIMCOM_ERR_NOT_MATCHED_RESP;
   }

   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
       return -QMIErrToSIMCOMErr(result);
   }

   return SIMCOM_SUCCESS;
}

#endif  /* SIMCOM_NETWORK_MANAGER */

/*===========================================================================
METHOD:
   QMIDMSGetMEIDReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI DMS Get Serial Numbers Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIDMSGetMEIDReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID )
{
   if (pBuffer == 0 || buffSize < QMIDMSGetMEIDReqSize() )
   {
      return -ENOMEM;
   }

   // QMI DMS GET SERIAL NUMBERS REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 3) = 0x0025;
   // Size of TLV's
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 5) = 0x0000;

  // success
  return sizeof( sQMUX ) + 7;
}

/*=========================================================================*/
// Parse data from QMI responses
/*=========================================================================*/

/*===========================================================================
METHOD:
   QMICTLGetClientIDResp (Public Method)

DESCRIPTION:
   Parse the QMI CTL Get Client ID Resp

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer
   pClientID       [ 0 ] - Recieved client ID

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMICTLGetClientIDResp(
   void * pBuffer,
   u16    buffSize,
   u16 *  pClientID )
{
   int result;
   
   // Ignore QMUX and SDU
   //    QMI CTL SDU is 2 bytes, not 3
   u8 offset = sizeof( sQMUX ) + 2;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x22)
   {
      return -EFAULT;
   }

   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      return -EFAULT;
   }

   result = GetTLV( pBuffer, buffSize, 0x01, pClientID, 2 );
   if (result != 2)
   {
      return -EFAULT;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMICTLReleaseClientIDResp (Public Method)

DESCRIPTION:
   Verify the QMI CTL Release Client ID Resp is valid

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMICTLReleaseClientIDResp(
   void *   pBuffer,
   u16      buffSize )
{
   int result;

   // Ignore QMUX and SDU
   //    QMI CTL SDU is 2 bytes, not 3
   u8 offset = sizeof( sQMUX ) + 2;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x23)
   {
      return -EFAULT;
   }

   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      return -EFAULT;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMIWDSEventResp (Public Method)

DESCRIPTION:
   Parse the QMI WDS Set Event Report Resp/Indication or
      QMI WDS Get PKG SRVC Status Resp/Indication

   Return parameters will only be updated if value was received

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer
   pTXOk           [ O ] - Number of transmitted packets without errors
   pRXOk           [ O ] - Number of recieved packets without errors
   pTXErr          [ O ] - Number of transmitted packets with framing errors
   pRXErr          [ O ] - Number of recieved packets with framing errors
   pTXOfl          [ O ] - Number of transmitted packets dropped due to overflow
   pRXOfl          [ O ] - Number of recieved packets dropped due to overflow
   pTXBytesOk      [ O ] - Number of transmitted bytes without errors
   pRXBytesOk      [ O ] - Number of recieved bytes without errors
   pbLinkState     [ 0 ] - Is the link active?
   pbReconfigure   [ 0 ] - Must interface be reconfigured? (reset IP address)

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMIWDSEventResp(
   void *   pBuffer,
   u16      buffSize,
   u32 *    pTXOk,
   u32 *    pRXOk,
   u32 *    pTXErr,
   u32 *    pRXErr,
   u32 *    pTXOfl,
   u32 *    pRXOfl,
   u64 *    pTXBytesOk,
   u64 *    pRXBytesOk,
   bool *   pbLinkState,
   bool *   pbReconfigure )
{
   int result;
   u8 pktStatusRead[2];

   // Ignore QMUX and SDU
   u8 offset = sizeof( sQMUX ) + 3;

   if (pBuffer == 0
   || buffSize < offset
   || pTXOk == 0
   || pRXOk == 0
   || pTXErr == 0
   || pRXErr == 0
   || pTXOfl == 0
   || pRXOfl == 0
   || pTXBytesOk == 0
   || pRXBytesOk == 0
   || pbLinkState == 0
   || pbReconfigure == 0 )
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   // Note: Indications.  No Mandatory TLV required

   result = GetQMIMessageID( pBuffer, buffSize );
   // QMI WDS Set Event Report Resp
   if (result == 0x01)
   {
      // TLV's are not mandatory
      GetTLV( pBuffer, buffSize, 0x10, (void*)pTXOk, 4 );
      GetTLV( pBuffer, buffSize, 0x11, (void*)pRXOk, 4 );
      GetTLV( pBuffer, buffSize, 0x12, (void*)pTXErr, 4 );
      GetTLV( pBuffer, buffSize, 0x13, (void*)pRXErr, 4 );
      GetTLV( pBuffer, buffSize, 0x14, (void*)pTXOfl, 4 );
      GetTLV( pBuffer, buffSize, 0x15, (void*)pRXOfl, 4 );
      GetTLV( pBuffer, buffSize, 0x19, (void*)pTXBytesOk, 8 );
      GetTLV( pBuffer, buffSize, 0x1A, (void*)pRXBytesOk, 8 );
   }
   // QMI WDS Get PKG SRVC Status Resp
   else if (result == 0x22)
   {
      result = GetTLV( pBuffer, buffSize, 0x01, &pktStatusRead[0], 2 );
      // 1 or 2 bytes may be received
      if (result >= 1)
      {
         if (pktStatusRead[0] == 0x02)
         {
            *pbLinkState = true;
         }
         else
         {
            *pbLinkState = false;
         }
      }
      if (result == 2)
      {
         if (pktStatusRead[1] == 0x01)
         {
            *pbReconfigure = true;
         }
         else
         {
            *pbReconfigure = false;
         }
      }

      if (result < 0)
      {
         return result;
      }
   }
   else
   {
      return -EFAULT;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMIDMSGetMEIDResp (Public Method)

DESCRIPTION:
   Parse the QMI DMS Get Serial Numbers Resp

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer
   pMEID           [ O ] - Device MEID
   meidSize        [ I ] - Size of MEID buffer (at least 14)

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMIDMSGetMEIDResp(
   void *   pBuffer,
   u16      buffSize,
   char *   pMEID,
   int      meidSize )
{
   int result;

   // Ignore QMUX and SDU
   u8 offset = sizeof( sQMUX ) + 3;

   if (pBuffer == 0 || buffSize < offset || meidSize < 14)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x25)
   {
      return -EFAULT;
   }

   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      return -EFAULT;
   }

   result = GetTLV( pBuffer, buffSize, 0x12, (void*)pMEID, 14 );
   if (result != 14)
   {
      return -EFAULT;
   }

   return 0;
}

#ifdef SIMCOM_NETWORK_MANAGER
int QMIErrToSIMCOMErr(int QMIErr)
{
  switch(QMIErr)
    {
    case 1:
      return SIMCOM_ERR_MALFORMED_MSG;
    case 2:
      return SIMCOM_ERR_NO_MEMORY;
    case 3:
      return SIMCOM_ERR_INTERNEL;
    case 10:
      return SIMCOM_ERR_INVALID_PROFILE;
    case 14:
      return SIMCOM_ERR_CALL_FAILED;
    case 19:
      return SIMCOM_ERR_ARG_TOO_LONG;
    case 26:
      return SIMCOM_ERR_NO_EFFECT;
    case 28:
      return SIMCOM_ERR_INVALID_PDP_TYPE;
    case 29:
      return SIMCOM_ERR_INVALID_TECH_PREF;
    case 64:
      return SIMCOM_ERR_INVALID_IP_FAMILY_PREF;
    case 82:
      return SIMCOM_ERR_ACCESS_DENIED;
    default:
      return SIMCOM_ERR_INTERNEL;
    }
}
#endif /* SIMCOM_NETWORK_MANAGER */