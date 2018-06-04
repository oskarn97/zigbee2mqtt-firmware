/**************************************************************************************************
  Filename:       zcl_sampledoorlockcontroller.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $

  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This device is a door lock controller.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Send toggle door lock PIN code
    - SW2: Invoke EZMode
    - SW3: Go to PIN screen
    - SW4: Enable/Disable Permit Join
    - SW5: Go to Help screen

  PIN:
    - SW1: Increase PIN number
    - SW3: Decrease PIN number
    - SW5: Enter PIN number
  ----------------------------------------
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "MT.h"
#include "MT_APP.h"
#include "OSAL_Nv.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_closures.h"

#include "zcl_sampledoorlockcontroller.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"


/*********************************************************************
 * MACROS
 */

// There is no attribute in the Mandatory Reportable Attribute list for now
#define zcl_MandatoryReportableAttribute( a ) ( a == NULL )

// BACnet Device ID of 0x3FFFFF indicates that the associated BACnet device
// is not commissioned.
#define PI_BACNET_COMMISSIONED( addr )        ( ( addr[0] == 3 )  && \
                                                ( addr[1] != 0xff || \
                                                  addr[2] != 0xff || \
                                                  addr[3] != 0x3f ) )

// 11073 System ID of zero indicates that the associated 11073 device
// is not commissioned.
#define PI_11073_COMMISSIONED( addr )         ( ( addr[0] == Z_EXTADDR_LEN ) && \
                                                ( addr[1] != 0x00 || \
                                                  addr[2] != 0x00 || \
                                                  addr[3] != 0x00 || \
                                                  addr[4] != 0x00 || \
                                                  addr[5] != 0x00 || \
                                                  addr[6] != 0x00 || \
                                                  addr[7] != 0x00 || \
                                                  addr[8] != 0x00 ) )

/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSampleDoorLockController_TaskID;

uint8 zclSampleDoorLockControllerSeqNum;

uint8  zclSampleDoorLockController_OnOff;

static byte gPermitDuration = 0;  // disable by default

uint8 zclSampleDoorLockController_LockState;

uint8 zclSampleDoorLockController_LockType;

bool zclSampleDoorLockController_ActuatorEnabled;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleDoorLockController_DstAddr;

uint8 iToggleBuff = LIGHT_OFF;

#ifdef ZCL_EZMODE
static void zclSampleDoorLockController_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleDoorLockController_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclSampleDoorLockController_RegisterEZModeData =
{
  &zclSampleDoorLockController_TaskID,
  SAMPLEDOORLOCKCONTROLLER_EZMODE_NEXTSTATE_EVT,
  SAMPLEDOORLOCKCONTROLLER_EZMODE_TIMEOUT_EVT,
  &zclSampleDoorLockControllerSeqNum,
  zclSampleDoorLockController_EZModeCB
};


// NOT ZCL_EZMODE, Use EndDeviceBind
#else
#define ZCLSAMPLEDOORLOCKCONTROLLER_BINDINGLIST   2
static cId_t bindingOutClusters[ZCLSAMPLEDOORLOCKCONTROLLER_BINDINGLIST] =
{
  ZCL_CLUSTER_ID_GEN_TIME,
  ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK
};
#endif

devStates_t zclSampleDoorLockController_NwkState = DEV_INIT;

static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

/*
  Screen Display Options:
  0 = main screen
  1 = PIN screen
  2 = help screen
*/
uint8 giDoorLockScreenMode = DOORLOCK_MAINMODE;   // display main screen mode first

uint8 giDoorLockPINCount = 0;

uint8 giDoorLockPINColumnCount = 0;

uint8 aiDoorLockPIN[] = {4, 0x31, 0x32, 0x33, 0x34};   // default PIN (size 4) of 1,2,3,4 (ASCII)

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleDoorLockController_TestEp =
{
  SAMPLEDOORLOCKCONTROLLER_ENDPOINT,                                 // Test endpoint
  &zclSampleDoorLockController_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleDoorLockController_HandleKeys( byte shift, byte keys );
static void zclSampleDoorLockController_BasicResetCB( void );
static void zclSampleDoorLockController_IdentifyCB( zclIdentify_t *pCmd );
static void zclSampleDoorLockController_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
//static void zclSampleDoorLockController_OnOffCB( uint8 cmd );
static void zclSampleDoorLockController_ProcessIdentifyTimeChange( void );

void zclSampleDoorLockController_LcdDisplayMainMode(void);
void zclSampleDoorLockController_LcdDisplayHelpMode(void);
void zclSampleDoorLockController_LcdDisplayPINMode(void);
void zclSampleDoorLockController_LcdDisplayUpdate(void);

static void zclSampleDoorLockController_ProcessAppMsg( uint8 srcEP, uint8 len, uint8 *msg );
static void zclSampleDoorLockController_ProcessFoundationMsg( afAddrType_t *dstAddr, uint16 clusterID,
                                                              zclFrameHdr_t *hdr, zclParseCmd_t *pParseCmd );
static void zclSampleDoorLockController_ProcessGeneralMsg( uint8 srcEP, afAddrType_t *dstAddr,
                                                           uint16 clusterID, zclFrameHdr_t *hdr, uint8 len, uint8 *data );
static void zclSampleDoorLockController_ProcessGroupCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                         uint16 clusterID, zclFrameHdr_t *hdr, uint8 len, uint8 *data );
static void zclSampleDoorLockController_ProcessSceneCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                         uint16 clusterID, zclFrameHdr_t *hdr, uint8 len, uint8 *data );
static void zclSampleDoorLockController_ProcessClosuresMsg( uint8 srcEP, afAddrType_t *dstAddr,
                                                            uint16 clusterID, zclFrameHdr_t *hdr, uint8 len, uint8 *data );

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleDoorLockController_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleDoorLockController_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleDoorLockController_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleDoorLockController_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );

#ifdef ZCL_DISCOVER
static uint8 zclSampleDoorLockController_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
#endif

static ZStatus_t zclSampleDoorLockController_DoorLockRspCB ( zclIncoming_t *pInMsg, uint8 status );

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]    = " ";
const char sDeviceName[]   = "DoorLock Remote";
const char sSwLock[]       = "SW1: Lock/Unlock";
const char sSwEZMode[]     = "SW2: EZ-Mode";
const char sSwPIN[]        = "SW3: Enter PIN";
const char sPINLine2[]     = "SW1:+";
const char sPINLine3[]     = "SW3:-  SW5:Enter";
const char sSwHelp[]       = "SW5: Help";
const char sStorePIN[]     = "   PIN SAVED";
const char sInvalidPIN[]   = "  Invalid PIN";
const char sFail[]         = "Lock/Unlock Fail";
const char sDoorLocked[]   = "  Door Locked";
const char sDoorUnlocked[] = "  Door Unlocked";
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleDoorLockController_CmdCallbacks =
{
  zclSampleDoorLockController_BasicResetCB,           // Basic Cluster Reset command
  zclSampleDoorLockController_IdentifyCB,             // Identify command
#ifdef ZCL_EZMODE
  NULL,                                               // Identify EZ-Mode Invoke command
  NULL,                                               // Identify Update Commission State command
#endif
  NULL,                                               // Identify Trigger Effect command
  zclSampleDoorLockController_IdentifyQueryRspCB,     // Identify Query Response command
  NULL,                                               // On/Off cluster commands
  NULL,                                               // On/Off cluster enhanced command Off with Effect
  NULL,                                               // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                               // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                               // Level Control Move to Level command
  NULL,                                               // Level Control Move command
  NULL,                                               // Level Control Step command
  NULL,                                               // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                               // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                               // Scene Store Request command
  NULL,                                               // Scene Recall Request command
  NULL,                                               // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                               // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                               // Get Event Log command
  NULL,                                               // Publish Event Log command
#endif
  NULL,                                               // RSSI Location command
  NULL                                                // RSSI Location Response command
};

/*********************************************************************
 * ZCL Closure cluster Callback table
 */
static zclClosures_DoorLockAppCallbacks_t zclSampleDoorLockController_DoorLockCmdCallbacks =
{
  NULL,                                                  // DoorLock cluster command
  zclSampleDoorLockController_DoorLockRspCB,             // DoorLock Response
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};

/*********************************************************************
 * @fn          zclSampleDoorLockController_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleDoorLockController_Init( byte task_id )
{
  zclSampleDoorLockController_TaskID = task_id;

  // Set destination address to indirect
  zclSampleDoorLockController_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleDoorLockController_DstAddr.endPoint = 0;
  zclSampleDoorLockController_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleDoorLockController_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, &zclSampleDoorLockController_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, SAMPLEDOORLOCKCONTROLLER_MAX_ATTRIBUTES, zclSampleDoorLockController_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleDoorLockController_TaskID );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleDoorLockController_TaskID );

  //Register the ZCL DoorLock Cluster Library callback function
  zclClosures_RegisterDoorLockCmdCallbacks( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, &zclSampleDoorLockController_DoorLockCmdCallbacks );

  // Register for a test endpoint
  afRegister( &sampleDoorLockController_TestEp );

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleDoorLockController_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif

  ZDO_RegisterForZDOMsg( zclSampleDoorLockController_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( zclSampleDoorLockController_TaskID, Match_Desc_rsp );

  // initialize NVM for storing PIN information
  if ( SUCCESS == osal_nv_item_init( ZCD_NV_APS_DOORLOCK_PIN, 5, aiDoorLockPIN ) )
    // use NVM PIN number in APP
    osal_nv_read( ZCD_NV_APS_DOORLOCK_PIN, 0, 5, aiDoorLockPIN );

#ifdef LCD_SUPPORTED
  // display the device name
  HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclSampleDoorLockController_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleDoorLockController_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclSampleDoorLockController_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif

        case MT_SYS_APP_MSG:
          // Message received from MT
          zclSampleDoorLockController_ProcessAppMsg( ((mtSysAppMsg_t *)MSGpkt)->endpoint,
                                                     ((mtSysAppMsg_t *)MSGpkt)->appDataLen,
                                                     ((mtSysAppMsg_t *)MSGpkt)->appData );
          break;

        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleDoorLockController_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSampleDoorLockController_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleDoorLockController_NwkState = (devStates_t)(MSGpkt->hdr.status);

          // now on the network
          if ( (zclSampleDoorLockController_NwkState == DEV_ZB_COORD) ||
               (zclSampleDoorLockController_NwkState == DEV_ROUTER)   ||
               (zclSampleDoorLockController_NwkState == DEV_END_DEVICE) )
          {
#ifndef HOLD_AUTO_START
            giDoorLockScreenMode = DOORLOCK_MAINMODE;
            zclSampleDoorLockController_LcdDisplayUpdate();
#endif
#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SAMPLEDOORLOCKCONTROLLER_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSampleDoorLockController_IdentifyTime > 0 )
      zclSampleDoorLockController_IdentifyTime--;
    zclSampleDoorLockController_ProcessIdentifyTimeChange();

    return ( events ^ SAMPLEDOORLOCKCONTROLLER_IDENTIFY_TIMEOUT_EVT );
  }

  if ( events & SAMPLEDOORLOCKCONTROLLER_MAIN_SCREEN_EVT )
  {
    giDoorLockScreenMode = DOORLOCK_MAINMODE;
    zclSampleDoorLockController_LcdDisplayUpdate();

    return ( events ^ SAMPLEDOORLOCKCONTROLLER_MAIN_SCREEN_EVT );
  }

#ifdef ZCL_EZMODE
  // going on to next state
  if ( events & SAMPLEDOORLOCKCONTROLLER_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SAMPLEDOORLOCKCONTROLLER_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & SAMPLEDOORLOCKCONTROLLER_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SAMPLEDOORLOCKCONTROLLER_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclSampleDoorLockController_HandleKeys( byte shift, byte keys )
{
  uint8 numBuff;   // used to convert decimal to ASCII

  if ( keys & HAL_KEY_SW_1 )
  {
    // increase PIN number
    if ( giDoorLockScreenMode == DOORLOCK_PINMODE )
    {
      if ( giDoorLockPINColumnCount < 4 )
      {
        if ( giDoorLockPINCount > 8 )
        {
          giDoorLockPINCount = 0;
        }
        else
        {
          giDoorLockPINCount++;
        }
      }
    }
    // for all other modes, lock/unlock door
    else
    {
      zclDoorLock_t cmd;

      cmd.pPinRfidCode = aiDoorLockPIN;

      static bool locked = TRUE;

      giDoorLockScreenMode = DOORLOCK_MAINMODE;

      if ( locked )
      {
        zclClosures_SendDoorLockUnlockDoor( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, &zclSampleDoorLockController_DstAddr, &cmd, TRUE, zclSampleDoorLockControllerSeqNum++ );
        locked = FALSE;
      }
      else
      {
        zclClosures_SendDoorLockLockDoor( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, &zclSampleDoorLockController_DstAddr, &cmd, TRUE, zclSampleDoorLockControllerSeqNum++ );
        locked = TRUE;
      }
    }
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    if ( ( giDoorLockScreenMode == DOORLOCK_MAINMODE ) ||
         ( giDoorLockScreenMode == DOORLOCK_HELPMODE ) )
    {
      giDoorLockScreenMode = DOORLOCK_MAINMODE;
#ifdef ZCL_EZMODE
      {
        zclEZMode_InvokeData_t ezModeData;
        static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK };   // only bind on the on/off cluster

        // Invoke EZ-Mode
        ezModeData.endpoint = SAMPLEDOORLOCKCONTROLLER_ENDPOINT; // endpoint on which to invoke EZ-Mode
        if ( ( zclSampleDoorLockController_NwkState == DEV_ZB_COORD ) ||
             ( zclSampleDoorLockController_NwkState == DEV_ROUTER )   ||
             ( zclSampleDoorLockController_NwkState == DEV_END_DEVICE ) )
        {
          ezModeData.onNetwork = TRUE;      // node is already on the network
        }
        else
        {
          ezModeData.onNetwork = FALSE;     // node is not yet on the network
        }

        ezModeData.initiator = TRUE;            // DoorLock Controller is an initiator
        ezModeData.numActiveInClusters = 0;
        ezModeData.pActiveInClusterIDs = NULL;
        ezModeData.numActiveOutClusters = 1;    // active output cluster
        ezModeData.pActiveOutClusterIDs = clusterIDs;
        zcl_InvokeEZMode( &ezModeData );
#ifdef LCD_SUPPORTED
        HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif
      }

// NOT ZCL_EZMODE, Use EndDeviceBind
#else
      {
        zAddrType_t dstAddr;
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

        // Initiate an End Device Bind Request, this bind request will
        // only use a cluster list that is important to binding.
        dstAddr.addrMode = afAddr16Bit;
        dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
        ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                              SAMPLEDOORLOCKCONTROLLER_ENDPOINT,
                              ZCL_HA_PROFILE_ID,
                              0, NULL,   // No incoming clusters to bind
                              ZCLSAMPLEDOORLOCKCONTROLLER_BINDINGLIST, bindingOutClusters,
                              TRUE );
      }
#endif // ZCL_EZMODE
    }
  }

  if ( keys & HAL_KEY_SW_3 )
  {
    // decrease PIN number
    if ( giDoorLockScreenMode == DOORLOCK_PINMODE )
    {
      if ( giDoorLockPINColumnCount < 4 )
      {
        if ( giDoorLockPINCount < 1 )
        {
          giDoorLockPINCount = 9;
        }
        else
        {
          giDoorLockPINCount--;
        }
      }
    }
    else
    {
      giDoorLockPINColumnCount = 0;
      giDoorLockScreenMode = DOORLOCK_PINMODE;   // set to PIN screen
    }
  }

  if ( keys & HAL_KEY_SW_4 )
  {
    if ( ( zclSampleDoorLockController_NwkState == DEV_ZB_COORD ) ||
        ( zclSampleDoorLockController_NwkState == DEV_ROUTER ) )
    {
      zAddrType_t tmpAddr;

      tmpAddr.addrMode = Addr16Bit;
      tmpAddr.addr.shortAddr = NLME_GetShortAddr();

      // toggle permit join
      gPermitDuration = gPermitDuration ? 0 : 0xff;

      // Trust Center significance is always true
      ZDP_MgmtPermitJoinReq( &tmpAddr, gPermitDuration, TRUE, FALSE );
    }
  }

  if ( shift && ( keys & HAL_KEY_SW_5 ) )
  {
    zclSampleDoorLockController_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    // for PIN screen
    if ( giDoorLockScreenMode == DOORLOCK_PINMODE )
    {
      if ( giDoorLockPINCount > 0 )
      {
        _itoa(giDoorLockPINCount, &numBuff, 10);    // convert number to ASCII

        aiDoorLockPIN[giDoorLockPINColumnCount + 1] = numBuff;   // copy current PIN number
      }
      else
      {
        // make sure '0' is copied into PIN variable
        aiDoorLockPIN[giDoorLockPINColumnCount + 1] = 0x30;   // ASCII '0'
      }

      if ( giDoorLockPINColumnCount < 3 )
      {
        giDoorLockPINColumnCount++;   // adjust PIN column
      }
      else
      {
        giDoorLockPINColumnCount = 4;   // hold here until PIN screen reset
        giDoorLockPINCount = 0;   // reset PIN count

        // insert the size of the PIN (always 4 for sampledoorlockcontroller)
        aiDoorLockPIN[0] = 4;

        // store PIN to NVM (always 5 bytes)
        osal_nv_write( ZCD_NV_APS_DOORLOCK_PIN, 0, 5, aiDoorLockPIN );
#ifdef LCD_SUPPORTED
        HalLcdWriteString( (char *)sStorePIN, HAL_LCD_LINE_2 );
        HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_3 );
#endif
        giDoorLockScreenMode = DOORLOCK_MAINMODE;
      }
    }
    else if ( giDoorLockScreenMode == DOORLOCK_MAINMODE )
    {
      giDoorLockScreenMode = DOORLOCK_HELPMODE;
    }
    else if ( giDoorLockScreenMode == DOORLOCK_HELPMODE )
    {
#ifdef LCD_SUPPORTED
      HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
#endif
      giDoorLockScreenMode = DOORLOCK_MAINMODE;
    }
  }

  // update display
  zclSampleDoorLockController_LcdDisplayUpdate();
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleDoorLockController_LcdDisplayUpdate(void)
{
  if ( giDoorLockScreenMode == DOORLOCK_PINMODE )
  {
    zclSampleDoorLockController_LcdDisplayPINMode();
  }
  else if ( giDoorLockScreenMode == DOORLOCK_HELPMODE )
  {
    zclSampleDoorLockController_LcdDisplayHelpMode();
  }
  else
  {
    zclSampleDoorLockController_LcdDisplayMainMode();
  }
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleDoorLockController_LcdDisplayMainMode(void)
{
  char sPermitJoinStatus[16];

  if ( zclSampleDoorLockController_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( ZCL_HA_DEVICE_COORDINATOR );
  }
  else if ( zclSampleDoorLockController_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( ZCL_HA_DEVICE_ROUTER );
  }
  else if ( zclSampleDoorLockController_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1( ZCL_HA_DEVICE_END_DEVICE );
  }

#ifdef LCD_SUPPORTED
  if ( ( zclSampleDoorLockController_NwkState == DEV_ZB_COORD ) ||
       ( zclSampleDoorLockController_NwkState == DEV_ROUTER ) )
  {
    if ( gPermitDuration )
    {
      osal_memcpy(sPermitJoinStatus, "SW5: Help      *", 16);
    }
    else
    {
      osal_memcpy(sPermitJoinStatus, "SW5: Help       ", 16);
    }
    // display help key with permit join status
    HalLcdWriteString( (char *)sPermitJoinStatus, HAL_LCD_LINE_3 );
  }
  else
  {
    // display help key
    HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3 );
  }
#endif
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_LcdDisplayHelpMode
 *
 * @brief   Called to display the SW options on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleDoorLockController_LcdDisplayHelpMode(void)
{
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sSwLock, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwPIN, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_LcdDisplayPINMode
 *
 * @brief   Called to display the PIN screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleDoorLockController_LcdDisplayPINMode(void)
{
  char sDisplayPIN[16];

  // cover appropriate columns with security '*'
  if ( giDoorLockPINColumnCount == 0 )
  {
    osal_memcpy(sDisplayPIN, "Enter PIN: ", 11);
    _ltoa(giDoorLockPINCount, (void *)(&sDisplayPIN[11 + giDoorLockPINColumnCount]), 10);
  }
  else if ( giDoorLockPINColumnCount == 1 )
  {
    osal_memcpy(sDisplayPIN, "Enter PIN: *", 12);
    _ltoa(giDoorLockPINCount, (void *)(&sDisplayPIN[11 + giDoorLockPINColumnCount]), 10);
  }
  else if ( giDoorLockPINColumnCount == 2 )
  {
    osal_memcpy(sDisplayPIN, "Enter PIN: **", 13);
    _ltoa(giDoorLockPINCount, (void *)(&sDisplayPIN[11 + giDoorLockPINColumnCount]), 10);
  }
  else if ( giDoorLockPINColumnCount == 3 )
  {
    osal_memcpy(sDisplayPIN, "Enter PIN: ***", 14);
    _ltoa(giDoorLockPINCount, (void *)(&sDisplayPIN[11 + giDoorLockPINColumnCount]), 10);
  }
  else if ( giDoorLockPINColumnCount > 3 )
  {
    osal_memcpy(sDisplayPIN, "Enter PIN: **** ", 16);
  }
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sDisplayPIN, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sPINLine2, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sPINLine3, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessAppMsg
 *
 * @brief   Process DoorLock messages
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   len - number of bytes
 * @param   msg - pointer to message
 *          0 - lo byte destination address
 *          1 - hi byte destination address
 *          2 - destination endpoint
 *          3 - lo byte cluster ID
 *          4 - hi byte cluster ID
 *          5 - message length
 *          6 - destination address mode (first byte of data)
 *          7 - zcl command frame
 *
 * @return  none
 */
static void zclSampleDoorLockController_ProcessAppMsg( uint8 srcEP, uint8 len, uint8 *msg )
{
  afAddrType_t dstAddr;
  uint16 clusterID;
  zclFrameHdr_t hdr;
  uint8 *pData;
  uint8 dataLen;

  dstAddr.addr.shortAddr = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;
  dstAddr.endPoint = *msg++;
  clusterID = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;
  dataLen = *msg++; // Length of message (Z-Tool can support up to 255 octets)
  dstAddr.addrMode = (afAddrMode_t)(*msg++);
  dataLen--; // Length of ZCL frame

  // Begining of ZCL frame
  pData = zclParseHdr( &hdr, msg );
  dataLen -= (uint8)( pData - msg );

  // Is this a foundation type message?
  if ( zcl_ProfileCmd( hdr.fc.type ) )
  {
    if ( hdr.fc.manuSpecific )
    {
      // We don't support any manufacturer specific command -- just forward it.
      zcl_SendCommand( srcEP, &dstAddr, clusterID, hdr.commandID, FALSE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr.fc.disableDefaultRsp, hdr.manuCode, hdr.transSeqNum, dataLen, pData );
    }
    else
    {
      zclParseCmd_t cmd;

      cmd.endpoint = srcEP;
      cmd.dataLen = dataLen;
      cmd.pData = pData;

      zclSampleDoorLockController_ProcessFoundationMsg( &dstAddr, clusterID, &hdr, &cmd );
    }
  }
  else
  {
    // Nope, must be specific to the cluster ID
    if ( hdr.fc.manuSpecific )
    {
      // We don't support any manufacturer specific command -- just forward it.
      zcl_SendCommand( srcEP, &dstAddr, clusterID, hdr.commandID, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr.fc.disableDefaultRsp, hdr.manuCode, hdr.transSeqNum, dataLen, pData );
    }
    else
    {
      if ( ZCL_CLUSTER_ID_GEN( clusterID ) )
      {
        zclSampleDoorLockController_ProcessGeneralMsg( srcEP, &dstAddr, clusterID, &hdr, dataLen, pData );
      }
      else if ( ZCL_CLUSTER_ID_CLOSURES( clusterID ) )
      {
        zclSampleDoorLockController_ProcessClosuresMsg( srcEP, &dstAddr, clusterID, &hdr, dataLen, pData );
      }
    }
  }
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessFoundationMsg
 *
 * @brief   Process Foundation message
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   dstAddr - where to send the request
 * @param   clusterID - real cluster ID
 * @param   hdr - pointer to the message header
 * @param   len - length of the received message
 * @param   data - received message
 *
 * @return  none
 */
static void zclSampleDoorLockController_ProcessFoundationMsg( afAddrType_t *dstAddr, uint16 clusterID,
                                                zclFrameHdr_t *hdr, zclParseCmd_t *pParseCmd )
{
#if defined(ZCL_READ) || defined(ZCL_WRITE) || defined(ZCL_REPORT) || defined(ZCL_DISCOVER)
  void *cmd;
#endif

  switch ( hdr->commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ:
      cmd = zclParseInReadCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendRead( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID, (zclReadCmd_t *)cmd,
                      ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_READ

#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE:
      cmd = zclParseInWriteCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendWrite( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID, (zclWriteCmd_t *)cmd,
                       ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_WRITE_UNDIVIDED:
      cmd = zclParseInWriteCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendWriteUndivided( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID, (zclWriteCmd_t *)cmd,
                                ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_WRITE_NO_RSP:
      cmd = zclParseInWriteCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendWriteNoRsp( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID, (zclWriteCmd_t *)cmd,
                            ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_WRITE

#ifdef ZCL_REPORT
    case ZCL_CMD_CONFIG_REPORT:
      cmd = zclParseInConfigReportCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendConfigReportCmd( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr,  clusterID, (zclCfgReportCmd_t *)cmd,
                                 ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      cmd = zclParseInReadReportCfgCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendReadReportCfgCmd( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID, (zclReadReportCfgCmd_t *)cmd,
                                  ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_REPORT:
      cmd = zclParseInReportCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendReportCmd( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID, (zclReportCmd_t *)cmd,
                           ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_REPORT

    default:
      // Unsupported command -- just forward it.
      zcl_SendCommand( pParseCmd->endpoint, dstAddr, clusterID, hdr->commandID, FALSE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr->fc.disableDefaultRsp, 0, hdr->transSeqNum, pParseCmd->dataLen, pParseCmd->pData );
      break;
  }
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessGeneralMsg
 *
 * @brief   Process General Cluster message
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   dstAddr - where to send the request
 * @param   clusterID - real cluster ID
 * @param   hdr - pointer to the message header
 * @param   len - length of the received message
 * @param   data - received message
 *
 * @return  none
 */
static void zclSampleDoorLockController_ProcessGeneralMsg( uint8 srcEP, afAddrType_t *dstAddr,
                  uint16 clusterID, zclFrameHdr_t *hdr, uint8 len, uint8 *data )
{
  switch ( clusterID )
  {
#ifdef ZCL_BASIC
    case ZCL_CLUSTER_ID_GEN_BASIC:
      if ( hdr->commandID == COMMAND_BASIC_RESET_FACT_DEFAULT )
      {
        zclGeneral_SendBasicResetFactoryDefaults( srcEP, dstAddr, hdr->fc.disableDefaultRsp,
                                                  hdr->transSeqNum );
      }
      break;
#endif // ZCL_BASIC

#ifdef ZCL_GROUPS
    case ZCL_CLUSTER_ID_GEN_GROUPS:
      zclSampleDoorLockController_ProcessGroupCmd( srcEP, dstAddr, clusterID, hdr, len, data );
      break;
#endif // ZCL_GROUPS

#ifdef ZCL_SCENES
    case ZCL_CLUSTER_ID_GEN_SCENES:
      zclSampleDoorLockController_ProcessSceneCmd( srcEP, dstAddr, clusterID, hdr, len, data  );
      break;
#endif // ZCL_SCENES

    default:
      break;
  }
}

#ifdef ZCL_GROUPS
/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessGroupCmd
 *
 * @brief   Process Group Command
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   dstAddr - where to send the request
 * @param   clusterID - real cluster ID
 * @param   hdr - pointer to the message header
 * @param   len - length of the received message
 * @param   data - received message
 *
 * @return  none
 */
static void zclSampleDoorLockController_ProcessGroupCmd( uint8 srcEP, afAddrType_t *dstAddr,
                  uint16 clusterID, zclFrameHdr_t *hdr, uint8 len, uint8 *data )
{
  uint16 groupID;
  uint8 grpCnt;
  uint16 *grpList;

  switch ( hdr->commandID )
  {
    case COMMAND_GROUP_ADD:
      groupID =  BUILD_UINT16( data[0], data[1] );
      data += 2;
      zclGeneral_SendGroupAdd( srcEP, dstAddr, groupID, data,
                               hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_GROUP_VIEW:
      groupID =  BUILD_UINT16( data[0], data[1] );
      zclGeneral_SendGroupView( srcEP, dstAddr, groupID,
                                hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_GROUP_GET_MEMBERSHIP:
      grpCnt = data[0];
      grpList = (uint16 *)(&data[1]);
      zclGeneral_SendGroupGetMembership( srcEP, dstAddr, grpCnt, grpList,
                                         hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_GROUP_REMOVE:
      groupID =  BUILD_UINT16( data[0], data[1] );
      zclGeneral_SendGroupRemove( srcEP,  dstAddr, groupID,
                                  hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_GROUP_REMOVE_ALL:
       zclGeneral_SendGroupRemoveAll( srcEP, dstAddr, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_GROUP_ADD_IF_IDENTIFYING:
      groupID =  BUILD_UINT16( data[0], data[1] );
      data += 2;
      zclGeneral_SendGroupAddIfIdentifying( srcEP, dstAddr, groupID, data,
                                            hdr->fc.disableDefaultRsp,  hdr->transSeqNum );
      break;

    default:
      // Unsupported command -- just forward it.
      zcl_SendCommand( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID,
                       hdr->commandID, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr->fc.disableDefaultRsp, 0, hdr->transSeqNum, len, data );
      break;
  }
}
#endif // ZCL_GROUPS

#ifdef ZCL_SCENES
/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessSceneCmd
 *
 * @brief   Process Scene Command
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   dstAddr - where to send the request
 * @param   clusterID - real cluster ID
 * @param   hdr - pointer to the message header
 * @param   len - length of the received message
 * @param   data - received message
 *
 * @return  none
 */
static void zclSampleDoorLockController_ProcessSceneCmd( uint8 srcEP, afAddrType_t *dstAddr,
                  uint16 clusterID, zclFrameHdr_t *hdr, uint8 len, uint8 *data )
{
  zclGeneral_Scene_t scene;
  uint8 *pData = data;
  uint8 nameLen;

  osal_memset( (uint8*)&scene, 0, sizeof( zclGeneral_Scene_t ) );

  scene.groupID = BUILD_UINT16( pData[0], pData[1] );
  pData += 2;   // Move past group ID
  scene.ID = *pData++;

  switch ( hdr->commandID )
  {
    case COMMAND_SCENE_ADD:
      // Parse the rest of the incoming message
      scene.transTime = BUILD_UINT16( pData[0], pData[1] );
      pData += 2;
      nameLen = *pData++; // Name length
      if ( nameLen > (ZCL_SCENE_NAME_LEN-1) )
      {
        scene.name[0] = ZCL_SCENE_NAME_LEN-1;
      }
      else
      {
        scene.name[0] = nameLen;
      }
      osal_memcpy( &(scene.name[1]), pData, scene.name[0] );
      pData += nameLen; // move past name, use original length

      // Add the extension field(s)
      scene.extLen = len - ( (uint8)( pData - data ) );
      if ( scene.extLen > 0 )
      {
        // Copy the extention field(s)
        if ( scene.extLen > ZCL_GEN_SCENE_EXT_LEN )
        {
          scene.extLen = ZCL_GEN_SCENE_EXT_LEN;
        }
        osal_memcpy( scene.extField, pData, scene.extLen );
      }

      zclGeneral_SendAddScene( srcEP, dstAddr, &scene,
                               hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_SCENE_VIEW:
      zclGeneral_SendSceneView( srcEP, dstAddr, scene.groupID, scene.ID,
                                hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_SCENE_REMOVE:
      zclGeneral_SendSceneRemove( srcEP, dstAddr, scene.groupID, scene.ID,
                                  hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_SCENE_REMOVE_ALL:
      zclGeneral_SendSceneRemoveAll( srcEP, dstAddr, scene.groupID,
                                     hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_SCENE_STORE:
      zclGeneral_SendSceneStore( srcEP, dstAddr, scene.groupID, scene.ID,
                                 hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_SCENE_RECALL:
      zclGeneral_SendSceneRecall( srcEP, dstAddr, scene.groupID, scene.ID,
                                  hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    case COMMAND_SCENE_GET_MEMBERSHIP:
      zclGeneral_SendSceneGetMembership( srcEP, dstAddr, scene.groupID,
                                         hdr->fc.disableDefaultRsp, hdr->transSeqNum );
      break;

    default:
      // Unsupported command -- just forward it.
      zcl_SendCommand( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID,
                       hdr->commandID, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr->fc.disableDefaultRsp, 0, hdr->transSeqNum, len, data );
      break;
  }
}
#endif // ZCL_SCENES

/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessClosuresMsg
 *
 * @brief   Process Closures Cluster Command
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   dstAddr - where to send the request
 * @param   clusterID - real cluster ID
 * @param   hdr - pointer to the message header
 * @param   len - length of the received message
 * @param   data - received message
 *
 * @return  none
 */
static void zclSampleDoorLockController_ProcessClosuresMsg( uint8 srcEP, afAddrType_t *dstAddr,
                                                            uint16 clusterID, zclFrameHdr_t *hdr,
                                                            uint8 len, uint8 *data )
{
#ifdef ZCL_DOORLOCK

  uint8 i;
  uint16 calculatedArraySize;

  if ( clusterID == ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK )
  {
    // Client-to-Server
    if ( zcl_ServerCmd( hdr->fc.direction ) )
    {
      switch( hdr->commandID )
      {
        case COMMAND_CLOSURES_LOCK_DOOR:
        {
          zclDoorLock_t cmd;

          // first octet of PIN/RFID Code variable string identifies its length
          calculatedArraySize = data[0] + 1;

          cmd.pPinRfidCode = osal_mem_alloc( calculatedArraySize );
          if ( !cmd.pPinRfidCode )
          {
            return;  // no memory
          }

          for ( i = 0; i < calculatedArraySize; i++ )
          {
            cmd.pPinRfidCode[i] = data[i];
          }

          zclClosures_SendDoorLockLockDoor( srcEP, dstAddr, &cmd, hdr->fc.disableDefaultRsp, hdr->transSeqNum );

          osal_mem_free( cmd.pPinRfidCode );
          break;
        }

        case COMMAND_CLOSURES_UNLOCK_DOOR:
        {
          zclDoorLock_t cmd;

          // first octet of PIN/RFID Code variable string identifies its length
          calculatedArraySize = data[0] + 1;

          cmd.pPinRfidCode = osal_mem_alloc( calculatedArraySize );
          if ( !cmd.pPinRfidCode )
          {
            return;  // no memory
          }

          for ( i = 0; i < calculatedArraySize; i++ )
          {
            cmd.pPinRfidCode[i] = data[i];
          }

          zclClosures_SendDoorLockUnlockDoor( srcEP, dstAddr, &cmd, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
          osal_mem_free( cmd.pPinRfidCode );
          break;
        }

        case COMMAND_CLOSURES_TOGGLE_DOOR:
        {
          zclDoorLock_t cmd;

          // first octet of PIN/RFID Code variable string identifies its length
          calculatedArraySize = data[0] + 1;

          cmd.pPinRfidCode = osal_mem_alloc( calculatedArraySize );
          if ( !cmd.pPinRfidCode )
          {
            return;  // no memory
          }

          for ( i = 0; i < calculatedArraySize; i++ )
          {
            cmd.pPinRfidCode[i] = data[i];
          }

          zclClosures_SendDoorLockToggleDoor( srcEP, dstAddr, &cmd, hdr->fc.disableDefaultRsp, hdr->transSeqNum );

          osal_mem_free( cmd.pPinRfidCode );
          break;
        }

        default:
          // Unsupported command -- just forward it.
          zcl_SendCommand( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID,
                           hdr->commandID, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR,
                           hdr->fc.disableDefaultRsp, 0, hdr->transSeqNum, len, data );
        break;
      }
    }
    // Server-to-Client
    else
    {
      switch( hdr->commandID )
      {
        case COMMAND_CLOSURES_LOCK_DOOR_RSP:
          zclClosures_SendDoorLockLockDoorRsp( srcEP, dstAddr, data[0], hdr->fc.disableDefaultRsp, hdr->transSeqNum );
          break;

        case COMMAND_CLOSURES_UNLOCK_DOOR_RSP:
          zclClosures_SendDoorLockUnlockDoorRsp( srcEP, dstAddr, data[0], hdr->fc.disableDefaultRsp, hdr->transSeqNum );
          break;

        case COMMAND_CLOSURES_TOGGLE_DOOR_RSP:
          zclClosures_SendDoorLockToggleDoorRsp( srcEP, dstAddr, data[0], hdr->fc.disableDefaultRsp, hdr->transSeqNum );
          break;

        default:
          // Unsupported command -- just forward it.
          zcl_SendCommand( SAMPLEDOORLOCKCONTROLLER_ENDPOINT, dstAddr, clusterID,
                           hdr->commandID, TRUE, ZCL_FRAME_SERVER_CLIENT_DIR,
                           hdr->fc.disableDefaultRsp, 0, hdr->transSeqNum, len, data );
          break;
      }
    }
  }
  else
  {
    return; // unsupported cluster
  }
#endif // ZCL_DOORLOCK
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleDoorLockController_ProcessIdentifyTimeChange( void )
{
  if ( zclSampleDoorLockController_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleDoorLockController_TaskID, SAMPLEDOORLOCKCONTROLLER_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSampleDoorLockController_OnOff )
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    else
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    osal_stop_timerEx( zclSampleDoorLockController_TaskID, SAMPLEDOORLOCKCONTROLLER_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleDoorLockController_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network

  // restart device
  MT_SysCommandProcessing( aProcessCmd );
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclSampleDoorLockController_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSampleDoorLockController_IdentifyTime = pCmd->identifyTime;
  zclSampleDoorLockController_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclSampleDoorLockController_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
{
  // Query Response (with timeout value)
  (void)pRsp;
#ifdef ZCL_EZMODE
  {
    zclEZMode_ActionData_t data;
    data.pIdentifyQueryRsp = pRsp;
    zcl_EZModeAction ( EZMODE_ACTION_IDENTIFY_QUERY_RSP, &data );
  }
#endif
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_DoorLockRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Door Lock response for this application.
 *
 * @param   pInMsg - process incoming message
 * @param   status - response status of requesting door lock cmd
 *
 * @return  status
 */
static ZStatus_t zclSampleDoorLockController_DoorLockRspCB ( zclIncoming_t *pInMsg, uint8 status )
{
  zclReadCmd_t readCmd;

  if ( status == ZSuccess )
  {
    readCmd.numAttr = 1;
    readCmd.attrID[0] = ATTRID_CLOSURES_LOCK_STATE;

    pInMsg->hdr.transSeqNum += 1;

    zcl_SendRead( pInMsg->msg->endPoint, &pInMsg->msg->srcAddr,
                  ZCL_CLUSTER_ID_CLOSURES_DOOR_LOCK, &readCmd,
                  ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, pInMsg->hdr.transSeqNum );
  }
  // ZFailure
  else
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sFail, HAL_LCD_LINE_2 );
#endif
  }

  return ( ZCL_STATUS_SUCCESS );
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleDoorLockController_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleDoorLockController_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleDoorLockController_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleDoorLockController_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleDoorLockController_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleDoorLockController_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleDoorLockController_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclSampleDoorLockController_ProcessInReportCmd( pInMsg );
      break;
#endif
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleDoorLockController_ProcessInDiscAttrsRspCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleDoorLockController_ProcessInDefaultRspCmd( pInMsg );
      break;

    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleDoorLockController_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;

  // update the display
  giDoorLockScreenMode = DOORLOCK_MAINMODE;
  zclSampleDoorLockController_LcdDisplayUpdate();

  // read the lock state and display the information
  if ( readRspCmd->attrList[0].data[0] == 1 )
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sDoorLocked, HAL_LCD_LINE_2 ); // locked
#endif
    HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
    HalLedSet( HAL_LED_2, HAL_LED_MODE_BLINK );
  }
  else if ( readRspCmd->attrList[0].data[0] == 2 )
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sDoorUnlocked, HAL_LCD_LINE_2 ); // unlocked
#endif
    HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK );
    HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
  }

  return TRUE;
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleDoorLockController_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return TRUE;
}
#endif // ZCL_WRITE

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleDoorLockController_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}
#endif // ZCL_DISCOVER

/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleDoorLockController_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return TRUE;
}

#if ZCL_EZMODE
/*********************************************************************
 * @fn      zclSampleDoorLockController_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSampleDoorLockController_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
{
  zclEZMode_ActionData_t data;
  ZDO_MatchDescRsp_t *pMatchDescRsp;

  // Let EZ-Mode know of the Match Descriptor Response
  if ( pMsg->clusterID == Match_Desc_rsp )
  {
    pMatchDescRsp = ZDO_ParseEPListRsp( pMsg );
    data.pMatchDescRsp = pMatchDescRsp;
    zcl_EZModeAction( EZMODE_ACTION_MATCH_DESC_RSP, &data );
    osal_mem_free( pMatchDescRsp );
  }
}

/*********************************************************************
 * @fn      zclSampleDoorLockController_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclSampleDoorLockController_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char szLine[20];
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

    zclSampleDoorLockController_IdentifyTime = (EZMODE_TIME / 1000);  // convert to seconds
    zclSampleDoorLockController_ProcessIdentifyTimeChange();
  }

  // autoclosing, show what happened (success, cancelled, etc...)
  if( state == EZMODE_STATE_AUTOCLOSE )
  {
#ifdef LCD_SUPPORTED
    pStr = NULL;
    err = pData->sAutoClose.err;
    if ( err == EZMODE_ERR_SUCCESS )
    {
      pStr = "EZMode: Success";
    }
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    if ( pStr )
    {
      if ( giDoorLockScreenMode == DOORLOCK_MAINMODE )
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclSampleDoorLockController_IdentifyTime = 0;
    zclSampleDoorLockController_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // "EZDst:1234 EP:34"
      osal_memcpy(szLine, "EZDst:", 6);
      zclHA_uint16toa( pData->sFinish.nwkaddr, &szLine[6]);
      osal_memcpy(&szLine[10], " EP:", 4);
      _ltoa( pData->sFinish.ep, (void *)(&szLine[14]), 16 );  // _ltoa NULL terminates
      pStr = szLine;
    }
    else if ( err == EZMODE_ERR_BAD_PARAMETER )
    {
      pStr = "EZMode: BadParm";
    }
    else if ( err == EZMODE_ERR_CANCELLED )
    {
      pStr = "EZMode: Cancel";
    }
    else
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giDoorLockScreenMode == DOORLOCK_MAINMODE )
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif  // LCD_SUPPORTED

    // show main UI screen 3 seconds after binding
    osal_start_timerEx( zclSampleDoorLockController_TaskID, SAMPLEDOORLOCKCONTROLLER_MAIN_SCREEN_EVT, 3000 );
  }

}

#endif // ZCL_EZMODE

/****************************************************************************
****************************************************************************/
