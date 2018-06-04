/**************************************************************************************************
  Filename:       zcl_sampleheatingcoolingunit.c
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
  This device will act as a heating and cooling unit

  ----------------------------------------
  Main:
    - SW2: Invoke EZMode
    - SW4: Enable/Disable Permit Join
    - SW5: Go to Help screen
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
#include "ZDProfile.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_ms.h"
#include "zcl_hvac.h"

#include "zcl_sampleheatingcoolingunit.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 zclSampleHeatingCoolingUnit_TaskID;

uint8 zclSampleHeatingCoolingUnitSeqNum;

static uint8 gPermitDuration = 0;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleHeatingCoolingUnit_DstAddr;

// EZ-Mode setup
#ifdef ZCL_EZMODE
static void zclSampleHeatingCoolingUnit_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleHeatingCoolingUnit_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclSampleHeatingCoolingUnit_RegisterEZModeData =
{
  &zclSampleHeatingCoolingUnit_TaskID,
  SAMPLEHEATINGCOOLINGUNIT_EZMODE_NEXTSTATE_EVT,
  SAMPLEHEATINGCOOLINGUNIT_EZMODE_TIMEOUT_EVT,
  &zclSampleHeatingCoolingUnitSeqNum,
  zclSampleHeatingCoolingUnit_EZModeCB
};

// No ZCL_EZMODE, use EndDeviceBind
#else

static cId_t bindingInClusters[] =
{
  ZCL_CLUSTER_ID_HVAC_THERMOSTAT
};
#define ZCLSAMPLEHEATINGCOOLINGUNIT_BINDINGLIST   1
#endif  // ZCL_EZMODE

devStates_t zclSampleHeatingCoolingUnit_NwkState = DEV_INIT;

uint8 giHeatingCoolingUnitScreenMode = HEATCOOLUNIT_MAINMODE;   // display main screen mode first

static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleLight_TestEp =
{
  20,                                 // Test endpoint
  &zclSampleHeatingCoolingUnit_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleHeatingCoolingUnit_HandleKeys( byte shift, byte keys );
static void zclSampleHeatingCoolingUnit_BasicResetCB( void );
static void zclSampleHeatingCoolingUnit_IdentifyCB( zclIdentify_t *pCmd );
static void zclSampleHeatingCoolingUnit_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclSampleHeatingCoolingUnit_OnOffCB( uint8 cmd );
static void zclSampleHeatingCoolingUnit_ProcessIdentifyTimeChange( void );

// app display functions
void zclSampleHeatingCoolingUnit_LcdDisplayUpdate( void );
void zclSampleHeatingCoolingUnit_LcdDisplayMainMode( void );
void zclSampleHeatingCoolingUnit_LcdDisplayTempMode( void );
void zclSampleHeatingCoolingUnit_LcdDisplayHelpMode( void );

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleHeatingCoolingUnit_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleHeatingCoolingUnit_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleHeatingCoolingUnit_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_REPORT
static void zclSampleHeatingCoolingUnit_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleHeatingCoolingUnit_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]    = " ";
const char sDeviceName[]   = " Heat/Cool Unit";
const char sSwEZMode[]     = "SW2: EZ-Mode";
const char sSwHelp[]       = "SW5: Help";
const char sHeatMode[]     = "  HEATING MODE";
const char sCoolMode[]     = "  COOLING MODE";
const char sOffMode[]      = "   SYSTEM OFF";
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleHeatingCoolingUnit_CmdCallbacks =
{
  zclSampleHeatingCoolingUnit_BasicResetCB,         // Basic Cluster Reset command
  zclSampleHeatingCoolingUnit_IdentifyCB,           // Identify command
#ifdef ZCL_EZMODE
  NULL,                                             // Identify EZ-Mode Invoke command
  NULL,                                             // Identify Update Commission State command
#endif
  NULL,                                             // Identify Trigger Effect command
  zclSampleHeatingCoolingUnit_IdentifyQueryRspCB,   // Identify Query Response command
  zclSampleHeatingCoolingUnit_OnOffCB,              // On/Off cluster command
  NULL,                                             // On/Off cluster enhanced command Off with Effect
  NULL,                                             // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                             // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                             // Level Control Move to Level command
  NULL,                                             // Level Control Move command
  NULL,                                             // Level Control Step command
  NULL,                                             // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                             // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                             // Scene Store Request command
  NULL,                                             // Scene Recall Request command
  NULL,                                             // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                             // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                             // Get Event Log command
  NULL,                                             // Publish Event Log command
#endif
  NULL,                                             // RSSI Location command
  NULL                                              // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclSampleHeatingCoolingUnit_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleHeatingCoolingUnit_Init( byte task_id )
{
  zclSampleHeatingCoolingUnit_TaskID = task_id;

  // Set destination address to indirect
  zclSampleHeatingCoolingUnit_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleHeatingCoolingUnit_DstAddr.endPoint = 0;
  zclSampleHeatingCoolingUnit_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleHeatingCoolingUnit_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLEHEATINGCOOLINGUNIT_ENDPOINT, &zclSampleHeatingCoolingUnit_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( SAMPLEHEATINGCOOLINGUNIT_ENDPOINT, SAMPLEHEATINGCOOLINGUNIT_MAX_ATTRIBUTES, zclSampleHeatingCoolingUnit_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleHeatingCoolingUnit_TaskID );

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleHeatingCoolingUnit_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleHeatingCoolingUnit_TaskID );

  // Register for a test endpoint
  afRegister( &sampleLight_TestEp );

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
uint16 zclSampleHeatingCoolingUnit_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleHeatingCoolingUnit_TaskID ) ) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclSampleHeatingCoolingUnit_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif

        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleHeatingCoolingUnit_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSampleHeatingCoolingUnit_HandleKeys( ( (keyChange_t *)MSGpkt)->state, ( (keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleHeatingCoolingUnit_NwkState = (devStates_t)(MSGpkt->hdr.status);


          // now on the network
          if ( ( zclSampleHeatingCoolingUnit_NwkState == DEV_ZB_COORD ) ||
               ( zclSampleHeatingCoolingUnit_NwkState == DEV_ROUTER )   ||
               ( zclSampleHeatingCoolingUnit_NwkState == DEV_END_DEVICE ) )
          {
#ifndef HOLD_AUTO_START
            giHeatingCoolingUnitScreenMode = HEATCOOLUNIT_MAINMODE;

            zclSampleHeatingCoolingUnit_LcdDisplayMainMode();
#endif
#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif // ZCL_EZMODE
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

  if ( events & SAMPLEHEATINGCOOLINGUNIT_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSampleHeatingCoolingUnit_IdentifyTime > 0 )
    {
      zclSampleHeatingCoolingUnit_IdentifyTime--;
    }

    zclSampleHeatingCoolingUnit_ProcessIdentifyTimeChange();

    return ( events ^ SAMPLEHEATINGCOOLINGUNIT_IDENTIFY_TIMEOUT_EVT );
  }

  if ( events & SAMPLEHEATINGCOOLINGUNIT_MAIN_SCREEN_EVT )
  {
    giHeatingCoolingUnitScreenMode = HEATCOOLUNIT_MAINMODE;

    zclSampleHeatingCoolingUnit_LcdDisplayMainMode();

    return ( events ^ SAMPLEHEATINGCOOLINGUNIT_MAIN_SCREEN_EVT );
  }

#ifdef ZCL_EZMODE
  // going on to next state
  if ( events & SAMPLEHEATINGCOOLINGUNIT_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SAMPLEHEATINGCOOLINGUNIT_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & SAMPLEHEATINGCOOLINGUNIT_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SAMPLEHEATINGCOOLINGUNIT_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *
 * @return  none
 */
static void zclSampleHeatingCoolingUnit_HandleKeys( byte shift, byte keys )
{
  if ( keys & HAL_KEY_SW_2 )
  {
    if ( ( giHeatingCoolingUnitScreenMode == HEATCOOLUNIT_MAINMODE ) ||
         ( giHeatingCoolingUnitScreenMode == HEATCOOLUNIT_HELPMODE ) )
    {
      giHeatingCoolingUnitScreenMode = HEATCOOLUNIT_MAINMODE;

#ifdef ZCL_EZMODE
      // Invoke EZ-Mode
      zclEZMode_InvokeData_t ezModeData;

      // Invoke EZ-Mode
      ezModeData.endpoint = SAMPLEHEATINGCOOLINGUNIT_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( ( zclSampleHeatingCoolingUnit_NwkState == DEV_ZB_COORD ) ||
           ( zclSampleHeatingCoolingUnit_NwkState == DEV_ROUTER )   ||
           ( zclSampleHeatingCoolingUnit_NwkState == DEV_END_DEVICE ) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = FALSE;        // Heating/Cooling Unit is a target
      ezModeData.numActiveOutClusters = 0;   // active output cluster
      zcl_InvokeEZMode( &ezModeData );

#ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

      // NOT ZCL_EZMODE, use EndDeviceBind
#else
      {
        zAddrType_t dstAddr;
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

        // Initiate an End Device Bind Request, this bind request will
        // only use a cluster list that is important to binding.
        dstAddr.addrMode = afAddr16Bit;
        dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
        ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                             SAMPLEHEATINGCOOLINGUNIT_ENDPOINT,
                             ZCL_HA_PROFILE_ID,
                             ZCLSAMPLEHEATINGCOOLINGUNIT_BINDINGLIST, bindingInClusters,
                             0, NULL,   // No Outgoing clusters to bind
                             TRUE );
      }
#endif // ZCL_EZMODE
    }
  }

  if ( keys & HAL_KEY_SW_4 )
  {
    giHeatingCoolingUnitScreenMode = HEATCOOLUNIT_MAINMODE;

    if ( ( zclSampleHeatingCoolingUnit_NwkState == DEV_ZB_COORD ) ||
        ( zclSampleHeatingCoolingUnit_NwkState == DEV_ROUTER ) )
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
    zclSampleHeatingCoolingUnit_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    if ( keys & HAL_KEY_SW_5 )
    {
      if ( giHeatingCoolingUnitScreenMode == HEATCOOLUNIT_MAINMODE )
      {
        giHeatingCoolingUnitScreenMode = HEATCOOLUNIT_HELPMODE;
      }
      else if ( giHeatingCoolingUnitScreenMode == HEATCOOLUNIT_HELPMODE )
      {
#ifdef LCD_SUPPORTED
        HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
#endif
        giHeatingCoolingUnitScreenMode = HEATCOOLUNIT_MAINMODE;
      }
    }
  }

  // update display
  zclSampleHeatingCoolingUnit_LcdDisplayUpdate();
}

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleHeatingCoolingUnit_LcdDisplayUpdate( void )
{
  if ( giHeatingCoolingUnitScreenMode == HEATCOOLUNIT_HELPMODE )
  {
    zclSampleHeatingCoolingUnit_LcdDisplayHelpMode();
  }
  else
  {
    zclSampleHeatingCoolingUnit_LcdDisplayMainMode();
  }
}

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleHeatingCoolingUnit_LcdDisplayMainMode( void )
{
  if ( zclSampleHeatingCoolingUnit_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( 0 );
  }
  else if ( zclSampleHeatingCoolingUnit_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( 1 );
  }
  else if ( zclSampleHeatingCoolingUnit_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1( 2 );
  }

  // display current state of heating/cooling unit
  if ( ( zclSampleHeatingCoolingUnit_RunningState == HVAC_THERMOSTAT_RUNNING_STATE_HEAT_1ST_STAGE_ON ) ||
       ( zclSampleHeatingCoolingUnit_RunningState == HVAC_THERMOSTAT_RUNNING_STATE_HEAT_2ND_STAGE_ON ) ||
       ( zclSampleHeatingCoolingUnit_RunningState == HVAC_THERMOSTAT_RUNNING_STATE_HEAT_3RD_STAGE_ON ) )
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sHeatMode, HAL_LCD_LINE_2 );
#endif
  }
  else if ( ( zclSampleHeatingCoolingUnit_RunningState == HVAC_THERMOSTAT_RUNNING_STATE_COOL_1ST_STAGE_ON ) ||
            ( zclSampleHeatingCoolingUnit_RunningState == HVAC_THERMOSTAT_RUNNING_STATE_COOL_2ND_STAGE_ON ) ||
            ( zclSampleHeatingCoolingUnit_RunningState == HVAC_THERMOSTAT_RUNNING_STATE_COOL_3RD_STAGE_ON ) )
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sCoolMode, HAL_LCD_LINE_2 );
#endif
  }
  else
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sOffMode, HAL_LCD_LINE_2 );
#endif
  }

#ifdef LCD_SUPPORTED
  if ( ( zclSampleHeatingCoolingUnit_NwkState == DEV_ZB_COORD ) ||
       ( zclSampleHeatingCoolingUnit_NwkState == DEV_ROUTER ) )
  {
    // display help key with permit join status
    if ( gPermitDuration )
    {
      HalLcdWriteString("SW5: Help      *", HAL_LCD_LINE_3);
    }
    else
    {
      HalLcdWriteString("SW5: Help       ", HAL_LCD_LINE_3);
    }
  }
  else
  {
    // display help key
    HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3 );
  }
#endif
}

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_LcdDisplayHelpMode
 *
 * @brief   Called to display the SW options on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleHeatingCoolingUnit_LcdDisplayHelpMode( void )
{
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleHeatingCoolingUnit_ProcessIdentifyTimeChange( void )
{
  if ( zclSampleHeatingCoolingUnit_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleHeatingCoolingUnit_TaskID, SAMPLEHEATINGCOOLINGUNIT_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSampleHeatingCoolingUnit_OnOff )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    }
    else
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    }
    osal_stop_timerEx( zclSampleHeatingCoolingUnit_TaskID, SAMPLEHEATINGCOOLINGUNIT_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleHeatingCoolingUnit_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network

  // restart device
  MT_SysCommandProcessing( aProcessCmd );
}

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclSampleHeatingCoolingUnit_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSampleHeatingCoolingUnit_IdentifyTime = pCmd->identifyTime;
  zclSampleHeatingCoolingUnit_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclSampleHeatingCoolingUnit_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
{
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
 * @fn      zclSampleHeatingCoolingUnit_OnOffCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   cmd - COMMAND_ON, COMMAND_OFF or COMMAND_TOGGLE
 *
 * @return  none
 */
static void zclSampleHeatingCoolingUnit_OnOffCB( uint8 cmd )
{
#ifdef ZCL_ON_OFF
  // Turn off the unit
  if ( cmd == COMMAND_OFF )
  {
    zclSampleHeatingCoolingUnit_RunningState = 0;   // unit is off

    HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );

    // update the display
    zclSampleHeatingCoolingUnit_LcdDisplayUpdate();
  }
#endif
}

#ifdef ZCL_REPORT
/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_ProcessInReportCmd
 *
 * @brief   Process the "Profile" Report Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static void zclSampleHeatingCoolingUnit_ProcessInReportCmd( zclIncomingMsg_t *pInMsg )
{
  zclReportCmd_t *pInThermostatReport;

  pInThermostatReport = (zclReportCmd_t *)pInMsg->attrCmd;

  if ( ( pInThermostatReport->attrList[0].attrID != ATTRID_HVAC_THERMOSTAT_PI_HEATING_DEMAND ) &&
       ( pInThermostatReport->attrList[1].attrID != ATTRID_HVAC_THERMOSTAT_PI_COOLING_DEMAND ) )
  {
    return;
  }

  // check to see if heating/cooling unit should be off
  if ( ( pInThermostatReport->attrList[0].attrData[0] == 0 ) &&
       ( pInThermostatReport->attrList[1].attrData[0] == 0 ) )
  {
    // turn heating/cooling off
    zclSampleHeatingCoolingUnit_RunningState = 0;

    HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
  }
  else
  {
    // check if unit needs to turn heat on or off
    if ( pInThermostatReport->attrList[0].attrData[0] > 0 )
    {
      // turn heating on
      zclSampleHeatingCoolingUnit_RunningState = HVAC_THERMOSTAT_RUNNING_STATE_HEAT_1ST_STAGE_ON;

      HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
      HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
    }
    else if ( pInThermostatReport->attrList[0].attrData[0] == 0 )
    {
      // turn heating off
      HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
    }

    // check if unit needs to turn cool on or off
    if ( pInThermostatReport->attrList[1].attrData[0] > 0 )
    {
      // turn cooling on
      zclSampleHeatingCoolingUnit_RunningState = HVAC_THERMOSTAT_RUNNING_STATE_COOL_1ST_STAGE_ON;

      HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
      HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
    }
    else if ( pInThermostatReport->attrList[1].attrData[0] == 0 )
    {
      // turn cooling off
      HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
    }
  }

  // update the display
  zclSampleHeatingCoolingUnit_LcdDisplayUpdate();
}
#endif

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleHeatingCoolingUnit_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleHeatingCoolingUnit_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleHeatingCoolingUnit_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleHeatingCoolingUnit_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleHeatingCoolingUnit_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleHeatingCoolingUnit_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleHeatingCoolingUnit_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      zclSampleHeatingCoolingUnit_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleHeatingCoolingUnit_ProcessInDefaultRspCmd( pInMsg );
      break;

    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleHeatingCoolingUnit_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return TRUE;
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleHeatingCoolingUnit_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
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

/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleHeatingCoolingUnit_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return TRUE;
}

#if ZCL_EZMODE
/*********************************************************************
 * @fn      zclSampleHeatingCoolingUnit_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSampleHeatingCoolingUnit_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
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
 * @fn      zclSampleHeatingCoolingUnit_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclSampleHeatingCoolingUnit_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

    zclSampleHeatingCoolingUnit_IdentifyTime = (EZMODE_TIME / 1000);  // convert to seconds
    zclSampleHeatingCoolingUnit_ProcessIdentifyTimeChange();
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
      if ( giHeatingCoolingUnitScreenMode == HEATCOOLUNIT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclSampleHeatingCoolingUnit_IdentifyTime = 0;
    zclSampleHeatingCoolingUnit_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // already stated on autoclose
    }
    else if ( err == EZMODE_ERR_CANCELLED )
    {
      pStr = "EZMode: Cancel";
    }
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    else if ( err == EZMODE_ERR_BAD_PARAMETER )
    {
      pStr = "EZMode: BadParm";
    }
    else if ( err == EZMODE_ERR_TIMEDOUT )
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giHeatingCoolingUnitScreenMode == HEATCOOLUNIT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif  // LCD_SUPPORTED

    // show main UI screen 3 seconds after joining network
    osal_start_timerEx( zclSampleHeatingCoolingUnit_TaskID, SAMPLEHEATINGCOOLINGUNIT_MAIN_SCREEN_EVT, 3000 );
  }

}

#endif // ZCL_EZMODE

/****************************************************************************
****************************************************************************/


