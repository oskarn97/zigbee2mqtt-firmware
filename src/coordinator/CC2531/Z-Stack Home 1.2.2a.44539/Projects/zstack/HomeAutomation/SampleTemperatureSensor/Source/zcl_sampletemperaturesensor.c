/**************************************************************************************************
  Filename:       zcl_sampletemperaturesensor.c
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
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
  This device will act as a temperature sensor. It updates the current
  temperature on the thermostat when the user sends the desired
  temperature using SW1.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Send current temperature
    - SW2: Invoke EZMode
    - SW3: Adjust temperature
    - SW5: Go to Help screen

  Temperature:
    - SW1: Increase temperature
    - SW3: Decrease temperature
    - SW5: Enter temperature
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

#include "zcl_sampletemperaturesensor.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"


/*********************************************************************
 * MACROS
 */

// how often to report temperature
#define SAMPLETEMPERATURESENSOR_REPORT_INTERVAL   10000

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSampleTemperatureSensor_TaskID;

uint8 zclSampleTemperatureSensorSeqNum;

static byte gPermitDuration = 0x00;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleTemperatureSensor_DstAddr;

#ifdef ZCL_EZMODE
static void zclSampleTemperatureSensor_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleTemperatureSensor_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclSampleTemperatureSensor_RegisterEZModeData =
{
  &zclSampleTemperatureSensor_TaskID,
  SAMPLETEMPERATURESENSOR_EZMODE_NEXTSTATE_EVT,
  SAMPLETEMPERATURESENSOR_EZMODE_TIMEOUT_EVT,
  &zclSampleTemperatureSensorSeqNum,
  zclSampleTemperatureSensor_EZModeCB
};

// NOT ZCL_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT
};
#define ZCLSAMPLETEMPERATURESENSOR_BINDINGLIST        1
#endif

devStates_t zclSampleTemperatureSensor_NwkState = DEV_INIT;

uint8 giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;   // display main screen mode first

static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleTemperatureSensor_TestEp =
{
  20,                                 // Test endpoint
  &zclSampleTemperatureSensor_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleTemperatureSensor_HandleKeys( byte shift, byte keys );
static void zclSampleTemperatureSensor_BasicResetCB( void );
static void zclSampleTemperatureSensor_IdentifyCB( zclIdentify_t *pCmd );
static void zclSampleTemperatureSensor_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclSampleTemperatureSensor_ProcessIdentifyTimeChange( void );

// app display functions
void zclSampleTemperatureSensor_LcdDisplayUpdate(void);
void zclSampleTemperatureSensor_LcdDisplayMainMode(void);
void zclSampleTemperatureSensor_LcdDisplayTempMode(void);
void zclSampleTemperatureSensor_LcdDisplayHelpMode(void);

static void zclSampleTemperatureSensor_SendTemp(void);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleTemperatureSensor_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleTemperatureSensor_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleTemperatureSensor_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleTemperatureSensor_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleTemperatureSensor_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleTemperatureSensor_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleTemperatureSensor_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif // ZCL_DISCOVER

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]    = " ";
const char sDeviceName[]   = "  Temp Sensor";
const char sSwTempUp[]     = "SW1: Raise Temp";
const char sSwEZMode[]     = "SW2: EZ-Mode";
const char sSwTempDown[]   = "SW3: Lower Temp";
const char sSwHelp[]       = "SW5: Help";
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleTemperatureSensor_CmdCallbacks =
{
  zclSampleTemperatureSensor_BasicResetCB,        // Basic Cluster Reset command
  zclSampleTemperatureSensor_IdentifyCB,          // Identify command
#ifdef ZCL_EZMODE
  NULL,                                           // Identify EZ-Mode Invoke command
  NULL,                                           // Identify Update Commission State command
#endif
  NULL,                                           // Identify Trigger Effect command
  zclSampleTemperatureSensor_IdentifyQueryRspCB,  // Identify Query Response command
  NULL,             				                      // On/Off cluster command
  NULL,                                           // On/Off cluster enhanced command Off with Effect
  NULL,                                           // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                           // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                           // Level Control Move to Level command
  NULL,                                           // Level Control Move command
  NULL,                                           // Level Control Step command
  NULL,                                           // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                           // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                           // Scene Store Request command
  NULL,                                           // Scene Recall Request command
  NULL,                                           // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                           // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                           // Get Event Log command
  NULL,                                           // Publish Event Log command
#endif
  NULL,                                           // RSSI Location command
  NULL                                            // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclSampleTemperatureSensor_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleTemperatureSensor_Init( byte task_id )
{
  zclSampleTemperatureSensor_TaskID = task_id;

  // Set destination address to indirect
  zclSampleTemperatureSensor_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleTemperatureSensor_DstAddr.endPoint = 0;
  zclSampleTemperatureSensor_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleTemperatureSensor_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLETEMPERATURESENSOR_ENDPOINT, &zclSampleTemperatureSensor_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( SAMPLETEMPERATURESENSOR_ENDPOINT, SAMPLETEMPERATURESENSOR_MAX_ATTRIBUTES, zclSampleTemperatureSensor_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleTemperatureSensor_TaskID );

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleTemperatureSensor_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleTemperatureSensor_TaskID );

  // Register for a test endpoint
  afRegister( &sampleTemperatureSensor_TestEp );

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
uint16 zclSampleTemperatureSensor_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleTemperatureSensor_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclSampleTemperatureSensor_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif

        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleTemperatureSensor_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSampleTemperatureSensor_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleTemperatureSensor_NwkState = (devStates_t)(MSGpkt->hdr.status);


          // now on the network
          if ( (zclSampleTemperatureSensor_NwkState == DEV_ZB_COORD) ||
               (zclSampleTemperatureSensor_NwkState == DEV_ROUTER)   ||
               (zclSampleTemperatureSensor_NwkState == DEV_END_DEVICE) )
          {
#ifndef HOLD_AUTO_START
            giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;
            zclSampleTemperatureSensor_LcdDisplayUpdate();
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

  if ( events & SAMPLETEMPERATURESENSOR_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSampleTemperatureSensor_IdentifyTime > 0 )
      zclSampleTemperatureSensor_IdentifyTime--;
    zclSampleTemperatureSensor_ProcessIdentifyTimeChange();

    return ( events ^ SAMPLETEMPERATURESENSOR_IDENTIFY_TIMEOUT_EVT );
  }

#ifdef ZCL_EZMODE
  // going on to next state
  if ( events & SAMPLETEMPERATURESENSOR_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SAMPLETEMPERATURESENSOR_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & SAMPLETEMPERATURESENSOR_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SAMPLETEMPERATURESENSOR_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE

  if ( events & SAMPLETEMPERATURESENSOR_MAIN_SCREEN_EVT )
  {
    giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;
    zclSampleTemperatureSensor_LcdDisplayUpdate();

    return ( events ^ SAMPLETEMPERATURESENSOR_MAIN_SCREEN_EVT );
  }

  if ( events & SAMPLETEMPERATURESENSOR_TEMP_SEND_EVT )
  {
    zclSampleTemperatureSensor_SendTemp();

    // report current temperature reading every 10 seconds
    osal_start_timerEx( zclSampleTemperatureSensor_TaskID, SAMPLETEMPERATURESENSOR_TEMP_SEND_EVT, SAMPLETEMPERATURESENSOR_REPORT_INTERVAL );

    return ( events ^ SAMPLETEMPERATURESENSOR_TEMP_SEND_EVT );
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_HandleKeys
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
static void zclSampleTemperatureSensor_HandleKeys( byte shift, byte keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
    // increase temperature
    giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;

    if ( zclSampleTemperatureSensor_MeasuredValue < zclSampleTemperatureSensor_MaxMeasuredValue )
    {
      zclSampleTemperatureSensor_MeasuredValue = zclSampleTemperatureSensor_MeasuredValue + 100;  // considering using whole number value
    }
    else if ( zclSampleTemperatureSensor_MeasuredValue >= zclSampleTemperatureSensor_MaxMeasuredValue )
    {
      zclSampleTemperatureSensor_MeasuredValue = zclSampleTemperatureSensor_MaxMeasuredValue;
    }

    // Send temperature information
    zclSampleTemperatureSensor_SendTemp();
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    if ( ( giTemperatureSensorScreenMode == TEMPSENSE_MAINMODE ) ||
        ( giTemperatureSensorScreenMode == TEMPSENSE_HELPMODE ) )
    {
      giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;

#ifdef ZCL_EZMODE
      zclEZMode_InvokeData_t ezModeData;
      static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT };   // only bind on the Temperature Measurement cluster

      // Invoke EZ-Mode
      ezModeData.endpoint = SAMPLETEMPERATURESENSOR_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( ( zclSampleTemperatureSensor_NwkState == DEV_ZB_COORD ) ||
           ( zclSampleTemperatureSensor_NwkState == DEV_ROUTER )   ||
           ( zclSampleTemperatureSensor_NwkState == DEV_END_DEVICE ) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = TRUE;        // Temperature Sensor is an initiator
      ezModeData.numActiveInClusters = 1;
      ezModeData.pActiveInClusterIDs = clusterIDs;
      ezModeData.numActiveOutClusters = 0;   // active output cluster
      ezModeData.pActiveOutClusterIDs = NULL;
      zcl_InvokeEZMode( &ezModeData );

#ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

      // NOT ZCL_EZMODE, Use EndDeviceBind
#else
      {
        zAddrType_t dstAddr;
        dstAddr.addrMode = Addr16Bit;
        dstAddr.addr.shortAddr = 0;   // Coordinator makes the EDB match

        // Initiate an End Device Bind Request, this bind request will
        // only use a cluster list that is important to binding.
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
        ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                              SAMPLETEMPERATURESENSOR_ENDPOINT,
                              ZCL_HA_PROFILE_ID,
                              0, NULL,
                              ZCLSAMPLETEMPERATURESENSOR_BINDINGLIST, bindingOutClusters,
                              FALSE );
      }
#endif // ZCL_EZMODE
    }
  }

  if ( keys & HAL_KEY_SW_3 )
  {
    giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;

    // decrease the temperature
    if ( zclSampleTemperatureSensor_MeasuredValue > zclSampleTemperatureSensor_MinMeasuredValue )
    {
      zclSampleTemperatureSensor_MeasuredValue = zclSampleTemperatureSensor_MeasuredValue - 100;  // considering using whole number value
    }
    else if ( zclSampleTemperatureSensor_MeasuredValue >= zclSampleTemperatureSensor_MinMeasuredValue )
    {
      zclSampleTemperatureSensor_MeasuredValue = zclSampleTemperatureSensor_MinMeasuredValue;
    }

    // Send temperature information
    zclSampleTemperatureSensor_SendTemp();
  }

  if ( keys & HAL_KEY_SW_4 )
  {
    giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;

    if ( ( zclSampleTemperatureSensor_NwkState == DEV_ZB_COORD ) ||
         ( zclSampleTemperatureSensor_NwkState == DEV_ROUTER ) )
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
    zclSampleTemperatureSensor_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    if ( giTemperatureSensorScreenMode == TEMPSENSE_MAINMODE )
    {
      giTemperatureSensorScreenMode = TEMPSENSE_HELPMODE;
    }
    else if ( giTemperatureSensorScreenMode == TEMPSENSE_HELPMODE )
    {
#ifdef LCD_SUPPORTED
      HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
#endif
      giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;
    }
  }

  // update display
  zclSampleTemperatureSensor_LcdDisplayUpdate();
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleTemperatureSensor_LcdDisplayUpdate( void )
{
  // turn on red LED for temperatures >= 24.00C
  if ( zclSampleTemperatureSensor_MeasuredValue >= 2400 )
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
  }
  // turn on green LED for temperatures <= 20.00C
  else if ( zclSampleTemperatureSensor_MeasuredValue <= 2000 )
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
  }
  // turn on both red and green LEDs for temperatures between 20.00C and 24.00C
  else
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
  }

  if ( giTemperatureSensorScreenMode == TEMPSENSE_HELPMODE )
  {
    zclSampleTemperatureSensor_LcdDisplayHelpMode();
  }
  else
  {
    zclSampleTemperatureSensor_LcdDisplayMainMode();
  }
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleTemperatureSensor_LcdDisplayMainMode( void )
{
  char sDisplayTemp[16];

  if ( zclSampleTemperatureSensor_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( 0 );
  }
  else if ( zclSampleTemperatureSensor_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( 1 );
  }
  else if ( zclSampleTemperatureSensor_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1( 2 );
  }

  // display current temperature
  osal_memcpy(sDisplayTemp, "TEMP: ", 6);
  _ltoa( ( zclSampleTemperatureSensor_MeasuredValue / 100 ), (void *)(&sDisplayTemp[6]), 10 );   // convert temperature to whole number
  osal_memcpy( &sDisplayTemp[8], "C", 2 );
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sDisplayTemp, HAL_LCD_LINE_2 );
#endif

#ifdef LCD_SUPPORTED
  if ( ( zclSampleTemperatureSensor_NwkState == DEV_ZB_COORD ) ||
       ( zclSampleTemperatureSensor_NwkState == DEV_ROUTER ) )
  {
    // display help key with permit join status
    if ( gPermitDuration )
    {
      HalLcdWriteString( "SW5: Help      *", HAL_LCD_LINE_3 );
    }
    else
    {
      HalLcdWriteString( "SW5: Help       ", HAL_LCD_LINE_3 );
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
 * @fn      zclSampleTemperatureSensor_LcdDisplayHelpMode
 *
 * @brief   Called to display the SW options on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleTemperatureSensor_LcdDisplayHelpMode( void )
{
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sSwTempUp, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwTempDown, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_SendTemp
 *
 * @brief   Called to send current temperature information to the thermostat
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleTemperatureSensor_SendTemp( void )
{
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_TEMPERATURE_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&zclSampleTemperatureSensor_MeasuredValue);

    zcl_SendReportCmd( SAMPLETEMPERATURESENSOR_ENDPOINT, &zclSampleTemperatureSensor_DstAddr,
                       ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSampleTemperatureSensorSeqNum++ );
  }

  osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleTemperatureSensor_ProcessIdentifyTimeChange( void )
{
  if ( zclSampleTemperatureSensor_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleTemperatureSensor_TaskID, SAMPLETEMPERATURESENSOR_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSampleTemperatureSensor_OnOff )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    }
    else
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    }

    osal_stop_timerEx( zclSampleTemperatureSensor_TaskID, SAMPLETEMPERATURESENSOR_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleTemperatureSensor_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network

  // restart device
  MT_SysCommandProcessing( aProcessCmd );
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclSampleTemperatureSensor_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSampleTemperatureSensor_IdentifyTime = pCmd->identifyTime;
  zclSampleTemperatureSensor_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclSampleTemperatureSensor_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp )
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

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleTemperatureSensor_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleTemperatureSensor_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleTemperatureSensor_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleTemperatureSensor_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleTemperatureSensor_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleTemperatureSensor_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleTemperatureSensor_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclSampleTemperatureSensor_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleTemperatureSensor_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleTemperatureSensor_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleTemperatureSensor_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleTemperatureSensor_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleTemperatureSensor_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
  {
    osal_mem_free( pInMsg->attrCmd );
  }
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleTemperatureSensor_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleTemperatureSensor_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < readRspCmd->numAttr; i++ )
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleTemperatureSensor_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleTemperatureSensor_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleTemperatureSensor_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleTemperatureSensor_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleTemperatureSensor_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleTemperatureSensor_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleTemperatureSensor_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER

#ifdef ZCL_EZMODE

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSampleTemperatureSensor_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
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
 * @fn      zclSampleTemperatureSensor_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclSampleTemperatureSensor_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char szLine[20];
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
    zclSampleTemperatureSensor_IdentifyTime = ( EZMODE_TIME / 1000 );  // convert to seconds
    zclSampleTemperatureSensor_ProcessIdentifyTimeChange();
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
      if ( giTemperatureSensorScreenMode == TEMPSENSE_MAINMODE )
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
    zclSampleTemperatureSensor_IdentifyTime = 0;
    zclSampleTemperatureSensor_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // "EZDst:1234 EP:34"
      osal_memcpy( szLine, "EZDst:", 6 );
      zclHA_uint16toa( pData->sFinish.nwkaddr, &szLine[6] );
      osal_memcpy( &szLine[10], " EP:", 4 );
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
      if ( giTemperatureSensorScreenMode == TEMPSENSE_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif  // LCD_SUPPORTED

    // show main UI screen 3 seconds after joining network
    osal_start_timerEx( zclSampleTemperatureSensor_TaskID, SAMPLETEMPERATURESENSOR_MAIN_SCREEN_EVT, 3000 );

    // report current temperature reading 15 seconds after joinging the network
    osal_start_timerEx( zclSampleTemperatureSensor_TaskID, SAMPLETEMPERATURESENSOR_TEMP_SEND_EVT, SAMPLETEMPERATURESENSOR_REPORT_INTERVAL );
  }
}
#endif // ZCL_EZMODE

/****************************************************************************
****************************************************************************/


