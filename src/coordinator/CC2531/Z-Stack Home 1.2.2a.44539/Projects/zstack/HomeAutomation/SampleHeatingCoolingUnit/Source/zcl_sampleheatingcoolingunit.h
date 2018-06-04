/**************************************************************************************************
  Filename:       zcl_sampleheatingcoolingunit.h
  Revised:        $Date: 2013-04-22 14:49:05 -0700 (Mon, 22 Apr 2013) $
  Revision:       $Revision: 33994 $

  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef ZCL_SAMPLEHEATINGCOOLINGUNIT_H
#define ZCL_SAMPLEHEATINGCOOLINGUNIT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"

/*********************************************************************
 * CONSTANTS
 */
#define SAMPLEHEATINGCOOLINGUNIT_ENDPOINT            8

#define SAMPLEHEATINGCOOLINGUNIT_MAX_ATTRIBUTES      15

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Application Events
#define SAMPLEHEATINGCOOLINGUNIT_IDENTIFY_TIMEOUT_EVT         0x0001
#define SAMPLEHEATINGCOOLINGUNIT_POLL_CONTROL_TIMEOUT_EVT     0x0002
#define SAMPLEHEATINGCOOLINGUNIT_EZMODE_NEXTSTATE_EVT         0x0004
#define SAMPLEHEATINGCOOLINGUNIT_EZMODE_TIMEOUT_EVT           0x0008
#define SAMPLEHEATINGCOOLINGUNIT_MAIN_SCREEN_EVT              0x0010

// Application Display Modes
#define HEATCOOLUNIT_MAINMODE         0x00
#define HEATCOOLUNIT_HELPMODE         0x01

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclSampleHeatingCoolingUnit_SimpleDesc;

extern CONST zclAttrRec_t zclSampleHeatingCoolingUnit_Attrs[];

extern uint8  zclSampleHeatingCoolingUnit_OnOff;

extern uint16 zclSampleHeatingCoolingUnit_IdentifyTime;

extern uint16 zclSampleHeatingCoolingUnit_RunningState;

extern int16 zclSampleHeatingCoolingUnit_LocalTemperature;

extern int16 zclSampleHeatingCoolingUnit_OccupiedCoolingSetpoint;

extern int16 zclSampleHeatingCoolingUnit_OccupiedHeatingSetpoint;

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclSampleHeatingCoolingUnit_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSampleHeatingCoolingUnit_event_loop( byte task_id, UINT16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_SAMPLEHEATINGCOOLINGUNIT_H */
