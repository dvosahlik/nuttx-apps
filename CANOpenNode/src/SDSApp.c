/*
 * SDSApp.c
 *
 *  Created on: Feb 2, 2023
 *      Author: david
 */

#include <stdlib.h>

#include "CANopen.h"
#include "CO_application.h"
#include "CO_Solo.h"



#define SDS_SOLO_NODE_ID                1
#define SDS_SOLO_HEARTBEAT_RATE_MS      200
#define SDS_SOLO_RPM_LIM                200


typedef struct {
  uint8_t initialized;
} CO_SDS_t;

CO_SDS_t CO_info;

/**
 * @defgroup CO_SDOclient SDO client
 * CANopen Service Data Object - client protocol.
 *
 * @ingroup CO_CANopen_301
 * @{
 * SDO client is able to access Object Dictionary variables from remote nodes.
 * Usually there is one SDO client on CANopen network, which is able to
 * configure other CANopen nodes. It is also possible to establish individual
 * SDO client-server communication channels between devices.
 *
 * SDO client is used in CANopenNode from CO_gateway_ascii.c with default SDO
 * CAN identifiers. There is quite advanced usage in non-blocking function.
 *
 * If enabled, SDO client is initialized in CANopen.c file with
 * @ref CO_SDOclient_init() function.
 *
 * Basic usage:
 */
CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                            uint16_t index, uint8_t subIndex,
                            uint8_t *buf, size_t bufSize, size_t *readSize)
{
  CO_SDO_return_t SDO_ret;

  // setup client (this can be skipped, if remote device don't change)
  SDO_ret = CO_SDOclient_setup(SDO_C,
                               CO_CAN_ID_SDO_CLI + nodeId,
                               CO_CAN_ID_SDO_SRV + nodeId,
                               nodeId);
  if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
      return CO_SDO_AB_GENERAL;
  }

  // initiate upload
  SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 100, false);
  if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
      return CO_SDO_AB_GENERAL;
  }

  // upload data
  do {
      uint32_t timeDifference_us = 1000;
      CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

      SDO_ret = CO_SDOclientUpload(SDO_C,
                                   timeDifference_us,
                                   false,
                                   &abortCode,
                                   NULL, NULL, NULL);
      if (SDO_ret < 0) {
          return abortCode;
      }

      usleep(timeDifference_us);
  } while(SDO_ret > 0);

  // copy data to the user buffer (for long data function must be called
  // several times inside the loop)
  *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

  return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                             uint16_t index, uint8_t subIndex,
                             uint8_t *data, size_t dataSize)
{
  CO_SDO_return_t SDO_ret;
  bool_t bufferPartial = false;

  // setup client (this can be skipped, if remote device is the same)
  SDO_ret = CO_SDOclient_setup(SDO_C,
                               CO_CAN_ID_SDO_CLI + nodeId,
                               CO_CAN_ID_SDO_SRV + nodeId,
                               nodeId);
  if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
      return -1;
  }

  // initiate download
  SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex,
                                         0, 100, false);
  if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
      return -1;
  }

  // fill data
  size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
  if (nWritten < dataSize) {
      bufferPartial = true;
      // If SDO Fifo buffer is too small, data can be refilled in the loop.
  }

  //download data

  unsigned int timeDifference_us = 1000;
  CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
      SDO_ret = CO_SDOclientDownload(SDO_C,
                                     timeDifference_us,
                                     false,
                                     bufferPartial,
                                     &abortCode,
                                     NULL, NULL);
      if (SDO_ret < 0) {
          return abortCode;
      }

//      usleep(timeDifference_us);


  return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t write_SOLO(CO_t* CO, uint16_t index, uint8_t subIndex,
                              uint32_t data)
{
  CO_SDOclient_t *SDO_C = CO->SDOclient;
  return write_SDO(SDO_C, SDS_SOLO_NODE_ID,
                   index, subIndex,
                   (uint8_t*)&data, 4);

}

/**
 * @defgroup CO_applicationLinux Application interface to CANopenNode
 * Application interface, similar to Arduino, extended to CANopen and
 * additional, realtime thread.
 *
 * @ingroup CO_socketCAN
 * @{
 */

/**
 * Function is called once on the program startup, after Object dictionary
 * initialization and before CANopen initialization.
 *
 * @param [in,out] bitRate Stored CAN bit rate, can be overridden.
 * @param [in,out] nodeId Stored CANopen NodeId, can be overridden.
 * @param [out] errInfo Variable may indicate error information - index of
 * erroneous OD entry.
 *
 * @return @ref CO_ReturnError_t CO_ERROR_NO in case of success.
 */
CO_ReturnError_t app_programStart(uint16_t *bitRate,
                                  uint8_t *nodeId,
                                  uint32_t *errInfo)
{
  *nodeId = 2;
  return CO_ERROR_NO;
}


/**
 * Function is called after CANopen communication reset.
 *
 * @param co CANopen object.
 */
void app_communicationReset(CO_t *co)
{
  CO_info.initialized = 0;
}


/**
 * Function is called just before program ends.
 */
void app_programEnd()
{

}


/**
 * Function is called cyclically from main().
 *
 * Place for the slower code (all must be non-blocking).
 *
 * @warning
 * Mind race conditions between this functions and app_programRt(), which
 * run from the realtime thread. If accessing Object dictionary variable which
 * is also mappable to PDO, it is necessary to use CO_LOCK_OD() and
 * CO_UNLOCK_OD() macros from @ref CO_critical_sections.
 *
 * @param co CANopen object.
 * @param timer1usDiff Time difference since last call in microseconds
 */
void app_programAsync(CO_t *co, uint32_t timer1usDiff)
{
  /* Initialization */
  if (CO_info.initialized == 0)
    {
      /* Setup Solo */

      write_SOLO(co, CO_HEARTBEAT_PRODUCER, 0, SDS_SOLO_HEARTBEAT_RATE_MS);
      write_SOLO(co, CO_SOLO_COMMANDING_MODE, 0, COMMANDING_MODE_DIGITAL);
      write_SOLO(co, CO_SOLO_TORQUE_REFERENCE, 0, 0);
      write_SOLO(co, CO_SOLO_SPEED_REFERENCE, 0, 0);
      write_SOLO(co, CO_SOLO_POWER_REFERENCE, 0, 0);
      write_SOLO(co, CO_SOLO_ROTATION_DIRECTION, 0, ROTATION_DIRECTION_CCW);
      write_SOLO(co, CO_SOLO_SPEED_LIMIT, 0, SDS_SOLO_RPM_LIM);
      write_SOLO(co, CO_SOLO_FB_MODE, 0, FB_MODE_HALL);
      write_SOLO(co, CO_SOLO_MOTOR_TYPE, 0, MOTOR_TYPE_BLDC);
      write_SOLO(co, CO_SOLO_CTRL_MODE_TYPE, 0, CTRL_MODE_TYPE_TORQUE);
      write_SOLO(co, CO_SOLO_MAGNETIZING_CURRENT_REF, 0, 0);
      write_SOLO(co, CO_SOLO_POSITION_REF, 0, 0);

      /* Start the SOLO */

      CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, SDS_SOLO_NODE_ID);
      CO_info.initialized = 1;
    }
  float tqRef = 2;
  float current = 0;
  int speed = 0;
  size_t read;
  write_SOLO(co, CO_SOLO_TORQUE_REFERENCE, 0, (uint32_t)tqRef);
  usleep(2000);

  read_SDO(co->SDOclient, SDS_SOLO_NODE_ID,
           CO_SOLO_ID_CURRENT, 0,
                              (uint8_t *)&current, sizeof(float), &read);
  usleep(2000);

  read_SDO(co->SDOclient, SDS_SOLO_NODE_ID,
             CO_SOLO_SPEED, 0,
                                (uint8_t *)&speed, sizeof(speed), &read);

  usleep(2000);
}


/**
 * Function is called cyclically from realtime thread at constant intervals.
 *
 * Code inside this function must be executed fast. Take care on race conditions
 * with app_programAsync.
 *
 * @param co CANopen object.
 * @param timer1usDiff Time difference since last call in microseconds
 */
void app_programRt(CO_t *co, uint32_t timer1usDiff)
{

}
