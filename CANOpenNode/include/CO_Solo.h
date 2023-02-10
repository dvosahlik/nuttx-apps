/*
 * CO_Solo.h
 *
 *  Created on: Feb 3, 2023
 *      Author: david
 */

#ifndef CANOPENNODE_INCLUDE_CO_SOLO_H_
#define CANOPENNODE_INCLUDE_CO_SOLO_H_

#define CO_ERROR                                0x1001
#define CO_ERROR_GENERIC                        0x0
#define CO_ERROR_CURRENT                        0x1
#define CO_ERROR_VOLTAGE                        0x2
#define CO_ERROR_TEMP                           0x3
#define CO_ERROR_COMMUNICATION                  0x4
#define CO_ERROR_DEVICE                         0x5
#define CO_ERROR_MAUFACTURER                    0x7

#define CO_GUARD_TIME                           0x100C
#define CO_LIFETIME_FACTOR                      0x100D
#define CO_HEARTBEAT_PRODUCER                   0x1017

#define CO_SOLO_SET_DEVICE_ADDRESS              0x3001

#define CO_SOLO_COMMANDING_MODE                 0x3002
#define COMMANDING_MODE_ANALOGUE                0x0
#define COMMANDING_MODE_DIGITAL                 0x1

#define CO_SOLO_CURRENT_LIMIT                   0x3003
#define CO_SOLO_TORQUE_REFERENCE                0x3004
#define CO_SOLO_SPEED_REFERENCE                 0x3005
#define CO_SOLO_POWER_REFERENCE                 0x3006
#define CO_SOLO_MOTOR_IDENTIFICATION            0x3007
#define CO_SOLO_EMERGENCY_STOP                  0x3008
#define CO_SOLO_OUTPUT_PWM_FREQ                 0x3009
#define CO_SOLO_SPEED_KP_GAIN                   0x300A
#define CO_SOLO_SPEED_KI_GAIN                   0x300B

#define CO_SOLO_ROTATION_DIRECTION              0x300C
#define ROTATION_DIRECTION_CCW                  0x0
#define ROTATION_DIRECTION_CW                   0x1

#define CO_SOLO_PHASE_RESISTANCE                0x300D
#define CO_SOLO_PHASE_INDUCTANCE                0x300E
#define CO_SOLO_POLES_NR                        0x300F
#define CO_SOLO_ENCODER_LINES                   0x3010
#define CO_SOLO_SPEED_LIMIT                     0x3011

#define CO_SOLO_FB_MODE                         0x3013
#define FB_MODE_SENSORLESS                      0x0
#define FB_MODE_ENCODER                         0x1
#define FB_MODE_HALL                            0x2

#define CO_SOLO_RESET_FACTORY                   0x3014

#define CO_SOLO_MOTOR_TYPE                      0x3015
#define MOTOR_TYPE_DC                           0x0
#define MOTOR_TYPE_BLDC                         0x1
#define MOTOR_TYPE_ACIM                         0x2
#define MOTOR_TYPE_FAST_BLDC                    0x3

#define CO_SOLO_CTRL_MODE_TYPE                  0x3016
#define CTRL_MODE_TYPE_SPEED                    0x0
#define CTRL_MODE_TYPE_TORQUE                   0x1
#define CTRL_MODE_TYPE_POSITION                 0x2

#define CO_SOLO_CURRENT_KP_GAIN                 0x3017
#define CO_SOLO_CURRENT_KI_GAIN                 0x3018
#define CO_SOLO_MAGNETIZING_CURRENT_REF         0x301A

#define CO_SOLO_POSITION_REF                    0x301B
#define CO_SOLO_POSITION_KP_GAIN                0x301C
#define CO_SOLO_POSITION_KI_GAIN                0x301D
#define CO_SOLO_RESET_POSITION                  0x301F

#define CO_SOLO_DEVICE_ERROR                    0x3020
#define DEVICE_ERROR_OVERCURRENT                0x0
#define DEVICE_ERROR_OVERVOLTAGE                0x1
#define DEVICE_ERROR_OVERTEMPERATURE            0x2
#define DEVICE_ERROR_ENCODER_CAL_TIMEOUT        0x3
#define DEVICE_ERROR_HALL_CAL_TIMEOUT           0x4
#define DEVICE_ERROR_CAN_LOST                   0x5

#define CO_SOLO_SENSORLESS_OBS_GAIN_BLDC        0x3021
#define CO_SOLO_SENSORLESS_OBS_GAIN_FAST_BLDC   0x3022
#define CO_SOLO_SENSORLESS_OBS_GAIN_DC          0x3023
#define CO_SOLO_SENSORLESS_OBS_FIL_GAIN_BLDC    0x3024
#define CO_SOLO_SENSORLESS_OBS_FIL_GAIN_FAST_BLDC    0x3025

#define CO_SOLO_UART_BAUD_RATE                  0x3026
#define UART_BAUD_RATE_937500                   0x0
#define UART_BAUD_RATE_115200                   0x1

#define CO_SOLO_SENS_CAL                        0x3027
#define SENS_CAL_STOP                           0x0
#define SENS_CAL_START_ENCODER                  0x1
#define SENS_CAL_START_HALL                     0x2

#define CO_SOLO_SENSOR_CCW_OFFSET               0x3028
#define CO_SOLO_SENSOR_CW_OFFSET                0x3029
#define CO_SOLO_SPEED_ACCEL                     0x302A
#define CO_SOLO_SPEED_DECEL                     0x302B

#define CO_SOLO_CAN_BAUD_RATE                   0x302C
#define CAN_BAUD_RATE_1000                      0x0
#define CAN_BAUD_RATE_500                       0x1
#define CAN_BAUD_RATE_250                       0x2
#define CAN_BAUD_RATE_125                       0x3
#define CAN_BAUD_RATE_100                       0x4

#define CO_SOLO_PHASE_A_VOLT                    0x302D
#define CO_SOLO_PHASE_B_VOLT                    0x302E
#define CO_SOLO_PHASE_A_CURRENT                 0x302F
#define CO_SOLO_PHASE_B_CURRENT                 0x3030
#define CO_SOLO_BUS_VOLT                        0x3031
#define CO_SOLO_DC_CURRENT                      0x3032
#define CO_SOLO_DC_VOLT                         0x3033
#define CO_SOLO_IQ_CURRENT                      0x3034
#define CO_SOLO_ID_CURRENT                      0x3035
#define CO_SOLO_SPEED                           0x3036
#define CO_SOLO_POSITION                        0x3037
#define CO_SOLO_MOTOR_ANGLE                     0x3038
#define CO_SOLO_BOARD_TEMP                      0x3039
#define CO_SOLO_FW_VER                          0x303A
#define CO_SOLO_HW_VER                          0x303B
#define CO_SOLO_ENCODER_INDEX_COUNT             0x303D

#endif /* CANOPENNODE_INCLUDE_CO_SOLO_H_ */
