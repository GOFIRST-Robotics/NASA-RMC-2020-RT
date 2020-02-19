#ifndef NASA_RMC_RT_BLESSYOU_CONTROLLER_H
#define NASA_RMC_RT_BLESSYOU_CONTROLLER_H

#include "can_manager.h"

typedef enum { SET_SPEED = 0, SET_POSITION } ElevatorState;

#define BLESSYOU_MOTOR_POLE_PAIRS 7
#define BLESSYOU_MSG_STATUS 70u
#define BLESSYOU_MSG_SET_SPEED 71u
#define BLESSYOU_MSG_SET_POSITION 72u

#define BLESSYOU_LOOP_MS 20u

#define BLESSYOU_RPM_TO_MMS 0.001f
#define BLESSYOU_ADC_TO_MM 0.5f

#define BLESSYOU_POS_P 10.0f        // mm/s per mm err
#define BLESSYOU_POS_DEADZONE 1.0f  // +/- mm deadzone
#define BLESSYOU_POS_MINSPEED 1.0f  // mm/s min speed

#define BLESSYOU_POS_NONHOME_MIN \
  20u  // Minimum position we can be at if SNEEZE isn't homed

#define BLESSYOU_LEVEL_P 1.0f  // mm/s per side per mm error

void blessyouCANCallback(rmc_can_msg msg);

S8 getBLESSYOUPosition();
S8 getBLESSYOUSpeed();

#endif  // NASA_RMC_RT_BLESSYOU_CONTROLLER_H
