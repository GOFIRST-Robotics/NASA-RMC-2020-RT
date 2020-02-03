#ifndef NASA_RMC_RT_TISSUE_CONTROLLER_H
#define NASA_RMC_RT_TISSUE_CONTROLLER_H

#include "can_manager.h"

#define TISSUE_MOTOR_POLE_PAIRS 7
#define TISSUE_MSG_STATUS 60u
#define TISSUE_MSG_SET_DIG_SPEED 61u
#define TISSUE_MSG_GO_HOME 62u
#define TISSUE_MSG_SET_TRANS_SPEED 63u

#define TISSUE_LOOP_MS 20u

typedef enum { IDLE = 0, DIGGING, HOMING } TissueState;

void tissueCANCallback(rmc_can_msg msg);

bool isTISSUEHome();

#endif  // NASA_RMC_RT_TISSUE_CONTROLLER_H
