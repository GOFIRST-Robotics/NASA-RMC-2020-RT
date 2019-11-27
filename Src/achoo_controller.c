//
// Created by nick on 11/5/19.
//

#include "achoo_controller.h"
#include <FreeRTOS.h>
#include <can_manager.h>
#include <math.h>
#include <rt_conf.h>
#include <task.h>
#include "VESC.h"
#include "stdlib.h"

KneelState targetState = KNEELING;
KneelState currentState = KNEELING;
VESC* leftMotor;
VESC* rightMotor;

void achooCANCallback(rmc_can_msg msg) {
  // Because of the mask we only get messages that have our ID
  U32 msg_type = msg.id >> 8u;
  if (msg_type == ACHOO_MSG_SET_KNEEL && msg.length >= 1) {
    U8 param = msg.buf[0];
    switch (param) {
      case 1:
        targetState = KNEELING;
        break;
      default:
      case 0:
        targetState = STANDING;
    }
  }
}

void achooControllerFunc(void const* argument) {
  // System setup
  registerCANMsgHandler(ACHOO_SYS_ID, &achooCANCallback);
  leftMotor = create_vesc(ACHOO_MOTOR_L, ACHOO_MOTOR_POLE_PAIRS);
  rightMotor = create_vesc(ACHOO_MOTOR_R, ACHOO_MOTOR_POLE_PAIRS);

  TickType_t lastWakeTime;
  while (1) {
    vTaskDelayUntil(&lastWakeTime, ACHOO_LOOP_MS * portTICK_RATE_MS);
    // Handle target state transitions
    if (targetState == KNEELING &&
        (currentState == STANDING || currentState == MOVING_STAND)) {
      currentState = MOVING_KNEEL;
    } else if (targetState == STANDING &&
               (currentState == KNEELING || currentState == MOVING_KNEEL)) {
      currentState = MOVING_STAND;
    }
    // Check if our movement has completed

    // Set VESC movement
    float current = 0;
    if (currentState == MOVING_STAND) {
      current = 10;
    } else if (currentState == MOVING_KNEEL) {
      current = -10;
    }
    vesc_set_current(leftMotor, current);
    vesc_set_current(rightMotor, current);

    // Send status message
    U8 data[1];
    data[0] = currentState;
    do_send_can_message((ACHOO_MSG_STATUS << 8u) | ACHOO_SYS_ID, data, 1);
  }
}

KneelState getACHOOState() { return currentState; }
