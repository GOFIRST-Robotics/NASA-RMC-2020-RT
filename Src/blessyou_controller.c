#include "blessyou_controller.h"
#include <FreeRTOS.h>
#include <VESC.h>
#include <main.h>
#include <math.h>
#include <portmacro.h>
#include <rt_conf.h>
#include <sneeze_controller.h>
#include <stm32f3xx_hal.h>
#include <task.h>

extern ADC_HandleTypeDef hadc1;

static VESC* leftMotor;
static VESC* rightMotor;
static S8 targetSpeed = 0;
static S8 targetPos = 0;
static S8 currentSpeed = 0;
static S8 currentPos = 0;
static ElevatorState state = SET_POSITION;
static U32 adcVal[2];

void blessyouCANCallback(rmc_can_msg msg) {
  // Because of the mask we only get messages that have our ID
  U32 msg_type = msg.id >> 8u;
  if (msg_type == BLESSYOU_MSG_SET_SPEED && msg.length >= 1) {
    targetSpeed = (S8)msg.buf[0];
    state = SET_SPEED;
  } else if (msg_type == BLESSYOU_MSG_SET_POSITION && msg.length >= 1) {
    targetPos = (S8)msg.buf[0];
    state = SET_POSITION;
  }
}

void blessyou_controller(void const* argument) {
  // System setup
  registerCANMsgHandler(BLESSYOU_SYS_ID, &blessyouCANCallback);
  leftMotor = create_vesc(BLESSYOU_MOTOR_L, BLESSYOU_MOTOR_POLE_PAIRS);
  rightMotor = create_vesc(BLESSYOU_MOTOR_R, BLESSYOU_MOTOR_POLE_PAIRS);
  HAL_ADC_Start_DMA(&hadc1, adcVal, 2);

  TickType_t lastWakeTime;
  while (1) {
    vTaskDelayUntil(&lastWakeTime, BLESSYOU_LOOP_MS * portTICK_RATE_MS);

    bool lowLimitR =
        HAL_GPIO_ReadPin(BLESSYOU_LimitRL_GPIO_Port, BLESSYOU_LimitRL_Pin) == 0;
    bool lowLimitL =
        HAL_GPIO_ReadPin(BLESSYOU_LimitLL_GPIO_Port, BLESSYOU_LimitLL_Pin) == 0;
    bool highLimitR =
        HAL_GPIO_ReadPin(BLESSYOU_LimitRH_GPIO_Port, BLESSYOU_LimitRH_Pin) == 0;
    bool highLimitL =
        HAL_GPIO_ReadPin(BLESSYOU_LimitLH_GPIO_Port, BLESSYOU_LimitLH_Pin) == 0;
    // Update speed
    F32 speedL = vesc_get_rpm(leftMotor) * BLESSYOU_RPM_TO_MMS;
    F32 speedR = vesc_get_rpm(rightMotor) * BLESSYOU_RPM_TO_MMS;
    currentSpeed = (S8)((speedL + speedR) / 2.0f);
    // Update position
    U32 potRval = adcVal[0];
    U32 potLval = adcVal[1];
    F32 lPos = (F32)potLval * BLESSYOU_ADC_TO_MM;
    F32 rPos = (F32)potRval * BLESSYOU_ADC_TO_MM;
    currentPos = (S8)((lPos + rPos) / 2.0f);

    // Set target speeds
    F32 desSpeed = 0.0f;
    if (state == SET_SPEED) {
      desSpeed = (F32)targetSpeed;
    } else if (state == SET_POSITION) {
      F32 posErr = ((F32)currentPos - (F32)targetPos);
      if (fabs(posErr) < BLESSYOU_POS_DEADZONE) {
        desSpeed = 0.0f;
      } else {
        desSpeed = posErr * BLESSYOU_POS_P;
        if (fabs(desSpeed) < BLESSYOU_POS_MINSPEED) {
          desSpeed = copysignf(BLESSYOU_POS_MINSPEED, desSpeed);
        }
      }
    }

    // Apply corrective control loop to level elevator
    F32 levelErr =
        lPos - rPos;  // Positive error means left is lower than the right
    F32 levelCorrectSpeed = levelErr * BLESSYOU_LEVEL_P;
    // Send speed to VESCs
    F32 leftSpeed = desSpeed - levelCorrectSpeed;
    F32 rightSpeed = desSpeed + levelCorrectSpeed;
    // Zero speeds if limits are hit
    if (lowLimitL && leftSpeed > 0.0f) {
      leftSpeed = 0.0f;
    }
    if (lowLimitR && rightSpeed > 0.0f) {
      rightSpeed = 0.0f;
    }
    if (highLimitL && leftSpeed < 0.0f) {
      leftSpeed = 0.0f;
    }
    if (highLimitR && rightSpeed < 0.0f) {
      rightSpeed = 0.0f;
    }
    // Limit movement if SNEEZE isn't homed
    if (!isSNEEZEHome() && currentPos <= BLESSYOU_POS_NONHOME_MIN) {
      if (leftSpeed < 0.0f) {
        leftSpeed = 0.0f;
      }
      if (rightSpeed < 0.0f) {
        rightSpeed = 0.0f;
      }
    }
    vesc_set_rpm(leftMotor, leftSpeed / BLESSYOU_RPM_TO_MMS);
    vesc_set_rpm(rightMotor, rightSpeed / BLESSYOU_RPM_TO_MMS);

    // Send status message
    bool topLimit = highLimitL || highLimitR;
    bool bottomLimit = lowLimitL || lowLimitR;
    U8 buffer[3];
    buffer[0] = (((U8)bottomLimit) << 1) | ((U8)topLimit);
    buffer[1] = currentPos;
    buffer[2] = currentSpeed;
    do_send_can_message((BLESSYOU_MSG_STATUS << 8u) | ACHOO_SYS_ID, buffer, 3);
  }
}
