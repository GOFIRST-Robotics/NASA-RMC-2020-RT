//
// Created by hunter on 11/15/19.
//

#include "drivetrain_controller.h"
#include <FreeRTOS.h>
#include <can_manager.h>
#include <math.h>
#include <print.h>
#include <rt_conf.h>
#include <stdio.h>
#include <stm32f3xx_hal.h>
#include <string.h>
#include <task.h>
#include "VESC.h"
#include "stdlib.h"

// I am telling the motor controllers to implement a twist message until I get
// another twist message
F32 new_speed_left = 0;
F32 new_speed_right = 0;
F32 theta = 0;  // keeps tracks of robot angle, init facing 0 degrees on the top
                // down xy coordinate plane
F32 x = 0, y = 0;  // absolute position of the robot, starts at (0,0)

void drivetrain_move(rmc_can_msg msg) {
  // Because of the mask we only get messages that have our ID
  S32 cmd_speed = 0;  // in mm/s
  S32 cmd_angV = 0;   // in mrad/s

  //    if(msg.id & 0xFF != DRIVETRAIN_SYS_ID)
  //    {
  //        return;
  //    }
  // This portion is already done by the CAN dispatcher. It is here purely for
  // Hunter's understanding.

  S32 idx = 0;
  switch (msg.id >> 8) {
    case DRIVE_MSG_TWIST:

      cmd_angV = buffer_pop_int32(msg.buf, &idx);
      cmd_speed = buffer_pop_int32(msg.buf, &idx);

      new_speed_right = (cmd_angV * DT_WIDTH) / 2 + cmd_speed;
      new_speed_left = cmd_speed * 2 - new_speed_right;
      new_speed_right = new_speed_right * DT_MMS_TO_RPM;
      new_speed_left = new_speed_left * DT_MMS_TO_RPM;
      break;

    default:
      // weewoo something went wrong
      break;
  }
}

void drivetrain_loop(void const* argument) {
  // SETUP
  registerCANMsgHandler(DRIVETRAIN_SYS_ID, &drivetrain_move);
  VESC* blm = create_vesc(DRIVE_MOTOR_BL, DRIVE_MOTOR_POLE_PAIRS);
  VESC* brm = create_vesc(DRIVE_MOTOR_BR, DRIVE_MOTOR_POLE_PAIRS);
  VESC* frm = create_vesc(DRIVE_MOTOR_FR, DRIVE_MOTOR_POLE_PAIRS);
  VESC* flm = create_vesc(DRIVE_MOTOR_FL, DRIVE_MOTOR_POLE_PAIRS);

  TickType_t lastWakeTime;
  // END SETUP

  while (1) {
    vTaskDelayUntil(
        &lastWakeTime,
        DRIVE_LOOP_MS * portTICK_RATE_MS);  // Not sure what this does yet,
                                            // min 1hz refresh rate

    vesc_set_rpm(blm, new_speed_left);
    vesc_set_rpm(flm, new_speed_left);
    vesc_set_rpm(brm, new_speed_right);
    vesc_set_rpm(frm, new_speed_right);

    // now publish some odometry data
    // keep track of the position estimate, you start at 0,0, publish the
    // position estimate every loop.

    // I am using this model
    // https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-186-mobile-autonomous-systems-laboratory-january-iap-2005/study-materials/odomtutorial.pdf

    // distance traveled = wheel radius * rpm * loop length, this will be done
    // for each side D_center= (Dleft+Dright) /2 Angle change from orgin, theta=
    // dright-dleft/(WIDTH) direction = angle change from origin+ previous
    // direction x_new = x_old +d_center*cos(theta) y_new = y_old
    // +d_center*sin(theta)
    F32 speed_right = vesc_get_rpm(brm);
    F32 speed_left = vesc_get_rpm(blm);
    F32 speed_center = (speed_right + speed_left) / 2.0f;
    F32 omega = (speed_right - speed_left) / DT_WIDTH;

    F32 d_center =
        speed_center *
        (DRIVE_LOOP_MS / 1000.0f);  // distance traveled by the robot, in mm
    F32 phi = omega * (DRIVE_LOOP_MS /
                       1000.0f);  // how much the robot has turned this loop
    theta += phi;                 // theta can be negative
    // no need to map phi from 0 to 2pi, cos function will do that for me
    x = x + d_center * cos(theta);
    y = y + d_center * sin(theta);
    U32 id = (DRIVE_MSG_ODOM_POSE << 8u) | DRIVETRAIN_SYS_ID;
    U8 buf[10];
    S32 idx = 0;
    S16 xposi = (S16)x;
    S16 yposi = (S16)y;
    S16 thetai = (S16)theta;
    S16 xveli = (S16)speed_center;
    S16 omegai = (S16)omega;
    buffer_put_int16(buf, &idx, xposi);
    buffer_put_int16(buf, &idx, yposi);
    buffer_put_int16(buf, &idx, thetai);
    buffer_put_int16(buf, &idx, xveli);
    buffer_put_int16(buf, &idx, omegai);
    do_send_can_message(id, buf, 6);
    id = (DRIVE_MSG_ODOM_TWIST << 8u) | DRIVETRAIN_SYS_ID;
    do_send_can_message(id, buf + 6, 4);
  }
}