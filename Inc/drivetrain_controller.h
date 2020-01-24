//
// Created by hunter on 11/15/19.
//

#ifndef NASA_RMC_RT_DRIVETRAIN_CONTROLLER_H
#define NASA_RMC_RT_DRIVETRAIN_CONTROLLER_H

#include "can_manager.h"
//see https://husarion.com/tutorials/ros-tutorials/3-simple-kinematics-for-mobile-robot/
//for an explanation on how the mathematics for the twist were calculated

//ALL VALUES ARE IN MILLIMETERS
#define DRIVETRAIN_SYS_ID 100u
#define RADIUS 1.00
#define WIDTH 2.00
typedef enum {
    DRIVE_MSG_TWIST = 40
}DRIVE_MSG_T;

#define DRIVE_LOOP_MS 100 //milliseconds
#define DRIVE_MOTOR_POLE_PAIRS 14

#endif //NASA_RMC_RT_DRIVETRAIN_CONTROLLER_H
