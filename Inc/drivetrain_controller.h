//
// Created by hunter on 11/15/19.
//

#ifndef NASA_RMC_RT_DRIVETRAIN_CONTROLLER_H
#define NASA_RMC_RT_DRIVETRAIN_CONTROLLER_H

#include "can_manager.h"
// see
// https://husarion.com/tutorials/ros-tutorials/3-simple-kinematics-for-mobile-robot/
// for an explanation on how the mathematics for the twist were calculated

// ALL VALUES ARE IN MILLIMETERS
#define RADIUS 1.00f
#define WIDTH 2.00f
#define DRIVE_MSG_TWIST 35u
#define DRIVE_MSG_ODOM_POSE 36u
#define DRIVE_MSG_ODOM_TWIST 37u

#define DRIVE_LOOP_MS 100u  // milliseconds, unsigned
#define DRIVE_MOTOR_POLE_PAIRS 7u

#endif  // NASA_RMC_RT_DRIVETRAIN_CONTROLLER_H
