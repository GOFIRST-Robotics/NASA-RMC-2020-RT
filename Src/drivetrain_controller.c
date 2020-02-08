//
// Created by hunter on 11/15/19.
//

#include "drivetrain_controller.h"
#include <string.h>
#include <FreeRTOS.h>
#include <can_manager.h>
#include <rt_conf.h>
#include <task.h>
#include "VESC.h"
#include "stdlib.h"
#include "math.h"

//I am telling the motor controllers to implement a twist message until I get another twist message
F32 new_speed_left = 0;
F32 new_speed_right = 0;
F32 theta = 0; //keeps tracks of robot angle, init facing 0 degrees on the top down xy coordinate plane
F32 x = 0, y = 0; //absolute position of the robot, starts at (0,0)


void drivetrain_move(rmc_can_msg msg) {
    // Because of the mask we only get messages that have our ID
    int32_t cmd_speed = 0; //in mm/s
    int32_t cmd_angV = 0; //in mrad/s

//    if(msg.id & 0xFF != DRIVETRAIN_SYS_ID)
//    {
//        return;
//    }
// This portion is already done by the CAN dispatcher. It is here purely for Hunter's understanding.

    switch(msg.id >> 8)
    {
        case DRIVE_MSG_TWIST:
            memcpy(&cmd_speed, &(msg.buf[4]), 4);
            memcpy(&cmd_angV, (msg.buf), 4);

            new_speed_right = (cmd_angV * WIDTH) / 2 + cmd_speed; //in mm/s.
            new_speed_left = cmd_speed * 2 - new_speed_right;
        break;

        default:
            //weewoo something went wrong
        break;
    }
}

void drivetrain_loop(void)
{
    //SETUP
    registerCANMsgHandler(DRIVETRAIN_SYS_ID, &drivetrain_move);
    VESC* blm = create_vesc(DRIVE_MOTOR_BL, DRIVE_MOTOR_POLE_PAIRS);
    VESC* brm = create_vesc(DRIVE_MOTOR_BR, DRIVE_MOTOR_POLE_PAIRS);
    VESC* frm = create_vesc(DRIVE_MOTOR_FR, DRIVE_MOTOR_POLE_PAIRS);
    VESC* flm = create_vesc(DRIVE_MOTOR_FL, DRIVE_MOTOR_POLE_PAIRS);


    TickType_t lastWakeTime;
    //END SETUP

    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, DRIVE_LOOP_MS * portTICK_RATE_MS);// Not sure what this does yet,
        // min 1hz refresh rate

        F32 rpm_right = new_speed_right/(RADIUS*2*3.14);
        F32 rpm_left = new_speed_left/(RADIUS*2*3.14);
        vesc_set_rpm(blm, rpm_left);
        vesc_set_rpm(flm, rpm_left);
        vesc_set_rpm(brm, rpm_right);
        vesc_set_rpm(frm, rpm_right);

        //now publish some optometry data
        //keep track of the position estimate, you start at 0,0, publish the position estimate every loop.

        //I am using this model
        //https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-186-mobile-autonomous-systems-laboratory-january-iap-2005/study-materials/odomtutorial.pdf

        //distance traveled = wheel radius * rpm * loop length, this will be done for each side
        //D_center= (Dleft+Dright) /2
        //Angle change from orgin, theta= dright-dleft/(WIDTH)
        //direction = angle change from origin+ previous direction
        //x_new = x_old +d_center*cos(theta)
        //y_new = y_old +d_center*sin(theta)
        F32 speed_right = vesc_get_rpm(brm);
        F32 speed_left  = vesc_get_rpm(blm);

        F32 d_left= speed_left*(DRIVE_LOOP_MS/1000); //distance traveled by the left side, in mm
        F32 d_right= speed_right*(DRIVE_LOOP_MS/1000); //distance traveled by the right side, in mm
        F32 d_center = (d_left+d_right)/2;
        //https://en.wikipedia.org/wiki/Polar_coordinate_system
        F32 phi = (d_right-d_left)/WIDTH; // how much the robot has turned this loop
        theta = phi+theta; // theta can be negative
        //no need to map phi from 0 to 2pi, cos function will do that for me
        x = x + d_center*cos(phi);
        y = y + d_center*sin(phi);
    }
}