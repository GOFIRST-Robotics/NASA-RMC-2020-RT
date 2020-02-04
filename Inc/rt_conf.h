//
// Created by nick on 11/5/19.
//

#ifndef NASA_RMC_RT_RT_CONF_H
#define NASA_RMC_RT_RT_CONF_H
/*
 * This file should be a central location that all interacting IDs can be
 * placed, including VESC IDs and system IDs, since all of these have to be
 * unique between each other. Verify that this information is reflected in the
 * README
 */

// System IDs
#define ACHOO_SYS_ID 101u
#define GESUNDHEIT_SYS_ID 102u
#define SNEEZE_SYS_ID 103u

// Controller IDs
#define ACHOO_MOTOR_L 42u
#define ACHOO_MOTOR_R 2u
#define GESUNDHEIT_MOTOR_ID 3u
#define SNEEZE_MOTOR_DIG 4u
#define SNEEZE_MOTOR_TRANS 5u

#endif  // NASA_RMC_RT_RT_CONF_H
