#include "setup/control/base.h"
#include "setup/util/misc.h"

//Motor declarations
pros::Motor RB(20, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_ROTATIONS),
            RF(9, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_ROTATIONS),
            LB(1, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_ROTATIONS),
            LF(2, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_ROTATIONS),
            LM(12, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_ROTATIONS),
            RM(10, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_ROTATIONS);

//Inertial Sensor
pros::Imu Inertial(14);

//GPS Sensor
pros::GPS GPSSensor(13, -0.1905, 0.0254);
