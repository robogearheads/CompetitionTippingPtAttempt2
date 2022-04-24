#include "setup/control/base.h"
#include "setup/util/misc.h"

//Motor declarations
pros::Motor RB(20, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_ROTATIONS),
            RF(15, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_ROTATIONS),
            LB(17, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_ROTATIONS),
            LF(13, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_ROTATIONS),
            LM(12, MOTOR_GEARSET_18, 1, MOTOR_ENCODER_ROTATIONS),
            RM(14, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_ROTATIONS);

//Inertial Sensor
pros::Imu Inertial(7);

//GPS Sensor
pros::GPS GPSSensor(5, -0.2032, 0);
