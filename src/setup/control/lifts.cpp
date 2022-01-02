#include "setup/control/lifts.h"
#include "setup/util/misc.h"

pros::Motor FrontLift(11, MOTOR_GEARSET_36, 1, MOTOR_ENCODER_DEGREES);
pros::Motor BackLift(21, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_DEGREES);

pros::ADIDigitalOut Claw ('H');

pros::ADIDigitalOut BottomClaw ('A', true);

pros::Motor Tilter(7, MOTOR_GEARSET_18, 0, MOTOR_ENCODER_DEGREES);
