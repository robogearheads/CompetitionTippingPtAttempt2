#include "setup/control/lifts.h"
#include "setup/util/misc.h"

pros::Motor FrontLift(11, MOTOR_GEARSET_36, 1, MOTOR_ENCODER_DEGREES);
pros::Motor BackLift(13, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_DEGREES);

pros::ADIDigitalOut Claw ('A');

pros::ADIDigitalOut BottomClaw ('H', true);

pros::Motor Tilter(21, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_DEGREES);
