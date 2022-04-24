#include "setup/control/lifts.h"
#include "setup/util/misc.h"

pros::Motor FrontLift(6, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_DEGREES); //is backwards, made it 0 not 1

pros::ADIDigitalOut Claw ('C', false);

pros::ADIDigitalOut BackClamp ('A');

pros::ADIDigitalOut LeftWing ('B', false);
pros::ADIDigitalOut RightWing ('H', false);
