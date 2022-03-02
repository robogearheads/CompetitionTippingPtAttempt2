#include "setup/control/lifts.h"
#include "setup/util/misc.h"

pros::Motor FrontLift(11, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_DEGREES); //is backwards, made it 0 not 1
pros::Motor BackLift(13, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_DEGREES); //port 13

pros::ADIDigitalOut Claw ('A', true);

pros::ADIDigitalOut BottomClaw ('H');

pros::ADIDigitalOut LeftWing ('G', false);
pros::ADIDigitalOut RightWing ('B', false);

//Doesn't actually exist - but pros is being stupid
pros::Motor Tilter(21, MOTOR_GEARSET_36, 0, MOTOR_ENCODER_DEGREES);
