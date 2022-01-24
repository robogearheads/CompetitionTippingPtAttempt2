#include "main.h"

#include "setup/control/base.h"
#include "setup/control/intake.h"
#include "setup/control/lifts.h"

#include "setup/util/misc.h"
#include "setup/util/MovementFunctions.h"

void opcontrol() {
	//pros::Task lift(liftPID);

	//Setting brake modes
	LF.set_brake_mode(MOTOR_BRAKE_COAST);
	LB.set_brake_mode(MOTOR_BRAKE_COAST);
	RF.set_brake_mode(MOTOR_BRAKE_COAST);
	RB.set_brake_mode(MOTOR_BRAKE_COAST);

	FrontLift.set_brake_mode(MOTOR_BRAKE_HOLD);
	BackLift.set_brake_mode(MOTOR_BRAKE_HOLD);

	Tilter.set_brake_mode(MOTOR_BRAKE_HOLD);

	while (true) {
		//Drive Code
    LF.move(controller.get_analog(ANALOG_LEFT_Y));
    LB.move(controller.get_analog(ANALOG_LEFT_Y));
    RF.move(controller.get_analog(ANALOG_RIGHT_Y));
    RB.move(controller.get_analog(ANALOG_RIGHT_Y));

    //Lifts
    if(controller.get_digital(DIGITAL_R1)){
      FrontLift.move_velocity(100);
    }
    else if(controller.get_digital(DIGITAL_R2)){
      FrontLift.move_velocity(-100);
    }
    else{
      FrontLift.move_velocity(0);
    }

		//Back lift
		if(controller.get_digital(DIGITAL_L1)){
      BackLift.move_velocity(100);
    }
    else if(controller.get_digital(DIGITAL_L2)){
      BackLift.move_velocity(-100);
    }
    else{
      BackLift.move_velocity(0);
    }

		//Pneumatic Claw
		if(controller.get_digital(DIGITAL_UP)){
			Claw.set_value(false);
		}
		else if(controller.get_digital(DIGITAL_DOWN)){
			Claw.set_value(true);
		}

		//Bottom Pneumatic Claw
		if(controller.get_digital(DIGITAL_LEFT)){
			BottomClaw.set_value(false);
		}
		else if(controller.get_digital(DIGITAL_RIGHT)){
			BottomClaw.set_value(true);
		} //comment for no reaso

    if(controller.get_digital(DIGITAL_B)){
/*
			Claw.set_value(false);
			turnPID(90);
			goForwardPID(48);
*/
/*
			//Grab alliance goal and get out of corner
			Claw.set_value(false);
			BottomClaw.set_value(true);
			turnPID(50);
			goForwardPID(20);
			turnPID(110);
			goForwardPID(45);
			pros::delay(500);

			//Get first neutral goal
			Claw.set_value(true);
			liftHeight = 700;
			pros::delay(500);

			//Go to bridge and score
			moveToPoint(1.1176, 0.2032);
			liftHeight = 500;
			pros::delay(500);
			Claw.set_value(false);
			pros::delay(1000);

			//Spin around, drop alliance goal, pick up other alliance goal
			goForwardPID(-11);
			turnPID(0);
			pros::delay(1000);
			BottomClaw.set_value(false);
			pros::delay(500);
			liftHeight = 0;
			goForwardPID(9);
			pros::delay(100);
			turnPID(180);
			pros::delay(100);
			goForwardPID(-26);
			BottomClaw.set_value(true);
			pros::delay(500);
			//turnPID(180);
			goForwardPID(38);
			Claw.set_value(true);

			//Raise arm, score goal
			goForwardPID(-9);
			liftHeight = 620;
			pros::delay(1000);
			turnPID(110);
			goForwardPID(15);
			turnPID(120);
			Claw.set_value(false);

			//Back up, drive to, and pick up next goal
			goForwardPID(-7);
			liftHeight = 0;
			pros::delay(500);
			moveToPoint(GPSSensor.get_status().x, -1.0922);
			goForwardPID(-5);
			pros::delay(500);
			moveToPoint(0.1524, -0.9652);
			Claw.set_value(true);

			//Align and climb
			moveToPoint(-0.9144, GPSSensor.get_status().y);
			turnPID(40);
			goForwardPID(-38);
			turnPID(-8);
			liftHeight = 650;
			pros::delay(1300);
			goForwardPID(14);
			liftHeight = 0;
			turnPID(-1);
*/
			balancePID();
    }
    //GPS Sensor - printing values
    double xpos = GPSSensor.get_status().x;
    double ypos = GPSSensor.get_status().y;

    pros::lcd::print(2, "x value: %.3f", xpos);
    pros::lcd::print(3, "y value: %.3f", ypos);
    double robotHeading = GPSSensor.get_heading();
    pros::lcd::print(4, "GPS heading: %.3f", robotHeading);
		pros::lcd::print(1, "Inertial heading: %.3f", Inertial.get_heading());

		pros::delay(20);
	}
}
