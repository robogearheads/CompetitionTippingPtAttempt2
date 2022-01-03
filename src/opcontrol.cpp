#include "main.h"

#include "setup/control/base.h"
#include "setup/control/intake.h"
#include "setup/control/lifts.h"

#include "setup/util/misc.h"
#include "setup/util/MovementFunctions.h"

void opcontrol() {
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
      //moveToPoint(0, 0.54);
      turnPID(0);
    }
    //GPS Sensor - printing values
    double xpos = GPSSensor.get_status().x;
    double ypos = GPSSensor.get_status().y;

    pros::lcd::print(2, "x value: %.3f", xpos);
    pros::lcd::print(3, "y value: %.3f", ypos);
    double robotHeading = GPSSensor.get_heading();
    pros::lcd::print(4, "heading: %.3f", robotHeading);

		pros::delay(20);
	}
}
