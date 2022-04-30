#include "main.h"

#include "setup/control/base.h"
#include "setup/control/intake.h"
#include "setup/control/lifts.h"

#include "setup/util/misc.h"
#include "setup/util/MovementFunctions.h"

//extern pros::Task lift;

void opcontrol() {
	//Setting brake modes
	LF.set_brake_mode(MOTOR_BRAKE_COAST);
	LM.set_brake_mode(MOTOR_BRAKE_COAST);
	LB.set_brake_mode(MOTOR_BRAKE_COAST);
	RF.set_brake_mode(MOTOR_BRAKE_COAST);
	RM.set_brake_mode(MOTOR_BRAKE_COAST);
	RB.set_brake_mode(MOTOR_BRAKE_COAST);

	FrontLift.set_brake_mode(MOTOR_BRAKE_HOLD);

	while (true) {
		//Drive Code
    LF.move(controller.get_analog(ANALOG_LEFT_Y));
		LM.move(controller.get_analog(ANALOG_LEFT_Y));
    LB.move(controller.get_analog(ANALOG_LEFT_Y));
		RF.move(controller.get_analog(ANALOG_RIGHT_Y));
		RM.move(controller.get_analog(ANALOG_RIGHT_Y));
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

		//Intake
		if(controller.get_digital(DIGITAL_L1)){
      Intake.move_velocity(500);
    }
    else if(controller.get_digital(DIGITAL_L2)){
      Intake.move_velocity(-500);
    }
    else{
      Intake.move_velocity(0);
    }

		//Pneumatic Claw
		if(controller.get_digital(DIGITAL_DOWN)){
			Claw.set_value(false);
		}
		else if(controller.get_digital(DIGITAL_UP)){
			Claw.set_value(true);
		}

		//Back Clamp
		if(controller.get_digital(DIGITAL_LEFT)){
			BackClamp.set_value(false);
		}
		else if(controller.get_digital(DIGITAL_RIGHT)){
			BackClamp.set_value(true);
		}

		//Wings (testing)
		if(controller.get_digital(DIGITAL_B)){
			LeftWing.set_value(true);
			RightWing.set_value(true);
		}
		else if(controller.get_digital(DIGITAL_X)){
			LeftWing.set_value(false);
			RightWing.set_value(false);
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
