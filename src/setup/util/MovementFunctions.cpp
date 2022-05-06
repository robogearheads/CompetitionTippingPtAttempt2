#include "main.h"
#include "setup/util/MovementFunctions.h"
#include "setup/control/base.h"
#include "setup/control/lifts.h"

#define CONVERSION_FACTOR (3.25*M_PI)*5/3

//Function to go forward at a certain speed
void forwardVelocity(double velocity){
  LF.move_velocity(velocity);
  LM.move_velocity(velocity);
  LB.move_velocity(velocity);
  RF.move_velocity(velocity);
  RM.move_velocity(velocity);
  RB.move_velocity(velocity);
}

//Stops
void stop1(){
  LF.move_velocity(0);
  LM.move_velocity(0);
  LB.move_velocity(0);
  RF.move_velocity(0);
  RM.move_velocity(0);
  RB.move_velocity(0);
}

void turnPID(double targetTheta) {
    pros::lcd::print(1, "running turn PID");
		double current_angle = GPSSensor.get_heading();
		double error = 2;
		double integral = 0;
		double derivative = 0;
		double prevError = 0;
		double power = 1;
    double counter = 0;

		double kP = 1.3; //4
		double kI = 0; //0.5
		double kD = 0.8; //2

		while((error < -0.07 || error > 0.07) && counter < 400){ //Was -0.2
			error = (targetTheta - Inertial.get_heading());
      pros::lcd::print(0, "error is %.3f", error);
      if(error < -180){
        error = error + 360;
      }
      else if(error > 180){
        error = error - 360;
      }
			double derivative = error - prevError;
			prevError = error;
			integral = integral + error;
			if (error < 0){
				integral = 0;
			}
			if (error > 20){
				integral = 0;
			}
			power = error*kP + derivative*kD + integral*kI;
			LF.move_velocity(power);
			LB.move_velocity(power);
      LM.move_velocity(power);
			RF.move_velocity(-power);
      RM.move_velocity(-power);
			RB.move_velocity(-power);
      counter ++;
			pros::delay(5);
		}
		LF.move_velocity(0);
		LB.move_velocity(0);
    LM.move_velocity(0);
		RF.move_velocity(0);
    RM.move_velocity(0);
		RB.move_velocity(0);
    pros::lcd::print(6, "exiting loop");
}

void preciseTurnPID(double targetTheta) {
		double error = 1;
		double integral = 0;
		double derivative = 0;
		double prevError = 0;
		double power = 1;
    double counter = 0;

		double kP = 1.9; //4
		double kI = 0; //0.5
		double kD = 1.2; //2

		while((error < -0.003 || error > 0.003) && counter < 400){ //Was -0.2
			error = (targetTheta - Inertial.get_heading());
      pros::lcd::print(0, "error is %.3f", error);
      if(error < -180){
        error = error + 360;
      }
      else if(error > 180){
        error = error - 360;
      }
			double derivative = error - prevError;
			prevError = error;
			integral = integral + error;
			if (error < 0){
				integral = 0;
			}
			if (error > 20){
				integral = 0;
			}
			power = error*kP + derivative*kD + integral*kI;
			LF.move_velocity(power);
			LB.move_velocity(power);
      LM.move_velocity(power);
			RF.move_velocity(-power);
      RM.move_velocity(-power);
			RB.move_velocity(-power);
      counter ++;
			pros::delay(5);
		}
		LF.move_velocity(0);
		LB.move_velocity(0);
    LM.move_velocity(0);
		RF.move_velocity(0);
    RM.move_velocity(0);
		RB.move_velocity(0);
    pros::lcd::print(6, "exiting loop");
}

void goForwardPID(double distance){
  double error = 1;
	double integral = 0;
	double prevError = 0;
	double power = 1;
  double counter = 0;
	double kP = 11; //18
	double kD = 12.5; //12.5
	double kI = 0.07; //0.1

  //current motor values
  double LFcurrent = LF.get_position() * CONVERSION_FACTOR;
  double LMcurrent = LM.get_position() * CONVERSION_FACTOR;
  double LBcurrent = LB.get_position() * CONVERSION_FACTOR;
  double RFcurrent = RF.get_position() * CONVERSION_FACTOR;
  double RMcurrent = RM.get_position() * CONVERSION_FACTOR;
  double RBcurrent = RB.get_position() * CONVERSION_FACTOR;

	while ((error > 0.3 || error < -0.3) && counter < 200){
    //forward distances (motors separate)
    double LFdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double LMdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double LBdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double RFdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double RMdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double RBdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;

    double avgDistanceTraveled = (LFdistance + LMdistance + LBdistance + RFdistance + RMdistance + RBdistance)/6;

		error = distance - avgDistanceTraveled;
		pros::lcd::print(4, "error is %.3f", error);
		double derivative = error - prevError;
		prevError = error;
		integral = integral + error;
		if (error < 0){
			integral = 0;
		}
		if (error > (0.5*distance)){
			integral = 0;
		}
		power = error*kP + derivative*kD + integral*kI;
	  forwardVelocity(power);
    counter ++;
		pros::delay(15);
	}
  stop1();
}

double getAngle(double pointX, double pointY) { //returns in degrees
	double currentX = GPSSensor.get_status().x;
	double currentY = GPSSensor.get_status().y;
	double angle = atan2(pointX - currentX, pointY - currentY);
	return angle * 180 / M_PI;
}

double getLength(double pointX, double pointY){ //returns inches
	double currentX = GPSSensor.get_status().x;
	double currentY = GPSSensor.get_status().y;
	double delta_x = pointX - currentX;
	double delta_y = pointY - currentY;
	return sqrt(delta_x*delta_x + delta_y*delta_y) * 39.3700787402;
}

void moveToPoint(double x, double y){
  double targetAngle = getAngle (x, y);
  double forwardDistance = getLength(x, y);

  preciseTurnPID(targetAngle);
  pros::delay(100);
  goForwardPID(forwardDistance);
}

void preciseBackToPoint(double x, double y){
  double targetAngle = getAngle (x, y);
  double forwardDistance = getLength(x, y);

  preciseTurnPID(targetAngle + 180);
  pros::delay(100);
  goForwardPID(-forwardDistance);
}

void backToPoint(double x, double y){
  double targetAngle = getAngle (x, y);
  double forwardDistance = getLength(x, y);

  turnPID(targetAngle + 180);
  pros::delay(100);
  goForwardPID(-forwardDistance);
}

//Just used at the end (needs tuning)
void balancePID() {
	double error = 0;
	double derivative = 0;
	double prevError = 0;
	double power = 1;

	double kP = 2;
	double kD = 3;

  //Start going forward, wait until it starts climbing
  while(Inertial.get_pitch() < 20 && Inertial.get_pitch() > -20){ //was +- 20
    forwardVelocity(100);
  }
  stop1();

  //Climbing PD
  while(true){
    error = Inertial.get_pitch();
    if(Inertial.get_pitch() > 10 && Inertial.get_pitch() < 20){
      error = error - 38;
      BackClamp.set_value(false);
    }
    else if(Inertial.get_pitch() < -10 && Inertial.get_pitch() > -20){
      error = error + 38;
    }
    double derivative = error - prevError;
		prevError = error;
		power = error*kP + derivative*kD;
    forwardVelocity(power);
		pros::delay(15);
	}
}

void fastGoForwardPID(double distance){
  double error = 1;
	double integral = 0;
	double prevError = 0;
	double power = 1;
  double counter = 0;
	double kP = 25; //24
	double kD = 12.5; //12.5
	double kI = 0.07; //0.1

  //current motor values
  double LFcurrent = LF.get_position() * CONVERSION_FACTOR;
  double LMcurrent = LM.get_position() * CONVERSION_FACTOR;
  double LBcurrent = LB.get_position() * CONVERSION_FACTOR;
  double RFcurrent = RF.get_position() * CONVERSION_FACTOR;
  double RMcurrent = RM.get_position() * CONVERSION_FACTOR;
  double RBcurrent = RB.get_position() * CONVERSION_FACTOR;

	while ((error > 0.15 || error < -0.15) && counter < 300){ //0.1
    //forward distances (motors separate)
    double LFdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double LMdistance = LM.get_position()*CONVERSION_FACTOR - LMcurrent;
    double LBdistance = LB.get_position()*CONVERSION_FACTOR - LBcurrent;
    double RFdistance = RF.get_position()*CONVERSION_FACTOR - RFcurrent;
    double RMdistance = RM.get_position()*CONVERSION_FACTOR - RMcurrent;
    double RBdistance = RB.get_position()*CONVERSION_FACTOR - RBcurrent;

    double avgDistanceTraveled = (LFdistance + LMdistance + LBdistance + RFdistance + RMdistance + RBdistance)/6;

		error = distance - avgDistanceTraveled;
		pros::lcd::print(4, "error is %.3f", error);
		double derivative = error - prevError;
		prevError = error;
		integral = integral + error;
		if (error < 0){
			integral = 0;
		}
		if (error > (0.5*distance)){
			integral = 0;
		}
		power = error*kP + derivative*kD + integral*kI;
	  forwardVelocity(power);
    counter ++;
		pros::delay(15);
	}
  stop1();
}

void DriverBalancePID(){
  double error = 0;
	double derivative = 0;
	double prevError = 0;
	double power = 1;

	double kP = 3.5;
	double kD = 3;

  //Climbing PD
  while(true){
    error = Inertial.get_pitch();
    double derivative = error - prevError;
		prevError = error;
		power = error*kP + derivative*kD;
    forwardVelocity(power);
		pros::delay(15);
	}
}

void driverMovement(){
  //Drive Code
  LF.move(controller.get_analog(ANALOG_LEFT_Y));
  LB.move(controller.get_analog(ANALOG_LEFT_Y));
  RF.move(controller.get_analog(ANALOG_RIGHT_Y));
  RB.move(controller.get_analog(ANALOG_RIGHT_Y));
}

void forwardForDistance(double amount, double speed){
  double LFcurrent = LF.get_position() * CONVERSION_FACTOR;
  double LMcurrent = LM.get_position() * CONVERSION_FACTOR;
  double LBcurrent = LB.get_position() * CONVERSION_FACTOR;
  double RFcurrent = RF.get_position() * CONVERSION_FACTOR;
  double RMcurrent = RM.get_position() * CONVERSION_FACTOR;
  double RBcurrent = RB.get_position() * CONVERSION_FACTOR;

  double avgDistanceTraveled = 1;
  double counter = 0;

  while(avgDistanceTraveled < amount && counter < 350){
    double LFdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double LMdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double LBdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double RFdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double RMdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;
    double RBdistance = LF.get_position()*CONVERSION_FACTOR - LFcurrent;

    avgDistanceTraveled = (LFdistance + LMdistance + LBdistance + RFdistance + RMdistance + RBdistance)/6;
    forwardVelocity(speed);

    counter ++;

    pros::delay(20);
  }
  stop1();
}

void arcMoveToPoint(double x, double y, double accuracy){
  double counter = 0;
  double forwardDistance = 1;

  double fwdError = 100000;
	double fwdIntegral = 0;
	double fwdPrevError = 0;
	double fwdPower = 1;

  double turnError = 1;
  double turnIntegral = 0;
  double turnDerivative = 0;
  double turnPrevError = 0;
  double turnPower = 1;

	double fwd_kP = 11; //18
	double fwd_kD = 12.5; //12.5
	double fwd_kI = 0.07; //0.1

  double turn_kP = 2.2; //4
  double turn_kI = 0; //0.5
  double turn_kD = 1.2; //2

  double turnEffect = 0.4;
  double fwdEffect = 0.35;

	while ((fwdError > accuracy || fwdError < -accuracy)){ //add counter back later
    //Calculating distances
    double targetAngle = getAngle (x, y);
    forwardDistance = getLength(x, y);

    fwdError = forwardDistance;

		double fwdDerivative = fwdError - fwdPrevError;
		fwdPrevError = fwdError;
		fwdIntegral = fwdIntegral + fwdError;
		if (fwdError < 0){
			fwdIntegral = 0;
		}
		if (fwdError > (0.5*fwdError)){
			fwdIntegral = 0;
		}
		fwdPower = fwdError*fwd_kP + fwdDerivative*fwd_kD + fwdIntegral*fwd_kI;

    turnError = (targetAngle - Inertial.get_heading());
    if(turnError < -180){
      turnError = turnError + 360;
    }
    else if(turnError > 180){
      turnError = turnError - 360;
    }
    double derivative = turnError - turnPrevError;
    turnPrevError = turnError;
    turnIntegral = turnIntegral + turnError;
    if (turnError < 0){
      turnIntegral = 0;
    }
    if (turnError > 20){
      turnIntegral = 0;
    }
    turnPower = turnError*turn_kP + turnDerivative*turn_kD + turnIntegral*turn_kI;

    double leftPower = fwdEffect*fwdPower + turnEffect*turnPower;
    double rightPower = fwdEffect*fwdPower - turnEffect*turnPower;
/*
    while (leftPower < -200 || leftPower > 200 || rightPower < -200 || rightPower > 200){
      double scalingFactor = 1;
      scalingFactor = scalingFactor + 0.1;
      leftPower = leftPower/scalingFactor;
      rightPower = rightPower/scalingFactor;
      pros::delay(2);
    }
*/
    LF.move_velocity(leftPower);
    LB.move_velocity(leftPower);
    LM.move_velocity(leftPower);
    RF.move_velocity(rightPower);
    RM.move_velocity(rightPower);
    RB.move_velocity(rightPower);

    pros::lcd::print(4, "turnPower is %f", turnPower);
    pros::lcd::print(5, "fwdPower is %f", fwdPower);
    pros::lcd::print(6, "leftPower is %f", leftPower);
    pros::lcd::print(7, "rightPower is %f", rightPower);

    double xpos = GPSSensor.get_status().x;
    double ypos = GPSSensor.get_status().y;
    pros::lcd::print(0, "x value: %.3f", xpos*39.3700787402);
    pros::lcd::print(1, "y value: %.3f", ypos*39.3700787402);
    double robotHeading = GPSSensor.get_heading();
    pros::lcd::print(2, "GPS heading: %.3f", robotHeading);

    counter ++;
		pros::delay(15);
	}
}
