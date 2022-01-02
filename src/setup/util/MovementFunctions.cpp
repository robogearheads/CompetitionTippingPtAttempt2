#include "main.h"
#include "setup/util/MovementFunctions.h"
#include "setup/control/base.h"

#define CONVERSION_FACTOR (3.25*M_PI)

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
		double current_angle = GPSSensor.get_heading();
		double error = 2;
		double integral = 0;
		double derivative = 0;
		double prevError = 0;
		double power = 1;

		double kP = 3; //4
		double kI = 0.5; //0.5
		double kD = 2; //2

		double currentRotation = Inertial.get_rotation();

		while(error > 0.1 || error < -0.1){ //Was -1
			error = (targetTheta - GPSSensor.get_heading());
      pros::lcd::print(0, "error is %.3f", error);
      if(error < -180){
        error = error + 360;
      }
      else if(error > 180){
        error = error - 360;
      }
			pros::lcd::print(4, "error is %.3f", error);
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
			pros::delay(10);
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
	double kP = 18; //18
	double kD = 12.5; //12.5
	double kI = 0.5; //18

  //current motor values
  double LFcurrent = LF.get_position() * CONVERSION_FACTOR;
  double LMcurrent = LM.get_position() * CONVERSION_FACTOR;
  double LBcurrent = LB.get_position() * CONVERSION_FACTOR;
  double RFcurrent = RF.get_position() * CONVERSION_FACTOR;
  double RMcurrent = RM.get_position() * CONVERSION_FACTOR;
  double RBcurrent = RB.get_position() * CONVERSION_FACTOR;

	while (error > 0 || error < -1){
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

  turnPID(targetAngle);
  goForwardPID(forwardDistance);
}

//Just used at the end (needs tuning)
void balancePID() {
	double error = 0;
	double derivative = 0;
	double prevError = 0;
	double power = 1;

	double kP = 13;
	double kD = 2.7;

  //Start going forward, wait until it starts climbing
  while(Inertial.get_pitch() < 5 && Inertial.get_pitch() > -5){
    forwardVelocity(100);
  }
  stop1();

  //Climbing PD
  while(true){
    error = Inertial.get_pitch();
    double derivative = error - prevError;
		prevError = error;
		power = error*kP + derivative*kD;
    forwardVelocity(power);
		pros::delay(25);
	}
}
