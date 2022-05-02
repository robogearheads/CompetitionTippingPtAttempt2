#include "main.h"

#include "setup/control/base.h"
#include "setup/control/intake.h"
#include "setup/control/lifts.h"

#include "setup/util/misc.h"

#include "setup/util/MovementFunctions.h"

void WingTask(){
	pros::delay(200);
	LeftWing.set_value(true);
	RightWing.set_value(true);
	pros::delay(700);
	LeftWing.set_value(false);
	RightWing.set_value(false);

	while(true){
		pros::delay(20);
	}
}

void driveForward(){
	LF.move_velocity(50);
	LB.move_velocity(50);
	RF.move_velocity(50);
	RB.move_velocity(50);
	LM.move_velocity(50);
	RM.move_velocity(50);
}

void driveForwardFast(){
	LF.move_velocity(195);
	LB.move_velocity(195);
	RF.move_velocity(195);
	RB.move_velocity(195);
	LM.move_velocity(195);
	RM.move_velocity(195);
}

void driveBack(double speed){
	LF.move_velocity(-speed);
	LB.move_velocity(-speed);
	RF.move_velocity(-speed);
	RB.move_velocity(-speed);
	LM.move_velocity(-speed);
	RM.move_velocity(-speed);
}

void driveBackFast(){
	LF.move_velocity(-150);
	LB.move_velocity(-150);
	RF.move_velocity(-150);
	RB.move_velocity(-150);
	LM.move_velocity(-150);
	RM.move_velocity(-150);
}

void turnRight(double speed){
	LF.move_velocity(speed);
	LB.move_velocity(speed);
	RF.move_velocity(-speed);
	RB.move_velocity(-speed);
	LM.move_velocity(speed);
	RM.move_velocity(-speed);
}


void stop(){
	LF.move_velocity(0);
	LB.move_velocity(0);
	RF.move_velocity(0);
	RB.move_velocity(0);
	LM.move_velocity(0);
	RM.move_velocity(0);
}


void WinTriangle(){
  //Bring Arm Down

  //Drive to goal
  driveBack(75);
  pros::delay(1600);
  stop();

  //Pick up goal


  //Turn
  turnRight(75);
  pros::delay(250);
  stop();

  driveForward();
  pros::delay(2000);
  stop();



  driveForward();
  pros::delay(600);
  stop();
}

void CenterMiddle (){
	Claw.set_value(true);
	BackClamp.set_value(false);
	forwardForDistance(57, 195);
	Claw.set_value(false);
	fastGoForwardPID(-20);
	goForwardPID(-19);
	turnPID(-90);
	goForwardPID(-31);
	BackClamp.set_value(true);
	Intake.move_velocity(-500);
	forwardForDistance(30, 50);
	pros::delay(1000);
	Intake.move_velocity(0);
	BackClamp.set_value(false);
	forwardForDistance(5, 50);
}

void CenterTable(){
	BackClamp.set_value(false);
	Claw.set_value(true); //39
  forwardForDistance(40, 195);
  Claw.set_value(false);
	fastGoForwardPID(-10);
	goForwardPID(-11);
	turnPID(-90);
	goForwardPID(-15);
  BackClamp.set_value(true);
  pros::delay(500);
	Intake.move_velocity(-500);
	pros::delay(2000);
	goForwardPID(30);
  Intake.move_velocity(0);
  BackClamp.set_value(false);
	pros::delay(1000);
	forwardForDistance(5, 50);
}

void CenterBanner(){
	pros::Task lift(liftPID);
	Claw.set_value(true);
	forwardForDistance(44, 195);
  Claw.set_value(false);
  pros::delay(10);
	liftHeight = 100;
	fastGoForwardPID(-20);
	goForwardPID(-30);
	pros::delay(500);
	goForwardPID(2);
	turnPID(-85);
	goForwardPID(-12.5);
	BackClamp.set_value(true);
	pros::delay(1000);
	Intake.move_velocity(-500);
	pros::delay(2000);
	forwardForDistance(10, 50);
	lift.remove();
	Intake.move_velocity(0);
	BackClamp.set_value(false);
	forwardForDistance(5, 50);
}

void Banner2(){
	pros::Task lift(liftPID);
	//pros::Task lift(liftPID);
	Claw.set_value(false);
	goForwardPID(-49);
	goForwardPID(8.5);
	turnPID(160);
	goForwardPID(34);
	Claw.set_value(true);
	pros::delay(500);
	liftHeight = 500;
	goForwardPID(-44);
	turnPID(90);
	goForwardPID(-35);
}

void Table2(){
	BackClamp.set_value(false);
	Claw.set_value(true); //39
  forwardForDistance(41, 195);
  Claw.set_value(false);
	fastGoForwardPID(-21.5);
	Claw.set_value(true);
	goForwardPID(-14);
	turnPID(-43);
	goForwardPID(52);
	Claw.set_value(false);
	pros::delay(100);
	goForwardPID(-41);
	turnPID(-90);
	goForwardPID(-26);
	BackClamp.set_value(true);
	pros::delay(500);
	Intake.move_velocity(-500);
	goForwardPID(30);
	Intake.move_velocity(0);
	BackClamp.set_value(false);
	pros::delay(1000);
	forwardForDistance(5, 50);
}

void Skills(){
	pros::Task lift(liftPID);
	//Get first goal - fill rings
	BackClamp.set_value(true);
	Intake.move_velocity(-450);
	liftHeight = 400;
	Claw.set_value(true);
	pros::delay(2000);
	forwardForDistance(18, 15);
	goForwardPID(-18);
	pros::delay(1000);

	//Get neutral goal
	liftHeight = 0;
	preciseTurnPID(94);
	Intake.move_velocity(-450);
	goForwardPID(52);
	pros::delay(700);
	Claw.set_value(false);

	//Score first goal
	liftHeight = 800;
	pros::delay(1500);
	moveToPoint(1.1, 0.2286);
	liftHeight = 600;
	pros::delay(1000);
	Claw.set_value(true);
	BackClamp.set_value(false);
	goForwardPID(-16);
	goForwardPID(7);

	//Score second goal
	liftHeight = 0;
	preciseTurnPID(Inertial.get_heading() + 180);
	forwardForDistance(8, 50);
	Claw.set_value(false);
	pros::delay(500);
	preciseTurnPID(Inertial.get_heading() + 165);
	liftHeight = 550;
	pros::delay(1500);
	goForwardPID(20);
	Claw.set_value(true);

	//Get next goal
	goForwardPID(-5);
	liftHeight = 0;
	turnPID(180);
	goForwardPID(-43);
	BackClamp.set_value(true);

	//Get next goal while picking up rings
	liftHeight = 400;
	pros::delay(500);
	//moveToPoint(0.508, -0.8382);
	arcMoveToPoint(0.4064, -0.9398, 6);
	liftHeight = 0;
	turnPID(-90);
	goForwardPID(13);
	Claw.set_value(false);

	//Score
	liftHeight = 600;
	pros::delay(1700);
	moveToPoint(1.0668, -0.2032);
	Claw.set_value(true);

	//Get next goal
	goForwardPID(-20);
	liftHeight = 0;
	turnPID(-45);
	goForwardPID(20);
	Claw.set_value(false);
	turnPID(-125);
	liftHeight = 500;

	//Align + park
	goForwardPID(80);
	turnPID(0);
	goForwardPID(30);
	liftHeight = 0;
	pros::delay(1000);
	balancePID();

	//Fix - placement of blue/yellow goal, don't lift and turn, decrease beginning time, end stuff

}

void RakeTable(){
	pros::Task wings(WingTask);
	fastGoForwardPID(38); //was 34
	pros::delay(75);
	LeftWing.set_value(false);
	RightWing.set_value(false);
	pros::delay(150); //was 450
	goForwardPID(-35);
	wings.remove();
}
