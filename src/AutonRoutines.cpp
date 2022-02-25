#include "main.h"

#include "setup/control/base.h"
#include "setup/control/intake.h"
#include "setup/control/lifts.h"

#include "setup/util/misc.h"

#include "setup/util/MovementFunctions.h"

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
  BackLift.move_velocity(-100);
  pros::delay(2000);
  BackLift.move_velocity(0);

  //Drive to goal
  driveBack(75);
  pros::delay(1600);
  stop();

  //Pick up goal
  BackLift.move_velocity(100);
  pros::delay(1200);
  BackLift.move_velocity(0);

  //Turn
  turnRight(75);
  pros::delay(250);
  stop();

  driveForward();
  pros::delay(2000);
  stop();

  BackLift.move_velocity(-100);
  pros::delay(1300);
  BackLift.move_velocity(0);

  driveForward();
  pros::delay(600);
  stop();
}

void WinPlatform (){
  //Bring Arm Down
  BackLift.move_velocity(-100);
  pros::delay(2000);
  BackLift.move_velocity(0);

  driveBack(50);
  pros::delay(2000);
  stop();
  pros::delay(1000);
  driveForward();
  pros::delay(2000);
  stop();
}

void CenterTable(){
	Claw.set_value(false);
  forwardVelocity(200);
  pros::delay(1500);
  stop();

  Claw.set_value(true);
  pros::delay(100);
	liftHeight = 500;
	driveBack(185);
  pros::delay(1200);
  stop();
}

void CenterBanner(){
	Claw.set_value(false);
	driveForwardFast();
  pros::delay(1200);
  stop();
  pros::delay(750);
  Claw.set_value(true);
  pros::delay(100);
	liftHeight = 500;
  driveBack(195);
  pros::delay(1200);
  stop();
}

void Banner2(){
	//pros::Task lift(liftPID);
	Claw.set_value(false);
	goForwardPID(-49);
	BottomClaw.set_value(true);
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
	//pros::Task lift(liftPID);
	Claw.set_value(false);
	fastGoForwardPID(-48);
	BottomClaw.set_value(true);
	fastGoForwardPID(26);
	turnPID(132);
	fastGoForwardPID(42);
	Claw.set_value(true);
	pros::delay(200);
	liftHeight = 500;
	fastGoForwardPID(-57);
}

void Skills(){
	//pros::Task lift(liftPID);
	//liftHeight = -185;
	//Grab alliance goal and get out of corner
	Claw.set_value(false);
	BottomClaw.set_value(true);
	turnPID(50);
	//liftHeight = -5;
	goForwardPID(20);
	turnPID(110);
	goForwardPID(45);
	pros::delay(500);

	//Get first neutral goal
	Claw.set_value(true);
	liftHeight = 800;
	pros::delay(500);

	//Go to bridge and score
	moveToPoint(1.1176, 0.2032);
	liftHeight = 600;
	pros::delay(500);
	Claw.set_value(false);
	pros::delay(1000);

	//Spin around, drop alliance goal, pick up other alliance goal
	goForwardPID(-11);
	pros::delay(500);
	turnPID(0);
	pros::delay(500);
	BottomClaw.set_value(false);
	pros::delay(500);
	liftHeight = 0;
	goForwardPID(9);
	pros::delay(100);
	backLiftHeight = -1200;
	turnPID(180);
	pros::delay(100);
	goForwardPID(-28);
	//BottomClaw.set_value(true);
	backLiftHeight = -100;
	pros::delay(750);
	//turnPID(180);
	goForwardPID(42);
	Claw.set_value(true);

	//Raise arm, score goal
	goForwardPID(-9);
	liftHeight = 620;
	pros::delay(1000);
	turnPID(110);
	goForwardPID(10);
	turnPID(120);
	Claw.set_value(false);

	//Back up, drive to, and pick up next goal
	goForwardPID(-10);
	liftHeight = 0;
	pros::delay(500);
	moveToPoint(0.9144, -1.0922);
	goForwardPID(-5);
	backLiftHeight = -10;
	backToPoint(1.4224, -0.8636);
	BottomClaw.set_value(true);
	pros::delay(500);
	moveToPoint(0.5588, -0.9144);
	moveToPoint(0.1524, -0.9144); //.9652
	pros::delay(500);
	Claw.set_value(true);

	//Align and climb
	backLiftHeight = -150;
	moveToPoint(-0.9144, GPSSensor.get_status().y);
	turnPID(40);
	liftHeight = 650;
	goForwardPID(-36);
	turnPID(-8);
	goForwardPID(13);
	liftHeight = 0;
	turnPID(-10);
	pros::delay(1000);

	balancePID();
}

void RakeBanner(){

}

void RakeTable(){

}
