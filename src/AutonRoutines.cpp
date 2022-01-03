
#include "main.h"

#include "setup/control/base.h"
#include "setup/control/intake.h"
#include "setup/control/lifts.h"

#include "setup/util/misc.h"

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
  driveForwardFast();
  pros::delay(1200);
  stop();
  pros::delay(750);
  Claw.set_value(true);
  pros::delay(100);
	driveBack(185);
  pros::delay(1200);
  stop();
}

void CenterBanner(){
	driveForwardFast();
  pros::delay(1200);
  stop();
  pros::delay(750);
  Claw.set_value(true);
  pros::delay(100);
  driveBack(195);
  pros::delay(1200);
  stop();
}
