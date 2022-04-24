#include "main.h"

#include "setup/control/base.h"
#include "setup/control/intake.h"
#include "setup/control/lifts.h"

#include "setup/util/misc.h"

#include "setup/util/MovementFunctions.h"

void WingTask(){
	pros::delay(500);
	LeftWing.set_value(true);
	RightWing.set_value(true);

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

void WinPlatform (){
  //Bring Arm Down


  driveBack(50);
  pros::delay(2000);
  stop();
  pros::delay(1000);
  driveForward();
  pros::delay(2000);
  stop();
}

void CenterTable(){
	pros::Task lift(liftPID);
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
	forwardVelocity(50);
/*
	pros::Task lift(liftPID);
	Claw.set_value(false);
	forwardForDistance(44, 50);
  Claw.set_value(true);
  pros::delay(10);
	liftHeight = 100;
	fastGoForwardPID(-40);
  stop();
*/
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
	pros::Task lift(liftPID);
	//pros::Task lift(liftPID);
	Claw.set_value(false);
	fastGoForwardPID(-47);
	BackClamp.set_value(true);
	fastGoForwardPID(26);
	turnPID(132);
	fastGoForwardPID(42);
	Claw.set_value(true);
	pros::delay(200);
	liftHeight = 200;
	fastGoForwardPID(-57);
	pros::delay(300);
	liftHeight = 0;
	goForwardPID(10);
	pros::delay(500);
	turnPID(93);
	goForwardPID(-16);
	pros::delay(400);
	goForwardPID(15);
	lift.remove();
	turnPID(130);

}

void Skills(){
	pros::Task lift(liftPID);
	//pros::Task lift(liftPID);
	//liftHeight = -185;
	//Grab alliance goal and get out of corner
	Claw.set_value(false);
	turnPID(50);
	//liftHeight = -5;
	goForwardPID(20);
	turnPID(112);
	goForwardPID(45);
	pros::delay(500);

	//Get first neutral goal
	Claw.set_value(true);
	liftHeight = 800;
	pros::delay(750);

	//Go to bridge and score
	moveToPoint(1.1, 0.3); //1.1176 for x, 0.32 for y
	liftHeight = 600;
	pros::delay(1000);
	Claw.set_value(false);
	pros::delay(1000);

	//Spin around, drop alliance goal, pick up other alliance goal
	goForwardPID(-5);
	pros::delay(500);
	turnPID(2); //was 0
	pros::delay(500);
	//double tempX = GPSSensor.get_status().x;
	//double tempY = GPSSensor.get_status().y;
	pros::delay(500);
	liftHeight = 0;
	goForwardPID(10); //was 9
	pros::delay(100);
	//turnPID(180);
	pros::delay(100);
	preciseTurnPID(175);
	goForwardPID(-28);

	//preciseBackToPoint(GPSSensor.get_status().x, 1.3462); //x was 0.889
	pros::delay(750);
	turnPID(180);
	goForwardPID(42); //was 37
	//turnPID(180);
	//moveToPoint(tempX, tempY - 0.0762);
	Claw.set_value(true);

	//Raise arm, score goal
	goForwardPID(-6);
	liftHeight = 600;
	pros::delay(1250);
	turnPID(110);
	goForwardPID(5); //was 5
	turnPID(130); //was 135
	Claw.set_value(false);

	//Back up, drive to, and pick up next goal
	goForwardPID(-10); //was -10
	liftHeight = 0;
	preciseTurnPID(180);
	//moveToPoint(0.9144, -1.2954); //Avi - changing to PID
	goForwardPID(75);
	//goForwardPID(-5);
	backToPoint(0.9906, -0.8509); //was 1.49, -0.8
	pros::delay(200);
	fastGoForwardPID(16); //was 20 then 30
	pros::delay(500);
	//moveToPoint(0.5, -0.9398); //was 0.6096, -.9398 (Removed by Avi)
	moveToPoint(0.1524, -0.9); //y was 0.-9144
	pros::delay(500);
	Claw.set_value(true);

	//Align and climb
	moveToPoint(-0.9144, GPSSensor.get_status().y);
	turnPID(60); //was 40
	liftHeight = 500;
	goForwardPID(-33); //was like -34.5
	turnPID(0);
	goForwardPID(13);
	liftHeight = 0;
	pros::delay(1000);
	//turnPID(-10);
	balancePID();
}

void RakeBanner(){
	pros::delay(3000);
}

void RakeTable(){
	pros::Task wings(WingTask);
	fastGoForwardPID(39); //was 34
	wings.remove();
	pros::delay(75);
	LeftWing.set_value(false);
	RightWing.set_value(false);
	pros::delay(150); //was 450
	fastGoForwardPID(-30); //was -25
	/*
	LeftWing.set_value(true);
	RightWing.set_value(true);
	pros::delay(200);
	fastGoForwardPID(-5);
	//pros::delay(750);
	LeftWing.set_value(false);
	RightWing.set_value(false);
	pros::delay(100);
	goForwardPID(16);
	pros::delay(200);
	preciseTurnPID(-94);
	goForwardPID(-36);
	pros::delay(650);
	goForwardPID(33.5);
	turnPID(-52);
	Claw.set_value(false);
	goForwardPID(10);
	Claw.set_value(true);
	//goForwardPID(12);
	turnPID(-110);
	goForwardPID(6);
`*/
	//goForwardPID(-16);
}
