#include "main.h"

#include "setup/control/base.h"
#include "setup/control/intake.h"
#include "setup/control/lifts.h"

#include "setup/util/misc.h"
#include "setup/util/MovementFunctions.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 // global variables defined some where else - initialize.cpp
 extern int selection;
 extern char* titles[];

 // functions are located autoroutine.cpp
 extern void CenterMiddle();
 extern void CenterTable();
 extern void CenterBanner();
 extern void Table2();
 extern void Skills();
 extern void RakeTable();

void autonomous() {
  pros::lcd::print(2, "running auton");
  FrontLift.set_brake_mode(MOTOR_BRAKE_HOLD);

	switch(selection) {
      case 0 :
          pros::lcd::print(4, "Script#: %d\n", selection);
          pros::lcd::print(5, titles[selection]);
          CenterTable();
       break;

       case 1 :
             pros::lcd::print(4, "Script#: %d\n", selection);
             pros::lcd::print(5, titles[selection]);
             CenterMiddle();
          break;

       case 2 :
             pros::lcd::print(4, "Script#: %d\n", selection);
             pros::lcd::print(5, titles[selection]);
             CenterBanner();
          break;

      case 3 :
             pros::lcd::print(4, "Script#: %d\n", selection);
             pros::lcd::print(5, titles[selection]);
             Table2();
          break;

      case 4 :
             pros::lcd::print(4, "Script#: %d\n", selection);
              pros::lcd::print(5, titles[selection]);
              RakeTable();
          break;

      case 5 :
              pros::lcd::print(4, "Script#: %d\n", selection);
              pros::lcd::print(5, titles[selection]);
              Skills();
          break;

    default :
             // this should never happen as selection is alwasy inialized as 0
             // does the case of '0' is in essence the defualt.
          break;
  }
}
