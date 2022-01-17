#include "main.h"
#include "setup/util/misc.h"

#include "setup/control/lifts.h"

pros::Controller controller(CONTROLLER_MASTER);

double liftHeight = 0;

void liftPID(){
  double error = 0;
  double prevError = 0;
  double power = 0;

  double kP = 1;
  double kD = 0;

  while(true){
    error = liftHeight - FrontLift.get_position();
		power = error*kP;
    FrontLift.move_velocity(power);
		pros::delay(25);
  }
}
