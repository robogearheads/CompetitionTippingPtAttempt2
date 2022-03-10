#include "setup/util/misc.h"

void goForward(double amount);
void goBackward(double amount);

void turnRight(double amount);
void turnLeft(double amount);

void balancePID();
void forwardVelocity(double velocity);

void turnPID(double targetTheta);
void preciseTurnPID(double targetTheta);
void goForwardPID(double distance);
void fastGoForwardPID(double distance);

void moveToPoint(double x, double y);
void backToPoint(double x, double y);
void preciseBackToPoint(double x, double y);

void DriverBalancePID();
void driverMovement();
