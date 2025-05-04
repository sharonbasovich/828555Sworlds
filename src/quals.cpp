#include "main.h"
#include "config.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <queue>
#include "functions.h"

void intakeForward()
{
    intakeMotor.move(127);
    pros::delay(10);
}

void intakeBackward()
{
    intakeMotor.move(-127);
    pros::delay(10);
}

void intakeStop()
{
    intakeMotor.move(0);
    pros::delay(10);
}