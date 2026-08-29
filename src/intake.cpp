#include "main.h"
#include "config.h"
#include "functions.h"

namespace
{
// Intake output is deliberately capped below full PROS command range in the
// tuned competition configuration.
constexpr double kIntakePower = 127.0 * 0.9;
}

void intakeForward()
{
	intakeMotor.move(kIntakePower);
	pros::delay(10);
}

void intakeBackward()
{
	intakeMotor.move(-kIntakePower);
	pros::delay(10);
}

void intakeStop()
{
	intakeMotor.move(0);
	pros::delay(10);
}
