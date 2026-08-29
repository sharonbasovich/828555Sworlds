#include "main.h"
#include "config.h"
#include "functions.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

#include <cmath>
#include <limits>

// Operator and mechanism state shared with the lightweight PROS tasks below.
bool intakeEnabled = false;
bool outtakeEnabled = false;
int intakeRestartCooldownTicks = 0;
bool intakeRestartPending = false;

enum class WallStakeState : int
{
	bottom = 0,
	load = 1,
	score = 2,
};

WallStakeState wallStakeState = WallStakeState::bottom;

WallStakeState nextWallStakeState(WallStakeState current)
{
	if (current == WallStakeState::bottom) return WallStakeState::load;
	return WallStakeState::score;
}

WallStakeState previousWallStakeState(WallStakeState current)
{
	if (current == WallStakeState::score) return WallStakeState::load;
	return WallStakeState::bottom;
}

bool wallStakePidEnabled = true;
bool wallStakeHoldEnabled = false;

class CalibratedImu : public pros::Imu
{
public:
	CalibratedImu(int port, double headingScale)
		: pros::Imu(port), heading_scale(headingScale) {}

	double get_rotation() const override
	{
		double raw = pros::Imu::get_rotation();
		if (raw == PROS_ERR_F)
			return std::numeric_limits<double>::quiet_NaN();
		return raw * heading_scale;
	}

private:
	double heading_scale;
};

// Empirical heading scale correction measured during repeated full rotations.
constexpr double kImuHeadingScale = 361.5 / 360.0;
CalibratedImu imu(IMU, kImuHeadingScale);

lemlib::Drivetrain drivetrain(&left_mg,					  // left motor group
							  &right_mg,				  // right motor group
							  11,						  // tuned track width, in inches
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  DRIVE_RPM,						  // drivetrain rpm is 450
							  2							  // horizontal drift is 2 (for now)
);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, lemlib::Omniwheel::NEW_2, 0.5);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
							nullptr,				  // no second vertical tracking wheel
							nullptr,				  // no horizontal tracking wheel
							nullptr,				  // no second horizontal tracking wheel
							&imu					  // calibrated inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(9,   // proportional gain (kP)
											  0,   // integral gain (kI)
											  20,  // derivative gain (kD)
											  0,   // anti windup
											  0,   // small error range, in inches
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in inches
											  500, // large error range timeout, in milliseconds
											  20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.2, // proportional gain (kP)
											  0,   // integral gain (kI)
											  28,  // derivative gain (kD)
											  0,   // anti windup
											  0,   // small error range, in degrees
											  0,   // small error range timeout, in milliseconds
											  0,   // large error range, in degrees
											  0,   // large error range timeout, in milliseconds
											  0	   // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,			// drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors				// odometry sensors
);

// PROS lifecycle callbacks. The robot has no disabled-mode behavior or
// competition-time autonomous selector in this configuration.
void disabled() {}

void competition_initialize() {}

void middleMogoBlue()
{
	// === Set Start Position ===
	chassis.setPose(53, -24, 90); // mirrored X and heading

	// === Move to Mobile Goal 1 and Clamp ===
	chassis.moveToPoint(23, -24, 2000);
	pros::delay(1000);
	clamp.extend();
	pros::delay(250);

	// === Turn to Face Center and Move to Middle ===
	chassis.turnToHeading(120, 1000);
	pros::delay(250);
	chassis.moveToPose(11, -10, 120, 2000);
	pros::delay(200);
	rDoinker.extend();
	pros::delay(500);

	// === Go to Second Ring in Middle and Clamp ===
	chassis.turnToHeading(150, 1000);
	pros::delay(250);
	chassis.moveToPoint(6, -8, 2000); // slow
	pros::delay(200);
	lDoinker.extend();
	pros::delay(1000);

	// === Move Back and Align Rings ===
	chassis.moveToPose(43, -32, 90, 2000);
	pros::delay(1000);
	lDoinker.retract();
	rDoinker.retract();
	pros::delay(200);

	// === Move to First Ring and Score on Mogo ===
	chassis.moveToPose(23, -24, 120, 2000);
	intakeForward();
	pros::delay(500);

	// === Score Final 2 Rings ===
	chassis.turnToHeading(0, 2000);
	pros::delay(1000);
	chassis.moveToPose(23, -49, 0, 2000);

	// === Move to Corner ===
	chassis.moveToPose(57, -61, 115, 2000);
	pros::delay(1000);

	// === Leave Corner ===
	chassis.turnToHeading(110, 2000);
	pros::delay(1000);
	clamp.retract();
	chassis.moveToPoint(16.15, -56, 2000);
}

// The anti-jam task is implemented below but is intentionally opt-in: its
// task creation remains disabled in the competition entry points until it is
// revalidated on the current mechanism configuration.
bool antiJamEnabled = true;

void antiJam()
{
	constexpr double kJamVelocityThreshold = 10.0;
	constexpr int kJamReverseDurationMs = 200;
	constexpr int kJamSamplePeriodMs = 1000;
	double previousVelocity = 100.0;

	while (true)
	{
		const double currentVelocity = intakeMotor.get_actual_velocity();
		if (intakeEnabled && antiJamEnabled && !lift.is_extended())
		{
			if (currentVelocity < kJamVelocityThreshold &&
				previousVelocity < kJamVelocityThreshold)
			{
				if (wallStakeState == WallStakeState::load)
				{
					intakeEnabled = false;
					intakeStop();
				}
				else
				{
					intakeBackward();
					pros::delay(kJamReverseDurationMs);
					intakeForward();
				}
			}
		}
		previousVelocity = currentVelocity;
		pros::delay(kJamSamplePeriodMs);
	}
}

void holdPID()
{
	constexpr double kGravityFeedforward = 20.0;
	constexpr double kPositionUnitsToDegrees = 0.01;
	constexpr double kDegreesToRadians = 0.017453;

	while (true)
	{
		if (wallStakeHoldEnabled)
		{
			const double wallAngle = lbRotation.get_position() * kPositionUnitsToDegrees;
			// Holding uses gravity feedforward only; the calculated PID term was
			// never sent to the motor in the competition implementation.
			lbMotor.move(
				std::cos(wallAngle * kDegreesToRadians) * kGravityFeedforward);
		}
		pros::delay(20);
	}
}

void wallPID()
{
	// Setpoints are rotation-sensor centidegrees retained from the tuned
	// competition configuration.
	constexpr double kBottomPosition = 200.0;
	constexpr double kLoadPosition = 170.0;
	constexpr double kScorePosition = 35.0;
	constexpr double kP = 1.6;
	constexpr double kI = 0.0;
	constexpr double kD = 0.5;
	constexpr double kGravityFeedforward = 8.5;
	constexpr double kPositionUnitsToDegrees = 0.01;
	constexpr double kDegreesToRadians = 0.017453;

	double previousError = 0.0;
	double integral = 0.0;

	while (true)
	{
		if (wallStakePidEnabled)
		{
			double targetPosition = kBottomPosition;
			switch (wallStakeState)
			{
			case WallStakeState::bottom:
				targetPosition = kBottomPosition;
				break;
			case WallStakeState::load:
				targetPosition = kLoadPosition;
				break;
			case WallStakeState::score:
				targetPosition = kScorePosition;
				break;
			default:
				break;
			}

			const double wallAngle = lbRotation.get_position() * kPositionUnitsToDegrees;
			const double error = targetPosition - wallAngle;
			integral += error;
			const double derivative = error - previousError;
			const double output = kP * error + kI * integral + kD * derivative;
			// Feedback corrects position error while the cosine term offsets the
			// linkage's changing gravity torque across its travel.
			lbMotor.move(
				output +
				std::cos(wallAngle * kDegreesToRadians) * kGravityFeedforward);
			previousError = error;
		}
		pros::delay(20);
	}
}

constexpr int kRingHoldProximity = 100;

bool holdNextRing = false;
void holdRing()
{
	// When a route requests a hold, stop the intake as soon as the optical
	// sensor sees a ring in the intake path.
	while (true)
	{
		const int holdProximity = ring_color.get_proximity();
		if (holdProximity > kRingHoldProximity)
		{
			if (holdNextRing)
			{
				intakeMotor.move(0);
				holdNextRing = false;
			}
		}
		pros::delay(10);
	}
}

const bool allianceIsRed = kAllianceIsRed;
float hue = -1;
bool colorSortingEnabled = true;
int proximity = 0;
enum class RingColor : int
{
	none = 0,
	red = 1,
	blue = 2,
};

RingColor previousRingColor = RingColor::none;
RingColor ringColor = RingColor::none;

constexpr float kRedHueUpperBound = 25.0;
constexpr float kBlueHueLowerBound = 150.0;
constexpr int kOpticalLedPwm = 100;
constexpr int kOpposingRingDetectionDelayMs = 115;
constexpr int kOpposingRingStopDurationMs = 100;
constexpr int kOpposingRingReverseDurationMs = 200;

void colorSort()
{
	ring_color.set_led_pwm(kOpticalLedPwm);
	previousRingColor = RingColor::none;
	ringColor = RingColor::none;
	pros::delay(10);
	while (true)
	{
		hue = ring_color.get_hue();
		proximity = ring_color.get_proximity();

		if (colorSortingEnabled)
		{
			// These empirically tuned hue/proximity thresholds classify rings
			// only when the optical sensor is close enough to the intake path.
			if ((hue < kRedHueUpperBound) && (proximity > kRingColorProximity))
			{
				ringColor = RingColor::red;
			}
			else if ((hue > kBlueHueLowerBound) && (proximity > kRingColorProximity))
			{
				ringColor = RingColor::blue;
			}
			else
			{
				ringColor = RingColor::none;
			}

			if (ringColor != previousRingColor && ringColor != RingColor::none)
			{
				if (ringColor == RingColor::red && !allianceIsRed)
				{
					pros::delay(kOpposingRingDetectionDelayMs);
					intakeStop();
					pros::delay(kOpposingRingStopDurationMs);
					intakeBackward();
					pros::delay(kOpposingRingReverseDurationMs);
					intakeForward();
				}

				if (ringColor == RingColor::blue && allianceIsRed)
				{
					pros::delay(kOpposingRingDetectionDelayMs);
					intakeStop();
					pros::delay(kOpposingRingStopDurationMs);
					intakeBackward();
					pros::delay(kOpposingRingReverseDurationMs);
					intakeForward();
				}
			}

			previousRingColor = ringColor;
		}
		pros::delay(10);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	lbRotation.set_position(20000);
	pros::delay(10);
	pros::lcd::initialize();
	ring_color.set_integration_time(10);
	pros::delay(10);
	ring_color.disable_gesture();
	pros::delay(10);
	chassis.calibrate(); // calibrate sensors
	pros::delay(10);
	pros::Task screen_task([]()
	{
		while (true)
		{
			pros::lcd::print(0, "intake velocity: %f", intakeMotor.get_actual_velocity());
			pros::lcd::print(1, "hue: %f", hue);
			pros::lcd::print(2, "distance: %d", ring_distance.get());
			pros::lcd::print(3, "proximity: %d", proximity);
			pros::lcd::print(4, "ring color: %d", static_cast<int>(ringColor));
			pros::lcd::print(5, "Wall Angle: %f", lbRotation.get_angle());
			pros::lcd::print(6, "Y: %f", chassis.getPose().y);
			pros::delay(20);
		}
	});
}

void rightSwap()
{
	chassis.setPose(58, 15, 143);

	wallStakePidEnabled = false;
	lbMotor.move(-127);
	pros::delay(500);
	lbMotor.move(0);
	chassis.moveToPoint(41, 38, 1000, {.forwards = false});

	// go to mogo
	chassis.turnToPoint(18, 22, 600, {.forwards = false});
	chassis.moveToPoint(18, 22, 2000, {.forwards = false, .maxSpeed = 50});
	pros::delay(200);
	lbMotor.move(127);
	pros::delay(1000);
	clamp.extend();
	pros::delay(300);
	lbRotation.set_position(20000);
	wallStakePidEnabled = true;

	// get ring 1
	chassis.turnToPoint(16, 45, 600);
	// pros::delay(1000);
	intakeForward();
	chassis.moveToPoint(16, 45, 1000);

	// go corner
	chassis.turnToPoint(65, 78, 500);
	chassis.moveToPoint(65, 78, 2000);

	// drive back then forward then back to get the second ring
	// chassis.moveToPoint(34, 60, 1000, {.forwards = false});
	// chassis.moveToPoint(60, 70, 1000);
	chassis.moveToPoint(34, 60, 1000, {.forwards = false});

	// drive to m2r1 and filter first ring
	chassis.turnToPoint(44, 10, 500);
	pros::delay(200);
	intakeStop();
	pros::delay(200);
	clamp.retract();
	chassis.moveToPoint(44, 10, 800);
	pros::delay(200);
	intakeForward();
	holdNextRing = true;
	lift.extend();
	chassis.moveToPoint(44, 2, 1000, {.maxSpeed = 80});
	// pros::delay(500);
	// lift.retract();
	// clamp m2
	chassis.turnToPoint(17, -24, 500, {.forwards = false});
	chassis.moveToPoint(17, -24, 1000, {.forwards = false, .maxSpeed = 50});
	pros::delay(1200);
	clamp.extend();
	pros::delay(200);
	intakeForward();
	// go to bar
	chassis.moveToPoint(20, 0, 2000, {.maxSpeed = 80});
	// pros::delay(800);
	pros::delay(10000);
}

void rightMidSwap()
{
	chassis.setPose(58, 15, 143);

	wallStakePidEnabled = false;
	lbMotor.move(-127);
	pros::delay(500);
	lbMotor.move(0);
	chassis.moveToPoint(41, 38, 1000, {.forwards = false});

	// go to mogo
	chassis.turnToPoint(18, 22, 600, {.forwards = false});
	chassis.moveToPoint(18, 22, 2000, {.forwards = false, .maxSpeed = 50});
	pros::delay(200);
	lbMotor.move(127);
	pros::delay(1000);
	clamp.extend();
	pros::delay(300);
	lbRotation.set_position(20000);
	wallStakePidEnabled = true;

	// get ring 1
	chassis.turnToPoint(16, 45, 600);
	// pros::delay(1000);
	intakeForward();
	chassis.moveToPoint(16, 45, 1000);
	pros::delay(3000);
	// go corner
	chassis.turnToPoint(65, 78, 500);
	chassis.moveToPoint(65, 78, 2000);

	// drive back then forward then back to get the second ring
	chassis.moveToPoint(34, 60, 1000, {.forwards = false});
	// chassis.moveToPoint(60, 70, 1000);
	// chassis.moveToPoint(34, 60, 1000, {.forwards = false});
	// pros::delay(1500);
	chassis.turnToPoint(19, -6, 800);
	wallStakeState = WallStakeState::score;
	pros::delay(500);
	// go to bar
	chassis.moveToPoint(19, -6, 5000, {.maxSpeed = 40});
	// pros::delay(800);
	pros::delay(10000);
}

void rightAvoidRing()
{
	chassis.setPose(58, 15, 143);

	wallStakePidEnabled = false;
	lbMotor.move(-127);
	pros::delay(500);
	lbMotor.move(0);
	chassis.moveToPoint(41, 38, 1000, {.forwards = false});

	// go to mogo
	chassis.turnToPoint(18, 22, 600, {.forwards = false});
	chassis.moveToPoint(18, 22, 2000, {.forwards = false, .maxSpeed = 50});
	pros::delay(200);
	lbMotor.move(127);
	pros::delay(1000);
	clamp.extend();
	pros::delay(300);
	lbRotation.set_position(20000);
	wallStakePidEnabled = true;

	// get ring 1
	chassis.turnToPoint(16, 45, 600);
	// pros::delay(1000);
	intakeForward();
	chassis.moveToPoint(16, 45, 1000);
	pros::delay(1200);
	// go corner
	chassis.turnToPoint(42, 46, 500);
	chassis.moveToPoint(42, 46, 1000);
	chassis.turnToPoint(65, 78, 500);
	chassis.moveToPoint(65, 78, 3500, {.minSpeed = 127});

	// drive back then forward then back to get the second ring
	chassis.moveToPoint(34, 60, 1000, {.forwards = false, .maxSpeed = 60});
	// chassis.moveToPoint(60, 70, 1000);
	// chassis.moveToPoint(34, 60, 1000, {.forwards = false});
	// pros::delay(1500);
	chassis.turnToPoint(19, -6, 800);
	wallStakeState = WallStakeState::score;
	pros::delay(500);
	// go to bar
	chassis.moveToPoint(19, -6, 5000, {.maxSpeed = 50});
	// pros::delay(800);
	pros::delay(10000);
}

void leftSwap()
{
	chassis.setPose(58, -15, 37);

	wallStakePidEnabled = false;
	lbMotor.move(-127);
	pros::delay(500);
	lbMotor.move(0);
	chassis.moveToPoint(41, -38, 1000, {.forwards = false});

	// go to mogo
	chassis.turnToPoint(18, -22, 600, {.forwards = false});
	chassis.moveToPoint(18, -22, 2000, {.forwards = false, .maxSpeed = 50});
	pros::delay(200);
	lbMotor.move(127);
	pros::delay(1000);
	clamp.extend();
	pros::delay(300);
	lbRotation.set_position(20000);
	wallStakePidEnabled = true;

	// get ring 1
	chassis.turnToPoint(16, -45, 600);
	// pros::delay(1000);
	intakeForward();
	chassis.moveToPoint(16, -45, 1000);

	// go corner
	chassis.turnToPoint(65, -78, 500);
	chassis.moveToPoint(65, -78, 2000);

	// drive back then forward then back to get the second ring
	// chassis.moveToPoint(34, 60, 1000, {.forwards = false});
	// chassis.moveToPoint(60, 70, 1000);
	chassis.moveToPoint(34, -60, 1000, {.forwards = false});

	// drive to m2r1 and filter first ring
	chassis.turnToPoint(44, -10, 500);
	pros::delay(200);
	intakeStop();
	pros::delay(200);
	clamp.retract();
	chassis.moveToPoint(44, -10, 800);
	pros::delay(200);
	intakeForward();
	holdNextRing = true;
	lift.extend();
	chassis.moveToPoint(44, -2, 1000, {.maxSpeed = 80});
	// pros::delay(500);
	// lift.retract();
	// clamp m2
	chassis.turnToPoint(17, 24, 500, {.forwards = false});
	chassis.moveToPoint(17, 24, 1000, {.forwards = false, .maxSpeed = 50});
	pros::delay(1200);
	clamp.extend();
	pros::delay(200);
	intakeForward();
	// go to bar
	chassis.moveToPoint(20, 0, 2000, {.maxSpeed = 80});
	// pros::delay(800);
	pros::delay(10000);
}

void rightTower()
{
	// === Set Start Position ===
	chassis.setPose(-56, -24, 270);

	// === Move to Mobile Goal 1 and Clamp ===
	chassis.moveToPoint(-23, -24, 1000, {.forwards = false, .maxSpeed = 80});
	pros::delay(800);
	clamp.extend();
	pros::delay(50);
	intakeForward();
	pros::delay(250);

	// === Turn to Face Center and Move to Middle ===
	chassis.turnToHeading(40, 250);
	pros::delay(250);
	intakeStop();
	// chassis.moveToPoint(-13, -6, 1250, {.maxSpeed = 40});//-9.5, -11
	chassis.moveToPoint(-15, -10, 1000, {.maxSpeed = 40}); //-9.5, -11

	// pros::delay(1000);
	chassis.turnToHeading(70, 500, {.maxSpeed = 90}); // 400
	// pros::delay(400);
	chassis.moveToPoint(-12, -5, 750, {.maxSpeed = 40}); // 750
	pros::delay(650);
	lDoinker.extend();

	// === Go to Second Ring in Middle and Clamp ===

	pros::delay(400);
	chassis.turnToHeading(52, 700); // 500
	pros::delay(500);
	rDoinker.extend();
	// pros::delay(100);
	// === Move Back and Align Rings ===
	chassis.moveToPoint(-44, -36, 1300, {.forwards = false});
	chassis.turnToHeading(90, 500);
	pros::delay(500);
	lDoinker.retract();
	rDoinker.retract();
	pros::delay(300);
	// === Move to First Ring and Score on Mogo ===
	intakeForward();
	chassis.moveToPoint(-37, -6, 1000, {.forwards = false}); // drive back
	pros::delay(1000);
	// 2. Drive to (-25, -21), exit early so the next turn can start sooner
	chassis.turnToPoint(-25, -23, 350);
	pros::delay(350);
	chassis.moveToPoint(-29, -23, 1250, {.earlyExitRange = 3});

	// === Score Final 2 Rings ==

	// 4. Drive to final position (-23, -55), again with early exit to prevent stall
	chassis.moveToPoint(-23, -51, 2200, {.maxSpeed = 30, .earlyExitRange = 3});
	pros::delay(1000);
	// // === Move to Corner ===
	// chassis.moveToPoint(-37, -44, 500, {.forwards = false});

	// chassis.moveToPoint(-77, -77, 2000, {.minSpeed = 127});
	// pros::delay(1500);
	// chassis.moveToPoint(-50, -52, 1000, {.forwards = false});
	// pros::delay(1000);

	chassis.turnToPoint(-24, 12, 800);
	wallStakeState = WallStakeState::score;
	pros::delay(500);
	chassis.moveToPoint(-20, 12, 5000, {.maxSpeed = 40});
	pros::delay(10000);

	// chassis.moveToPoint(-59, -61, 1000);
	// pros::delay(1000);
	// chassis.moveToPoint(-50, -54, 1000, {.forwards = false});
	// pros::delay(1000);

	// === Leave Corner ===
	// clamp.retract();
	// chassis.moveToPoint(-16.15, -56, 2000);
}

void leftTower()
{
	// === Set Start Position ===
	chassis.setPose(-56, 24, 270);

	// === Move to Mobile Goal 1 and Clamp ===
	chassis.moveToPoint(-23, 24, 1000, {.forwards = false, .maxSpeed = 80});
	pros::delay(800);
	clamp.extend();
	pros::delay(50);
	intakeForward();
	pros::delay(250);

	// === Turn to Face Center and Move to Middle ===
	chassis.turnToHeading(140, 250);
	pros::delay(250);
	intakeStop();
	// chassis.moveToPoint(-13, -6, 1250, {.maxSpeed = 40});//-9.5, -11
	chassis.moveToPoint(-15, 10, 1000, {.maxSpeed = 40}); //-9.5, -11

	// pros::delay(1000);
	chassis.turnToHeading(110, 500, {.maxSpeed = 90}); // 400
	// pros::delay(400);
	chassis.moveToPoint(-12, 5, 750, {.maxSpeed = 40}); // 750
	pros::delay(650);
	rDoinker.extend();

	// === Go to Second Ring in Middle and Clamp ===

	pros::delay(400);
	chassis.turnToHeading(128, 700); // 500
	pros::delay(500);
	lDoinker.extend();
	// pros::delay(100);
	// === Move Back and Align Rings ===
	chassis.moveToPoint(-44, 36, 1300, {.forwards = false});
	chassis.turnToHeading(90, 500);
	pros::delay(500);
	lDoinker.retract();
	rDoinker.retract();
	pros::delay(300);
	// === Move to First Ring and Score on Mogo ===
	intakeForward();
	chassis.moveToPoint(-37, 6, 1000, {.forwards = false}); // drive back
	pros::delay(1000);
	// 2. Drive to (-25, -21), exit early so the next turn can start sooner
	chassis.turnToPoint(-25, 23, 350);
	pros::delay(350);
	chassis.moveToPoint(-29, 23, 1250, {.earlyExitRange = 3});

	// === Score Final 2 Rings ==

	// 4. Drive to final position (-23, -55), again with early exit to prevent stall
	chassis.moveToPoint(-23, 51, 2200, {.maxSpeed = 30, .earlyExitRange = 3});
	pros::delay(1000);
	// === Move to Corner ===
	// chassis.moveToPoint(-37, 44, 500, {.forwards = false});

	// chassis.moveToPoint(-77, 77, 2000, {.minSpeed = 127});
	// pros::delay(1500);
	// chassis.moveToPoint(-50, 52, 1000, {.forwards = false});
	// pros::delay(1000);
	chassis.turnToPoint(-24, -12, 800);
	wallStakeState = WallStakeState::score;
	pros::delay(500);
	chassis.moveToPoint(-20, -12, 5000, {.maxSpeed = 40});
	pros::delay(10000);
	// pros::delay(1000);
	// chassis.moveToPoint(-59, 61, 1000);
	// pros::delay(1000);
	// chassis.moveToPoint(-50, 54, 1000, {.forwards = false});
	// pros::delay(1000);

	// // === Leave Corner ===
	// clamp.retract();
	// chassis.moveToPoint(-16.15, 56, 2000);
}

void rightRingRush()
{
	// Ring-rush route from the positive-y starting corner.
	chassis.setPose(54, 17, 295);
	intakeForward();
	holdNextRing = true;
	rDoinker.extend();
	// rush to ring cluster
	chassis.moveToPoint(15, 38, 1500, {.minSpeed = 127});

	// pull rings backward - one in intake one on doinker
	chassis.moveToPoint(38, 32, 1100, {.forwards = false});
	pros::delay(900);
	rDoinker.retract();
	pros::delay(200);
	// turn to, go to, and clamp mogo
	chassis.turnToPoint(20, 21, 500, {.forwards = false});
	chassis.moveToPoint(20, 21, 1000, {.forwards = false, .maxSpeed = 80});
	pros::delay(800);
	clamp.extend();

	// score 3 rings
	intakeForward();
	chassis.turnToPoint(31, 53, 500);
	chassis.moveToPoint(31, 53, 2000, {.maxSpeed = 40});

	// go to corner rings with motion chaining
	chassis.turnToPoint(41, 53, 500);
	chassis.moveToPoint(41, 53, 500, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.moveToPoint(74, 61, 1800, {.minSpeed = 127});

	// go back and forward for second corner ring
	// chassis.moveToPoint(50, 50, 1000, {.forwards = false});
	// chassis.moveToPoint(65, 65, 1000);
	chassis.moveToPoint(50, 40, 1000, {.forwards = false});

	// chassis.turnToPoint(47, 7, 1000);
	chassis.turnToPoint(47, -4, 700);
	chassis.moveToPoint(47, 25, 500);
	pros::delay(1500);
	wallStakeState = WallStakeState::score;
	chassis.moveToPoint(15, -8, 10000, {.maxSpeed = 40});
	pros::delay(10000);

	// chassis.moveToPoint(48, -1, 2000, {.maxSpeed = 35});
	// pros::delay(100);
	// intakeForward();
	// lift.extend();
	// pros::delay(400);
	// lift.retract();
	// pros::delay(200);
	// intakeForward();

	// // chassis.turnToPoint(49, -4, 600);
	// chassis.turnToHeading(98, 400);
	// intakeForward();

	// chassis.moveToPoint(51, -3, 500);
	// intakeForward();

	// wallStakePidEnabled = false;
	// intakeForward();

	// pros::delay(100);
	// intakeMotor.move(-50);
	// pros::delay(150);
	// intakeMotor.move(0);
	// pros::delay(10);
	// lbMotor.move(-127);
	// pros::delay(10000);
}

void leftOld()
{

	// Earlier ring-rush variant from the negative-y starting corner.
	chassis.setPose(54, -17, 245);
	intakeForward();
	holdNextRing = true;
	lDoinker.extend();
	// rush to ring cluster
	chassis.moveToPoint(15, -38, 1500, {.minSpeed = 127});

	// pull rings backward - one in intake one on doinker
	chassis.moveToPoint(38, -32, 1100, {.forwards = false});
	pros::delay(900);
	lDoinker.retract();
	pros::delay(200);
	// turn to, go to, and clamp mogo
	chassis.turnToPoint(20, -21, 500, {.forwards = false});
	chassis.moveToPoint(20, -21, 1000, {.forwards = false, .maxSpeed = 80});
	pros::delay(800);
	clamp.extend();

	// score 3 rings
	intakeForward();
	chassis.turnToPoint(31, -53, 500);
	chassis.moveToPoint(31, -53, 2000, {.maxSpeed = 40});

	// go to corner rings with motion chaining
	chassis.turnToPoint(41, -53, 500);
	chassis.moveToPoint(41, -53, 500, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.moveToPoint(74, -61, 1800, {.minSpeed = 127});

	// go back and forward for second corner ring
	// chassis.moveToPoint(50, 50, 1000, {.forwards = false});
	// chassis.moveToPoint(65, 65, 1000);
	chassis.moveToPoint(50, -40, 1000, {.forwards = false});

	// chassis.turnToPoint(47, 7, 1000);
	chassis.turnToPoint(47, 4, 700);
	chassis.moveToPoint(47, -25, 500);
	pros::delay(1500);
	wallStakeState = WallStakeState::score;
	chassis.moveToPoint(21, -7, 10000, {.maxSpeed = 60});

	pros::delay(10000);
}

void leftRingRush()
{
	// Active ring-rush route from the negative-y starting corner.
	chassis.setPose(54, -17, 245);
	intakeForward();
	holdNextRing = true;
	lDoinker.extend();
	// rush to ring cluster
	chassis.moveToPoint(15, -38, 1500, {.minSpeed = 127});

	// pull rings backward - one in intake one on doinker
	chassis.moveToPoint(38, -32, 1100, {.forwards = false});
	pros::delay(900);
	lDoinker.retract();
	pros::delay(200);
	// turn to, go to, and clamp mogo
	chassis.turnToPoint(20, -21, 500, {.forwards = false});
	chassis.moveToPoint(20, -21, 1000, {.forwards = false, .maxSpeed = 80});
	pros::delay(800);
	clamp.extend();

	// score 3 rings
	holdNextRing = false;
	intakeForward();
	chassis.turnToPoint(31, -53, 500);
	chassis.moveToPoint(31, -53, 2000, {.maxSpeed = 40});

	// go to corner rings with motion chaining
	chassis.turnToPoint(41, -53, 500);
	chassis.moveToPoint(41, -53, 500, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.moveToPoint(74, -61, 1800, {.minSpeed = 127});

	// go back and forward for second corner ring
	// chassis.moveToPoint(45, -45, 800, {.forwards = false});
	// chassis.moveToPoint(65, -65, 1000, {.maxSpeed = 80});

	chassis.moveToPoint(50, -40, 1000, {.forwards = false});

	// chassis.turnToPoint(47, 7, 1000);
	chassis.turnToPoint(47, 4, 700);
	chassis.moveToPoint(47, -20, 10000);
	pros::delay(10000);
	// chassis.moveToPoint(15, 8, 10000, {.maxSpeed = 40});

	// pros::delay(10000);

	// chassis.moveToPoint(48, 1, 2000, {.maxSpeed = 35});
	// pros::delay(100);
	// 	intakeForward();
	// lift.extend();
	// 	pros::delay(400);
	// 	lift.retract();
	// 	pros::delay(200);
	// 	intakeForward();

	// chassis.turnToPoint(49, -4, 600);
	// lift.retract();
	// chassis.turnToHeading(82, 400);
	// 	intakeForward();

	// chassis.moveToPoint(51, 3, 500);
	// 	intakeForward();

	// wallStakePidEnabled = false;
	// 	intakeForward();

	// pros::delay(100);
	// intakeMotor.move(-50);
	// pros::delay(150);
	// intakeMotor.move(0);
	// pros::delay(10);
	// lbMotor.move(-127);
	// pros::delay(10000);
}

void autonomous()
{
	colorSortingEnabled = true;
	pros::Task ringHoldTask(holdRing);
	pros::Task wallStakeTask(wallPID);
	pros::Task holdStakeTask(holdPID);
	pros::Task colorSortTask(colorSort);

	// This is the active competition route in the checked-in configuration.
	leftRingRush();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
	colorSortingEnabled = true;
	pros::Task ringHoldTask(holdRing);
	pros::Task wallStakeTask(wallPID);
	pros::Task holdStakeTask(holdPID);
	pros::Task colorSortTask(colorSort);
	wallStakePidEnabled = true;

	while (true)
	{
		const int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		const int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(leftY, rightX);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			colorSortingEnabled = false;
		}

		// extend clamp on press
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
		{
			clamp.extend();
		}

		// retract clamp on press
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		{
			clamp.retract();
		}

		// right doinker
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		{
			rDoinker.toggle();
		}

		// left doinker
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			lDoinker.toggle();
		}
		// when a is pressed, toggle between intaking and stopping the intake
		// overrides outtaking when pressed
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			intakeEnabled = !intakeEnabled;
			outtakeEnabled = false;
			if (intakeEnabled)
			{
				intakeForward();
			}
			else
			{
				intakeStop();
			}
		} // activate intake

		// when b is pressed, toggle between outtaking and stopping the intake
		// overrides intaking when pressed
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			outtakeEnabled = !outtakeEnabled;
			intakeEnabled = false;
			if (outtakeEnabled)
			{
				intakeBackward();
			}
			else
			{
				intakeStop();
			}
		}

		// R1 advances the wall-stake mechanism through bottom -> load -> score.
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && wallStakePidEnabled)
		{
			if (wallStakeState == WallStakeState::load)
			{
				intakeMotor.move(-50);
				pros::delay(150);
				intakeMotor.move(0);
				pros::delay(10);
				if (intakeEnabled)
				{
					intakeEnabled = false;
				}
			}
			wallStakeState = nextWallStakeState(wallStakeState);
		}

		// R2 moves the wall-stake mechanism back one state.
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && wallStakePidEnabled)
		{
			if (wallStakeState != WallStakeState::bottom)
			{
				wallStakeState = previousWallStakeState(wallStakeState);
				if (!intakeEnabled)
				{
					intakeRestartCooldownTicks = 20;
					intakeRestartPending = true;
				}
			}
		}

		if (intakeRestartPending)
		{
			// Process the delayed intake restart once per driver-loop tick.
			if (intakeRestartCooldownTicks == 0)
			{
				intakeRestartPending = false;
				intakeForward();
				intakeEnabled = true;
			}
			intakeRestartCooldownTicks--;
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{

			wallStakePidEnabled = !wallStakePidEnabled;
			if (wallStakePidEnabled)
			{
				wallStakeHoldEnabled = false;
				lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			}
			else
			{
				lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			}

			intakeStop();
		}

		if (!wallStakePidEnabled)
		{
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			{
				lbMotor.move(-127 * kLiftManualScale);
				wallStakeHoldEnabled = false;
			}
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
			{
				lbMotor.move(127 * kLiftManualScale);
				wallStakeHoldEnabled = false;
			}
			else
			{
				wallStakeHoldEnabled = true;
			}
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			lift.toggle();
		}

		// Yield so the driver loop and background mechanism tasks remain responsive.
		pros::delay(10);
	}
}
