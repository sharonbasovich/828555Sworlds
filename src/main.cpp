#include "main.h"
#include "config.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <queue>
#include "functions.h"

// toggles
bool intake = false;
bool outake = false;
int cooldown = 0;
bool check = false;

double wallAngle;
int state = 0;
double target = 0;
double toutput = 0;
bool doPID = true;
bool doHoldPID = false;
double holdTarget = 0;

class MockIMU : public pros::Imu
{
public:
	MockIMU(int port, double gain)
		: pros::Imu(port), imu_gain(gain) {}

	double get_rotation() const override
	{
		double raw = pros::Imu::get_rotation();
		if (raw == PROS_ERR_F)
			return NAN;
		return raw * imu_gain;
	}

private:
	double imu_gain;
};

MockIMU imu(IMU, 360.0 / 360.0);

//spin right decreases 1.3 degrees
// 358.6, 357.0, 355.6

//360/361.5
// 358.2, 355.5, 354.6

// 361.5/360
//  358.3, 356.1, 354.9

//spin left
// 1.5, 2.9, 4.3 

//360/361.5
//1.3, 3.1, 4.8

// 361.5/360
// 1.2, 2.2, 4.4

lemlib::Drivetrain drivetrain(&left_mg,					  // left motor group
							  &right_mg,				  // right motor group
							  11,						  // 12 inch track width
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  450,						  // drivetrain rpm is 450
							  2							  // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
							nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
							nullptr, // horizontal tracking wheel 1
							nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
							&imu	 // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10,  // proportional gain (kP)
											  0,   // integral gain (kI)
											  3,   // derivative gain (kD)
											  3,   // anti windup
											  1,   // small error range, in inches
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in inches
											  500, // large error range timeout, in milliseconds
											  20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2,   // proportional gain (kP)
											  0,   // integral gain (kI)
											  10,  // derivative gain (kD)
											  3,   // anti windup
											  1,   // small error range, in degrees
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in degrees
											  500, // large error range timeout, in milliseconds
											  0	   // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve
	steer_curve(3,	  // joystick deadband out of 127
				10,	  // minimum output where drivetrain will move out of 127
				1.019 // expo curve gain0
	);

lemlib::ExpoDriveCurve
	throttle_curve(3,	 // joystick deadband out of 127
				   10,	 // minimum output where drivetrain will move out of 127
				   1.019 // expo curve gain
	);

lemlib::Chassis chassis(drivetrain,			// drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors				// odometry sensors
											// &throttle_curve, &steer_curve
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	lbRotation.set_position(20000);
	pros::delay(10);
	ring_color.set_integration_time(10);
	pros::delay(10);
	ring_color.disable_gesture();
	pros::delay(10);
	chassis.calibrate(); // calibrate sensors
	pros::delay(10);
	pros::lcd::initialize(); // initialize brain screen
	pros::delay(10);
	pros::Task screen_task([&]()
						   {
        while (true) {
            // pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
            // pros::lcd::print(2, "Theta: %f", ((chassis.getPose().theta) )); // heading
            // pros::lcd::print(3, "IMU HEADING: %f", imu.get_heading());

			pros::lcd::print(0, "intake velocity: %f", intakeMotor.get_actual_velocity()); //
			pros::lcd::print(1, "lbangle: %d", lbRotation.get_position()/100); //
			pros::lcd::print(2, " state: %d", state);
			pros::lcd::print(3, "dopid: %d", doPID);
			pros::lcd::print(4, "IMULEFT %f", imu.get_heading());
			pros::delay(20);
        } });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {
	while(true) {
		chassis.turnToHeading(180, 2000);
		pros::delay(2000);
		chassis.turnToHeading(0, 2000);
		pros::delay(2000);
	}
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

pros::Controller controller(pros::E_CONTROLLER_MASTER);



bool jam = true;

void antiJam()
{
	float threshold = 40;
	bool previous = intakeMotor.get_actual_velocity();
	while (true)
	{
		if (intake && jam && (state != 1) && (!lift.is_extended()))
		{
			if (intakeMotor.get_actual_velocity() < threshold && previous < threshold)
			{
				intakeBackward();
				pros::delay(200);
				intakeForward();
				pros::delay(1000);
			}
		}
		previous = intakeMotor.get_actual_velocity();
		pros::delay(500);
	}
}

void holdPID()
{

	const double tkP = 0.8; //
	const double tkI = 0;	// 00004;//lower the more perscise
	const double tkD = 0.5; // 4larger the stronger the the kD is so response is quicker
	const double kCos = 20;

	double terror = 0;
	double tprevious_error = 0;
	double tintegral = 0;
	double tderivative = 0;

	while (true)
	{

		if (doHoldPID)
		{
			wallAngle = lbRotation.get_position() / 100;
			terror = holdTarget - wallAngle;
			tintegral += terror;
			tderivative = terror - tprevious_error;
			toutput = tkP * terror + tkI * tintegral + tkD * tderivative;
			// lbMotor.move(toutput);
			lbMotor.move(cos(((lbRotation.get_position() / 100)) * 0.017453) * kCos);
			//  wall_motor.move(cos(((wall_rotation.get_position() / 100) - 40) * 0.017453) * kCos);
			// wall_motor.move(toutput);
			tprevious_error = terror;
		}
		pros::delay(20);
	}
}

void wallPID()
{

	double bottom = 200;
	double load = 167;
	double score = 35;

	const double tkP = 1.4; //
	const double tkI = 0;	// 00004;//lower the more perscise
	const double tkD = 1.0; // 4larger the stronger the the kD is so response is quicker
	const double kCos = 8.5;

	double terror = 0;
	double tprevious_error = 0;
	double tintegral = 0;
	double tderivative = 0;

	while (true)
	{

		if (doPID)
		{
			switch (state)
			{
			case 0:
				target = bottom;
				break;
			case 1:
				target = load;
				break;
			case 2:
				target = score;
				break;

			default:
				target = bottom;
				break;
			}

			wallAngle = lbRotation.get_position() / 100;
			terror = target - wallAngle;
			tintegral += terror;
			tderivative = terror - tprevious_error;
			toutput = tkP * terror + tkI * tintegral + tkD * tderivative;
			// lbMotor.move(toutput);
			lbMotor.move(toutput + cos(((lbRotation.get_position() / 100)) * 0.017453) * kCos);
			//  wall_motor.move(cos(((wall_rotation.get_position() / 100) - 40) * 0.017453) * kCos);
			// wall_motor.move(toutput);
			tprevious_error = terror;
		}
		pros::delay(20);
	}
}

bool isRed = IS_RED;
float hue = -1;
bool sort = true;
int currentRed = 0;
bool hasCurrent = false;
bool hasPrevious = false;
int proximity = 0;
int previousColour = 0;
int previousDist = 0;
int ringColour = 0;
// 0 is none, 1 is red, 2 is blue

int distance = 0;
std::deque<int> intakeQ;
// 1 is a red ring, 2 is a blue ring
void colorSort()
{
	ring_color.set_led_pwm(100);
	intakeQ.clear();
	pros::delay(10);
	while (true)
	{
		hue = ring_color.get_hue();
		distance = ring_distance.get_distance();

		if (sort)
		{
			if ((hue < 30) && (proximity > RING_PROXIMITY))
			{
				ringColour = 1;
			}
			else if ((hue > 100) && (proximity > RING_PROXIMITY))
			{
				ringColour = 2;
			}
			else
			{
				ringColour = 0;
			}

			if (ringColour != previousColour && ringColour != 0)
			{
				intakeQ.push_back(ringColour);
			}

			previousColour = ringColour;

			if (distance < RING_DISTANCE_THRESHOLD && previousDist >= RING_DISTANCE_THRESHOLD && !intakeQ.empty())
			{
				if ((IS_RED && intakeQ.front() == 2) || (!IS_RED && intakeQ.front() == 1))
				{
					pros::delay(40);
					intakeStop();
					pros::delay(400);
					intakeForward();
				}

				intakeQ.pop_front();
			}

			previousDist = distance;
		}
		pros::delay(10);
	}
}

void opcontrol()
{
	pros::Task wallstake_task(wallPID);
	pros::Task holdstake_task(holdPID);
	pros::Task antiJam_task(antiJam);
	// pros::Task sort_task(colorSort);
	bool holdLB = false;
	int preCatch = 1; // 1 is precatch, -1 is needs press again, 0 is in postcatch
	bool buttonPressed = false;

	// loop forever
	while (true)
	{

		// get left y and right x positions
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		// move the robot
		chassis.arcade(leftY, rightX); /// UNCOMMENT

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			sort = false;
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			jam = !jam;
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
			lDoinker.toggle();
		}

		// left doinker
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			rDoinker.toggle();
		}

		// when a is pressed, toggle between intaking and stopping the intake
		// overrides outtaking when pressed
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			intake = !intake;
			outake = 0;
			if (intake)
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
			outake = !outake;
			intake = 0;
			if (outake)
			{
				intakeBackward();
			}
			else
			{
				intakeStop();
			}
		}

		// if lb is in bottom range
		// if (160.0 < lbRotation.get_angle() && lbRotation.get_angle() < 210.0)
		// {
		// when r1 is pressed, moves wall stake mechanism forward by one state
		// unless wall stake mechanism is already fully extended
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && doPID)
		{

			if (state == 1)
			{
				intakeMotor.move(-50);
				pros::delay(150);
				intakeMotor.move(0);
				pros::delay(10);
				if (intake)
				{
					intake = false;
				}
			}
			if (state != 2)
			{
				state++;
			}
		}

		// when r2 is pressed, moves wall stake mechanism backward by one state
		// unless wall stake mechanism is already fully retracted
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && doPID)
		{

			if (state != 0)
			{
				state--;
				if (!intake)
				{
					cooldown = 20;
					check = true;
				}
			}
		}

		if (check)
		{
			if (cooldown == 0)
			{
				check = false;
				intakeForward();
				intake = true;
			}
			cooldown--;
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{

			doPID = !doPID;
			if (doPID)
			{
				doHoldPID = false;
				lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			}
			else
			{
				lbMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			}

			intakeStop();
		}

		if (!doPID)
		{
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			{
				lbMotor.move(-127 * LB_SPEED);
				doHoldPID = false;
			}
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
			{
				lbMotor.move(127 * LB_SPEED);
				doHoldPID = false;
			}
			else
			{
				if(!doHoldPID) {
					holdTarget = lbRotation.get_position() / 100;
				}
				doHoldPID = true;
			}
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			lift.toggle();
		}

		if (check)
		{
			if (cooldown == 0)
			{
				check = false;
				// intakeForward();
				intake = true;
			}
			cooldown--;
		}

		// delay to save resources
		pros::delay(10);
	}
}