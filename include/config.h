

#include "pros/misc.hpp"
#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// drive motors
#define IS_RED true
#define LB_SPEED 0.5 // 0-1 the speed of the lb in manual mode

#define LEFT_FRONT_DRIVE 1
#define LEFT_MIDDLE_DRIVE -2
#define LEFT_BACK_DRIVE -11

#define RIGHT_FRONT_DRIVE -9
#define RIGHT_MIDDLE_DRIVE 10
#define RIGHT_BACK_DRIVE 20

// drive config
#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06
#define WHEEL_DIAMETER 3.25
#define DRIVE_RPM 450

// intake motors
#define INTAKE_HOOKS -21

// wall stake motors
#define LB_MOTOR 16

// pneumatics
#define LDOINKER 'B'
#define CLAMP 'H'
#define RDOINKER 'A'
#define LIFT 'G' // h

// sensors
#define LB_ROTATION 8
#define IMU 12
#define VERTICAL_ODOM 16
#define HORIZONTAL_ODOM -13

// ring hold
#define RING_DISTANCE 7
//**IN MILLIMETERS** the value that the ring must be closer than to be detected
#define RING_DISTANCE_THRESHOLD 100
#define RING_PROXIMITY 100

// color sort
#define RING_COLOR 2
//*IN MILLISECONDS* the time that it takes for the ring to reach the top of the hooks from when the color sensor detects it*/
#define COLOR_TIME 500
inline pros::Controller master(pros::E_CONTROLLER_MASTER);

inline pros::Motor left_front_drive(LEFT_FRONT_DRIVE);
inline pros::Motor left_middle_drive(LEFT_MIDDLE_DRIVE);
inline pros::Motor left_back_drive(LEFT_BACK_DRIVE);
inline pros::Motor right_front_drive(RIGHT_FRONT_DRIVE);
inline pros::Motor right_middle_drive(RIGHT_MIDDLE_DRIVE);
inline pros::Motor right_back_drive(RIGHT_BACK_DRIVE);

inline pros::MotorGroup left_mg({LEFT_FRONT_DRIVE, LEFT_MIDDLE_DRIVE, LEFT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup right_mg({RIGHT_FRONT_DRIVE, RIGHT_MIDDLE_DRIVE, RIGHT_BACK_DRIVE}, pros::MotorGearset::blue);

inline pros::Rotation lbRotation(LB_ROTATION);
inline pros::Motor lbMotor(LB_MOTOR);

inline pros::Motor intakeMotor(INTAKE_HOOKS);

// inline pros::Imu imu(IMU);

inline pros::adi::Pneumatics clamp(CLAMP, false);
inline pros::adi::Pneumatics lDoinker(LDOINKER, false);
inline pros::adi::Pneumatics rDoinker(RDOINKER, false);
inline pros::adi::Pneumatics lift(LIFT, false);

inline pros::Distance ring_distance(RING_DISTANCE);
inline pros::Optical ring_color(RING_COLOR);

inline pros::Rotation vertical_odom(VERTICAL_ODOM);
inline pros::Rotation horizontal_odom(HORIZONTAL_ODOM);
