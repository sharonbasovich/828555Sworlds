
#pragma once

#include "main.h"

// drive motors
inline constexpr bool kAllianceIsRed = true;
inline constexpr double kLiftManualScale = 0.5; // 0-1 manual wall-stake speed

#define LEFT_FRONT_DRIVE 1
#define LEFT_MIDDLE_DRIVE -2
#define LEFT_BACK_DRIVE -11

#define RIGHT_FRONT_DRIVE -9
#define RIGHT_MIDDLE_DRIVE 10
#define RIGHT_BACK_DRIVE 20

// drive config
#define DRIVE_RPM 450

// intake motors
#define INTAKE_HOOKS -21

// wall stake motors
#define LB_MOTOR 16

// pneumatics
#define LDOINKER 'A'
#define CLAMP 'H'
#define RDOINKER 'B'
#define LIFT 'G'

// sensors
#define LB_ROTATION 8
#define IMU 14
#define VERTICAL_ODOM 13

// Ring sensing
#define RING_DISTANCE 6
inline constexpr int kRingColorProximity = 200;

// Color sorting
#define RING_COLOR 7
inline pros::Controller master(pros::E_CONTROLLER_MASTER);

inline pros::MotorGroup left_mg({LEFT_FRONT_DRIVE, LEFT_MIDDLE_DRIVE, LEFT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup right_mg({RIGHT_FRONT_DRIVE, RIGHT_MIDDLE_DRIVE, RIGHT_BACK_DRIVE}, pros::MotorGearset::blue);

inline pros::Rotation lbRotation(LB_ROTATION);
inline pros::Motor lbMotor(LB_MOTOR);

inline pros::Motor intakeMotor(INTAKE_HOOKS);

inline pros::adi::Pneumatics clamp(CLAMP, false);
inline pros::adi::Pneumatics lDoinker(LDOINKER, false);
inline pros::adi::Pneumatics rDoinker(RDOINKER, false);
inline pros::adi::Pneumatics lift(LIFT, false);

inline pros::Distance ring_distance(RING_DISTANCE);
inline pros::Optical ring_color(RING_COLOR);

inline pros::Rotation vertical_odom(VERTICAL_ODOM);
