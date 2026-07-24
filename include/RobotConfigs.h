#ifndef PROS_ROBOT_CONFIG_H_
#define PROS_ROBOT_CONFIG_H_
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.h"
#include "pros/optical.hpp"

// Motors for Functions
// extern pros::Motor ElevatorMotor1;
// extern pros::Motor ElevatorMotor2;
// extern pros::Motor ScoringMotor;
// extern pros::Motor IntakeMotor;
// extern pros::Optical ColorSensor;


//Drivetrain stuff
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern lemlib::Chassis chassis;
extern pros::Rotation vertical_encoder;
extern pros::Rotation horizontal_encoder;
extern pros::Imu imu;
#endif // PROS_ROBOT_CONFIG_H_