#ifndef PROS_ROBOT_CONFIG_H_
#define PROS_ROBOT_CONFIG_H_
#include "main.h"
#include "lemlib/api.hpp"


extern pros::Motor ElevatorMotor1;
extern pros::Motor ElevatorMotor2;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern lemlib::Chassis chassis;
extern pros::Rotation vertical_encoder;
extern pros::Rotation horizontal_encoder;
extern pros::Imu imu;

#endif // PROS_ROBOT_CONFIG_H_