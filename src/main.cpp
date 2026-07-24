
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robotConfigs.h"

// Functions with Button
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // this handles IMU calibration/reset internally
	// ColorSensor.set_led_pwm(80); //Change if you want
	
	while (imu.is_calibrating()) {
		pros::delay(10);
	}
	
	pros::lcd::print(3, "IMU Ready!");
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
    chassis.setPose(0, 0, 0);
	chassis.moveToPose(0, 100, 90, 4000, {.lead = 0.4, .maxSpeed = 50});
	chassis.moveToPose(25, 100, 180, 4000, {.lead = 0.4, .maxSpeed = 50});
	chassis.moveToPose(0, 8, 0, 2000, {.lead = 0.4, .maxSpeed = 90});
	chassis.moveToPose(0, 0, 0, 2000, {.forwards = false, .maxSpeed = 105});
	chassis.moveToPose(0, 8, 0, 2000, {.lead = 0.4, .maxSpeed = 90});
	chassis.moveToPose(0, 0, 0, 2000, {.forwards = false, .maxSpeed = 105});
	chassis.moveToPose(0, 24, 90, 4000, {.lead = 0.4, .maxSpeed = 50});
	chassis.moveToPose(0, 0, 0, 2000, {.forwards = false, .maxSpeed = 105});

    lemlib::Pose pose = chassis.getPose();
    pros::lcd::print(0, "X: %.2f", pose.x);
    pros::lcd::print(1, "Y: %.2f", pose.y);
    pros::lcd::print(2, "Theta: %.2f", pose.theta);
    // chassis.moveToPoint(0, 10, 2000);
	// chassis.turnToHeading(90, 2000);

}
//Real autonomous program template
//Toggle shift
//Manually measure some of these distances btw
	// 1. Move forward 8 inches
	// 2. Move backward 8 inches at ideal speed
	// 3. Move forward 8 inches
	// 4. Move backward 8 inches at ideal speed
	// 5. Move forward <= 24 inches
	// 6. Turn left 90 degrees 
	// 7. Go forward about <= 20 inches
	// 8. Score yellow pin 
	// Total score: 15 points + 12 point bonus = 27 points

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	bool HoldFunction = false;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		chassis.arcade(dir, turn);

		// Buttons
		// Hold Mode
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {  // If B is pressed, toggle HoldFunction
			HoldFunction = !HoldFunction;
			leftMotors.set_brake_mode(HoldFunction ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
			rightMotors.set_brake_mode(HoldFunction ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
		}
		master.print(0, 0, "Hold Mode: %s", HoldFunction ? "ON" : "OFF");

// 		// Elevator Control
// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
//     ElevatorMotor1.move_velocity(200);
//     ElevatorMotor2.move_velocity(200);
// } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
//     ElevatorMotor1.move_velocity(-200);
//     ElevatorMotor2.move_velocity(-200);
// } else {
//     ElevatorMotor1.move_velocity(0);
//     ElevatorMotor2.move_velocity(0);
// }

		// // Scoring 
		// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		// 	ScoringMotor.move_velocity(150);
		// } 
		// else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		// 	ScoringMotor.move_velocity(-150);
		// } 
		// else {
		// 	ScoringMotor.move_velocity(0);
		// }

// 		// Intake
// 		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
// 			IntakeMotor.move_velocity(200);
// 		} 
// 		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
// 			IntakeMotor.move_velocity(-200);
// 		} 
// 		else {
// 			IntakeMotor.move_velocity(0);
// 		}

// 	// Color sensor for blue case
// 	double color = ColorSensor.get_hue();
// 	if (color >= 235 && color <= 275 && IntakeMotor.get_actual_velocity() > 0) {
// 	    IntakeMotor.move_velocity(-200); // reverse - reject the blue pin
// 	} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
// 	    IntakeMotor.move_velocity(200); // normal intake
// 	} else {
//     IntakeMotor.move_velocity(0); // stop
// }

	// // Color sensor for red case
	// double color = ColorSensor.get_hue();
	// if (color >= 0 && color <= 60 && IntakeMotor.get_actual_velocity() > 0) {
	//     IntakeMotor.move_velocity(-200); // reverse - reject the red pin
	// } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
	//     IntakeMotor.move_velocity(200); // normal intake
	// } else {
	//     IntakeMotor.move_velocity(0); // stop
	// }

	// Just comment color sort for not color sort

		pros::delay(20);
	}
}