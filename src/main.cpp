
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robotConfigs.h"

double maximumtime = 1.5; // Variable to track elapsed time
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
	ElevatorMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	ElevatorMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
    pros::lcd::print(3, "Calibrating IMU...");
    imu.reset();

    int timeout = 3000;
    int waited = 0;
    while (imu.is_calibrating() && waited < timeout) {
        pros::delay(10);
        waited += 10;
    }

    if (imu.is_calibrating()) {
        pros::lcd::print(3, "IMU FAILED to calibrate!");
        return;
    }

    // Continuously print the current angle before the turn
    for (int i = 0; i < 150; i++) { // ~3 seconds at 20ms
        pros::lcd::print(3, "Heading: %.2f", imu.get_heading());
        pros::delay(20);
    }

    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 3000);

    // Continuously print the angle after the turn too, so you can check accuracy
    for (int i = 0; i < 150; i++) {
        pros::lcd::print(3, "Final Heading: %.2f", imu.get_heading());
        pros::delay(20);
    }
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
	bool ElevatorResist = false;
	bool WasL1pressed = false;
	
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



		// Elevator Control
		
if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
	ElevatorMotor1.brake();
	ElevatorMotor2.brake();
    ElevatorMotor1.move_velocity(600);
    ElevatorMotor2.move_velocity(-600);
	pros::lcd::print(3, "Elevator Up");

} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
	ElevatorMotor1.brake();
	ElevatorMotor2.brake();
    ElevatorMotor1.move_velocity(-600);
    ElevatorMotor2.move_velocity(600);
	pros::lcd::print(3, "Elevator Down");
}
else {
	ElevatorMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	ElevatorMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}


// Roller claw
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

	// Just comment color sort for no color sort

	//Elevator function
	
		pros::delay(20);
	}
}