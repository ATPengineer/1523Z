#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <iostream>
using namespace std;
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* motor ports
* Port 1 - top left (reversed)
* Port 2 - back left
* Port 3 - front left
* Port 6 - bottom rollers
* Port 7 - top rollers
* Port 8 - front right (reversed)
* Port 9 - back right (reversed)
* Port 10 - top right */

// drive motors

// drivetrain motors
pros::MotorGroup rightMotors({15, 16, -17}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-18, -19, 20}, pros::MotorGearset::blue);

// indexing/intake motors
pros::Motor backroller(-14, pros::MotorGearset::green);
pros::Motor midrollers(-13, pros::MotorGearset::green);
pros::Motor bottomrollers(12, pros::MotorGearset::blue);

// sensors
pros::Optical optical(9);
pros::Imu imu(8);
pros::adi::DigitalOut basket (1);
pros::adi::DigitalOut scraper (2);
pros::adi::DigitalOut lower_basket (3);
pros::adi::DigitalOut instapark (4);
// Inertial Sensor on port 7
/*
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);
*/
bool basket_toggle = false;
bool scraper_toggle = false;
bool lower_toggle = false;
constexpr float RED_HUE = 5;
constexpr float BLUE_HUE = 210;
constexpr float HUE_TOLERANCE = 15;

// Helper void functions to change states
void setIntakeTop() {
    basket.set_value(false);
    bottomrollers.move_velocity(500);
    midrollers.move_velocity(500);
    backroller.move_velocity(-500);
}

void setMidScoring() {
    basket.set_value(true);
    backroller.move_velocity(500);
    bottomrollers.move_velocity(500);
    midrollers.move_velocity(-500);
}

void setHighScoring() {
    basket.set_value(true);
    backroller.move_velocity(500);
    bottomrollers.move_velocity(500);
    midrollers.move_velocity(500);
}

void setLowScoring() {
    basket.set_value(true);
    bottomrollers.move_velocity(-500);
    midrollers.move_velocity(-500);
    backroller.move_velocity(500);
}

void setIdle() {
    basket.set_value(false);
    bottomrollers.move_velocity(0);
    midrollers.move_velocity(0);
    backroller.move_velocity(0);
}




// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14, // 14 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 3.25" omnis
                              343, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// turn to heading
// parameters: degrees, maxspeed, timeout (if timeout = 0, it will move on once the movement is done)
void setheading(float degrees, int maxspeed, int timeout) {
    if (timeout > 0) {
		// turn to specified heading with a timeout
		chassis.turnToHeading(degrees, timeout, {.maxSpeed = maxspeed}, false);
	} else {
			// Turn to the specified heading without a timeout
			chassis.turnToHeading(degrees, 0, {.maxSpeed = maxspeed}, false);

			// Wait until the chassis has stopped moving
			chassis.waitUntilDone();
	}
}

// move forward a specified distance
// perameters: inches, maxspeed, timeout (if timeout = 0, it will move on once the movement is done)
void movefwd(float inches, float maxspeed, int timeout = 0) {
    // Get the current pose of the chassis
    lemlib::Pose currentPose = chassis.getPose();

    // Calculate the target x position
    float targetX = currentPose.x + inches;

	if (timeout > 0) {
    // Move to the target x position with a timeout
    chassis.moveToPose(targetX, currentPose.y, currentPose.theta, timeout, {.maxSpeed = maxspeed});
	} else {
	// Move to the target x position until it reaches the target
	chassis.moveToPose(targetX, currentPose.y, currentPose.theta, 0, {.maxSpeed = maxspeed});

	// Wait until the chassis has stopped moving
	chassis.waitUntilDone();
	}
}

void colorsort(bool color){
    
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}
/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {
    
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * 
 */
void autonomous() {
    setIntakeTop();
    chassis.moveToPose(10, 30, 90, 4000);
    chassis.turnToHeading(135, 4000);
    chassis.moveToPoint(39, 20, 3000);
    chassis.turnToHeading(180, 2000);
    scraper.set_value(true);
    chassis.moveToPoint(39, 0, 2000);
    chassis.moveToPoint(39, 20, 2000);
    chassis.turnToHeading(0, 4000);
    


}
/**
 * Runs in driver control
 */
void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        float r_hue_min = std::fmod(RED_HUE - HUE_TOLERANCE + 360, 360.0f);
		float r_hue_max = std::fmod(RED_HUE + HUE_TOLERANCE, 360.0f);
		float b_hue_min = std::fmod(BLUE_HUE - HUE_TOLERANCE + 360, 360.0f);
		float b_hue_max = std::fmod(BLUE_HUE + HUE_TOLERANCE, 360.0f);

        const int linear_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int lateral_power = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if(basket_toggle){
            basket.set_value(true);    
        }
        else if (!basket_toggle){
            basket.set_value(false);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            setIntakeTop();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            setMidScoring();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            setHighScoring();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            setLowScoring();
        }else{
            setIdle();
        }
		
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            if(basket_toggle){
                basket_toggle = false;
            } else {
                basket_toggle = true;
            }
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            if(scraper_toggle){
                scraper_toggle = false;
            } else {
                scraper_toggle = true;
            }
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            if(lower_toggle){
                lower_toggle = false;
            } else {
                lower_toggle = true;
            }
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            instapark.set_value(true);
        }
        

        // delay to save resources
        pros::delay(25);
    }
}
