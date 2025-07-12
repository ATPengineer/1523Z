#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

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
pros::MotorGroup leftMotors({1, -2, -3},
                            pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-10, 9, 8}, pros::MotorGearset::blue);

// rollers
pros::Motor bottomrollers(6, pros::MotorGearset::blue, pros::MotorUnits::degrees); // using degrees as encoder units
pros::Motor toprollers(7, pros::MotorGearset::blue, pros::MotorUnits::degrees); // using degrees as encoder units

// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14, // 14 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              480, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
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
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
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
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			bottomrollers.move_velocity(500); // move bottom rollers at full speed
			toprollers.move_velocity(-500); // move top rollers at full speed
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			bottomrollers.move_velocity(500); // move bottom rollers at full speed
			toprollers.move_velocity(500); // move top rollers at full speed
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			bottomrollers.move_velocity(-500); // move bottom rollers at full speed in reverse
			toprollers.move_velocity(-500); // move top rollers at full speed in reverse
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			bottomrollers.move_velocity(-500); // move bottom rollers at full speed in reverse
			toprollers.move_velocity(0); // stop top rollers
		} else
		
		{
			bottomrollers.move_velocity(0); // stop bottom rollers
			toprollers.move_velocity(0); // stop top rollers
		}
        // move the robot
        chassis.curvature(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}
