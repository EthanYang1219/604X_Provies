/**
 * \file main.cpp
 *
 * Team 604X — VEX Over Under (2023-2024), BC Provincial Championship.
 *
 * This file owns the robot configuration: every motor, sensor, and
 * pneumatic, plus the LemLib chassis (drivetrain geometry, PID tuning,
 * and odometry sensors). It also implements the PROS competition
 * callbacks. The autonomous routines themselves live in
 * src/Autonomous.cpp and are selected inside autonomous() below.
 */

#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "autons.hpp"

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------
Controller controller(pros::E_CONTROLLER_MASTER);

// ---------------------------------------------------------------------------
// Drivetrain motors — 6-motor tank drive, blue (600 RPM) cartridges
// geared to 450 RPM on 2.75" omni wheels
// ---------------------------------------------------------------------------
Motor leftFrontMotor(20, pros::E_MOTOR_GEARSET_06, true);   // port 20, reversed
Motor leftMidMotor(16, pros::E_MOTOR_GEARSET_06, true);     // port 16, reversed
Motor leftBackMotor(10, pros::E_MOTOR_GEARSET_06, true);    // port 10, reversed
Motor rightFrontMotor(17, pros::E_MOTOR_GEARSET_06, false); // port 17
Motor rightMidMotor(14, pros::E_MOTOR_GEARSET_06, false);   // port 14
Motor rightBackMotor(3, pros::E_MOTOR_GEARSET_06, false);   // port 3

// ---------------------------------------------------------------------------
// Subsystem motors
// ---------------------------------------------------------------------------
Motor catapult(8); // match-load catapult, re-cocked via the rotation sensor
Motor intake(1);   // triball intake; negative = intake, positive = outtake

// Motor groups used by LemLib for the left and right halves of the drive
MotorGroup leftMotors({leftFrontMotor, leftBackMotor, leftMidMotor});
MotorGroup rightMotors({rightFrontMotor, rightBackMotor, rightMidMotor});

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
Imu inertial_sensor(21); // inertial sensor for heading

// Vertical tracking wheel: 2.75" omni on a rotation sensor, mounted
// 0.25" from the tracking center, 1:1 gearing
pros::Rotation vertTracking(15, false);
lemlib::TrackingWheel vertical(&vertTracking, lemlib::Omniwheel::NEW_275, 0.25, 1);

Rotation rotation_sensor(12); // catapult position sensor (re-cock angle)

// ---------------------------------------------------------------------------
// Pneumatics (3-wire ADI ports)
// ---------------------------------------------------------------------------
ADIDigitalOut wings(8);      // front wings — widens the robot for pushing triballs
ADIDigitalOut hangpiston(6); // hang mechanism release (also used as a plow in skills)
ADIDigitalOut backwings(7);  // back wings — corner descores and match-load flicks

// ---------------------------------------------------------------------------
// LemLib chassis configuration
// ---------------------------------------------------------------------------

// Drivetrain geometry
lemlib::Drivetrain drivetrain(
    &leftMotors,               // left motor group
    &rightMotors,              // right motor group
    11.15,                     // track width in inches
    lemlib::Omniwheel::NEW_275,// 2.75" omni wheels
    450,                       // drivetrain RPM
    8                          // chase power (higher = tighter pure-pursuit curves)
);

// Lateral (forward/backward) PID controller
lemlib::ControllerSettings linearController(
    15,  // proportional gain (kP)
    0,   // integral gain (kI)
    10,  // derivative gain (kD)
    3,   // anti-windup
    1,   // small error range, in inches
    100, // small error range timeout, in milliseconds
    3,   // large error range, in inches
    250, // large error range timeout, in milliseconds
    20   // maximum acceleration (slew)
);

// Angular (turning) PID controller
lemlib::ControllerSettings angularController(
    1.7, // proportional gain (kP)
    0,   // integral gain (kI)
    8,   // derivative gain (kD)
    3,   // anti-windup
    1,   // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3,   // large error range, in degrees
    250, // large error range timeout, in milliseconds
    20   // maximum acceleration (slew)
);

// Odometry sensors: one vertical tracking wheel + the IMU.
// The remaining slots are nullptr because the robot has no second vertical
// or horizontal tracking wheel.
lemlib::OdomSensors sensors(
    &vertical,        // vertical tracking wheel 1
    nullptr,          // vertical tracking wheel 2
    nullptr,          // horizontal tracking wheel 1
    nullptr,          // horizontal tracking wheel 2
    &inertial_sensor  // inertial sensor
);

// The chassis object used by every autonomous routine (extern'd in autons.hpp)
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize the brain screen
    chassis.calibrate();     // calibrate the IMU and tracking wheel

    // Background task: print the robot's live pose to the brain screen and
    // stream it over the telemetry sink for debugging/tuning.
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50); // delay to save resources
        }
    });
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
 * mode.
 *
 * To change which routine runs, swap the call below for any routine
 * declared in include/autons.hpp (implemented in src/Autonomous.cpp):
 *   farSideAuton()      - safe far-side route (ran at Provincials)
 *   awpAuton()          - win-point route
 *   skillsAuton()       - catapult volley + field-crossing skills route
 *   oneMinSkillsAuton() - one-minute skills variant
 */
void autonomous() {
    farSideAuton();
}

/**
 * Runs the operator control code.
 *
 * Driver controls (arcade, single stick pair):
 *   Left stick Y   - forward/backward
 *   Right stick X  - turning (scaled to 80% for finer control)
 *   L1 (hold)      - intake
 *   R1 (hold)      - outtake
 *   L2 (toggle)    - front wings
 *   R2 (toggle)    - back wings
 *   A  (toggle)    - hang piston
 *   Y  (hold)      - fire catapult manually
 *
 * The catapult auto-reloads: whenever the rotation sensor reads below the
 * cocked angle (3200 centidegrees), the catapult motor keeps pulling down
 * until the slip-gear is set.
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    rotation_sensor.reset();
    bool toggle = false; // (unused)
    bool tog = false;    // hang piston toggle state
    bool hang = false;   // front wings toggle state
    bool x = false;      // back wings toggle state
    while (true) {
        Controller master(pros::E_CONTROLLER_MASTER);
        int yaxis = master.get_analog(ANALOG_LEFT_Y);
        int xaxis = 0.8 * master.get_analog(ANALOG_RIGHT_X); // dampen turning

        Powerdrive(yaxis, xaxis);

        // Catapult: manual fire on Y, otherwise auto-reload to the cocked angle
        if (master.get_digital(DIGITAL_Y)) {
            catapult = 127;
        } else {
            if ((rotation_sensor.get_angle() < 3200)) {
                catapult = 127;
            } else {
                catapult = 0;
            }
        }

        // Front wings toggle
        if (master.get_digital_new_press(DIGITAL_L2)) {
            hang = !hang;
            wings.set_value(hang);
        }

        // Intake / outtake (hold)
        if (master.get_digital(DIGITAL_L1) == true) {
            intake = 117;
        } else if (master.get_digital(DIGITAL_R1) == true) {
            intake = -127;
        } else {
            intake = 0;
        }

        // Hang piston toggle
        if (master.get_digital_new_press(DIGITAL_A) == true) {
            tog = !tog;
            hangpiston.set_value(tog);
        }

        // Back wings toggle
        if (master.get_digital_new_press(DIGITAL_R2)) {
            x = !x;
            backwings.set_value(x);
        }

        delay(20); // standard control-loop delay
    }
}
