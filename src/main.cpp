#include "main.h"

#include "devices.h"
#include "lemlib/api.hpp"  // IWYU pragma: keep

using namespace lemlib;

Controller controller(E_CONTROLLER_MASTER);

Intake intake(-19);
Lift lift(-18);
ADIDigitalOut clamp('B');

MotorGroup leftMotors({20, -16, 1}, MotorGearset::blue);
MotorGroup rightMotors({-9, 17, -10}, MotorGearset::blue);

Imu imu(3);

Rotation horizontalEnc(21);
Rotation verticalEnc(5);
TrackingWheel horizontal(&horizontalEnc, Omniwheel::OLD_275, -1.25);
TrackingWheel vertical(&verticalEnc, Omniwheel::NEW_2, -0.5);

// drivetrain settings
Drivetrain drivetrain(&leftMotors,         // left motor group
                      &rightMotors,        // right motor group
                      14,                  // 10 inch track width
                      Omniwheel::NEW_275,  // using new 4" omnis
                      600,                 // drivetrain rpm is 360
                      20                   // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
ControllerSettings linearController(8,    // proportional gain (kP)
                                    0,    // integral gain (kI)
                                    4,    // derivative gain (kD)
                                    3,    // anti windup
                                    1,    // small error range, in inches
                                    100,  // small error range timeout, in milliseconds
                                    3,    // large error range, in inches
                                    500,  // large error range timeout, in milliseconds
                                    50    // maximum acceleration (slew)
);

// angular motion controller
ControllerSettings angularController(2.5,  // proportional gain (kP)
                                     0,    // integral gain (kI)
                                     14,   // derivative gain (kD)
                                     3,    // anti windup
                                     1,    // small error range, in degrees
                                     100,  // small error range timeout, in milliseconds
                                     3,    // large error range, in degrees
                                     500,  // large error range timeout, in milliseconds
                                     0     // maximum acceleration (slew)
);
ControllerSettings odomAngularController(1.5,  // proportional gain (kP)
                                         0,    // integral gain (kI)
                                         14,   // derivative gain (kD)
                                         3,    // anti windup
                                         1,    // small error range, in degrees
                                         100,  // small error range timeout, in milliseconds
                                         3,    // large error range, in degrees
                                         500,  // large error range timeout, in milliseconds
                                         0     // maximum acceleration (slew)
);

OdomSensors sensors(nullptr,      // vertical tracking wheel
                    nullptr,      // vertical tracking wheel 2, set to nullptr as we don't have a second one
                    &horizontal,  // horizontal tracking wheel
                    nullptr,      // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                    &imu          // inertial sensor
);

ExpoDriveCurve throttleCurve(3,     // joystick deadband out of 127
                             10,    // minimum output where drivetrain will move out of 127
                             1.019  // expo curve gain
);

ExpoDriveCurve steerCurve(3,     // joystick deadband out of 127
                          10,    // minimum output where drivetrain will move out of 127
                          1.019  // expo curve gain
);

// Chassis chassisForTurns(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
Chassis chassis(drivetrain, linearController, odomAngularController, sensors, &throttleCurve, &steerCurve);

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    lift.init();

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
            pros::lcd::print(0, "X: %f", chassis.getPose().x);          // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);          // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);  // heading

            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            delay(50);
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
// ASSET(example_txt);  // '.' replaced with "_" to make c++ happy

void exit_condition(lemlib::Pose target, double exit_range) {
    double distance = fabs(chassis.getPose().distance(target));
    chassis.waitUntil(distance - exit_range);
    chassis.cancelMotion();
}

void hardStop() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.cancelMotion();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
};

void antiJamTaskF() {
    while (true) {
        intake.handleAntiJam();
        delay(20);
    };
};

void autonomous() {
    float start = pros::millis();

    pros::Task antiJamTask(antiJamTaskF);

    // set pose
    chassis.setPose(-58, 7, -135);

    // clamp mogo
    chassis.moveToPoint(-45, 23, 3000, {.forwards = false, .maxSpeed = 70});
    exit_condition({-51, 23}, 2);
    clamp.set_value(1);
    pros::delay(50);

    // turn to ring
    chassis.turnToHeading(90, 500);

    intake.in();

    // first ring
    chassis.moveToPoint(-17, 30, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({-17, 30}, 2);

    // turn toward wall
    chassis.turnToHeading(0, 900);

    // checkpoint to 2nd ring
    chassis.moveToPoint(-20, 32, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({-20, 32}, 2);

    // go to 2nd ring
    chassis.moveToPoint(22, 53, 3000, {.forwards = true, .maxSpeed = 80});
    exit_condition({22, 53}, 2);
    hardStop();

    // navigate to neutral stake
    chassis.moveToPoint(-9, 45, 3000, {.forwards = true, .maxSpeed = 70});
    exit_condition({-9, 45}, 2);
    chassis.turnToHeading(0, 1000);
    hardStop();

    /// go to netutral stake
    chassis.moveToPose(-9, 65, 0, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 70});
    delay(1000);
    chassis.moveToPoint(-9, 48, 3000, {.forwards = false, .maxSpeed = 100});
    exit_condition({-9, 48}, 2);

    // go to 4th ring
    chassis.turnToHeading(-90, 500);
    chassis.moveToPoint(-25, 44, 3000, {.forwards = true, .maxSpeed = 50});
    exit_condition({-25, 44}, 2);

    // go to 5th ring
    chassis.moveToPoint(-70, 44, 3000, {.forwards = true, .maxSpeed = 50});
    exit_condition({-70, 44}, 2);

    // swing to 6th ring
    chassis.moveToPose(-63, 53, 0, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 50});
    exit_condition({-63, 53}, 2);

    chassis.moveToPose(-50, 58, 90, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 50}, false);
    // exit_condition({-50, 58},2);

    delay(500);
    clamp.set_value(0);
    chassis.moveToPoint(-68, 60, 3000, {.forwards = false, .maxSpeed = 100});
    exit_condition({-568, 60}, 2);
    hardStop();

    intake.stop();

    float end = pros::millis();
    printf("Quadrant 1: %f\n", end - start);

    /// 2ND QUADRANT
    start = pros::millis();

    // Aim Towards 2nd Quadrant
    chassis.turnToHeading(170, 1000, {}, false);

    // Move towards clamp
    chassis.moveToPose(-55, 5, 0, 4000, {.forwards = true, .lead = 0.1, .maxSpeed = 100});
    exit_condition({-55, 5}, 2);

    // Clamp
    chassis.turnToHeading(0, 1700, {.maxSpeed = 80}, false);
    chassis.moveToPoint(-52, -10, 5000, {.forwards = false, .maxSpeed = 60});
    exit_condition({-52, -10}, 1);
    clamp.set_value(1);
    delay(50);

    // turn toward first ring
    chassis.turnToHeading(90, 500);

    intake.in();

    // first ring
    chassis.moveToPoint(-26, -9, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({-26, -9}, 2);
    chassis.turnToHeading(135, 700, {}, false);

    // transition to 2nd ring
    chassis.moveToPoint(0, -20, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({0, -20}, 2);

    // go to 2nd ring
    chassis.moveToPoint(18, -27, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({18, -27}, 2);
    delay(500);

    // go to neutral ring
    chassis.turnToHeading(-80, 1000, {}, false);
    chassis.moveToPoint(1, -27, 3000, {.forwards = true, .maxSpeed = 80});
    exit_condition({1, -27}, 1);

    // turn to neutral stake & score
    chassis.turnToHeading(180, 1500, {}, false);
    chassis.moveToPoint(0, -47, 3000, {.forwards = true, .maxSpeed = 80});
    exit_condition({0, -47}, 1);
    delay(500);
    chassis.moveToPoint(1, -39, 3000, {.forwards = false, .maxSpeed = 70});
    exit_condition({1, -39}, 1);
    chassis.turnToHeading(-90, 1000);

    // go to 3rd ring
    chassis.moveToPoint(-10, -39, 3000, {.forwards = true, .maxSpeed = 70});
    exit_condition({-10, -39}, 2);

    // go to 4th ring
    chassis.moveToPoint(-25, -39, 3000, {.forwards = true, .maxSpeed = 50});
    exit_condition({-25, -39}, 2);

    // go to 5th ring
    chassis.moveToPoint(-52, -39, 3000, {.forwards = true, .maxSpeed = 50});
    exit_condition({-52, -39}, 2);

    // checkpoint for 6th ring
    chassis.moveToPose(-43, -35, 160, 3000, {.forwards = false, .lead = 0.1, .maxSpeed = 30});
    exit_condition({-43, -35}, 2);

    // go to 6th ring
    chassis.moveToPose(-35, -60, 90, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 50}, false);
    exit_condition({-35, -50}, 2);
    delay(500);

    // store in corner
    clamp.set_value(0);
    chassis.moveToPose(-50, -55, 55, 3000, {.forwards = false, .lead = 0.1, .maxSpeed = 50}, false);
    exit_condition({-50, -55}, 2);

    hardStop();
    end = pros::millis();
    printf("Quadrant 2: %f", end - start);
}

/**
 * Runs in driver control
 */
void opcontrol() {
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        intake.handleDrivercontrol(controller.get_digital(E_CONTROLLER_DIGITAL_R1) == 1, controller.get_digital(E_CONTROLLER_DIGITAL_R2) == 1);
        lift.handleDrivercontrol(controller.get_digital(E_CONTROLLER_DIGITAL_L1) == 1, controller.get_digital(E_CONTROLLER_DIGITAL_L2) == 1);

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP) == 1) {
            lift.handleMacro();
        }

        pros::delay(10);
    }
}
