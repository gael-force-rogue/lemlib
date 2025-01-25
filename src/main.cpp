#include "main.h"

#include "devices.h"
#include "lemlib/api.hpp"  // IWYU pragma: keep

using namespace lemlib;

Controller controller(E_CONTROLLER_MASTER);

Intake intake(-19);
LiftWithPID lift(-18);
pros::ADIDigitalOut clamp('B');
pros::ADIDigitalOut knocker('A');

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

Chassis chassisForTurns(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
Chassis chassis(drivetrain, linearController, odomAngularController, sensors, &throttleCurve, &steerCurve);

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

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
            pros::lcd::print(3, "Lift: %f", lift.position());           // heading

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

bool antiJamEnabled = true;
void antiJamTaskF() {
    while (true) {
        if (antiJamEnabled) {
            intake.handleAntiJam();
        };
        delay(20);
    }
};

void red_awp() {
    chassis.setPose({0, 0, 0});

    chassisForTurns.turnToHeading(45, 2000, {}, false);

    chassis.moveToPoint(8.5, 2, 2000, {.forwards = true, .maxSpeed = 100});
    exit_condition({8.5, 2}, 3);
    chassisForTurns.turnToHeading(63, 700, {}, false);
    lift.spinToPosition(900, 100);
    delay(500);
    lift.spinToPosition(200, 100);
    chassis.moveToPose(-24, -16, 45, 2000, {.forwards = false, .maxSpeed = 100});
    exit_condition({-24, -16}, 1);
    clamp.set_value(1);
    delay(200);
    chassisForTurns.turnToHeading(180, 1000, {});
    intake.in();

    // Ring 1
    chassis.moveToPoint(-26, -36, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({-26, -36}, 1);
    delay(700);

    // Swing back
    chassis.moveToPose(-20, -20, -110, 3000, {.forwards = false, .lead = 0.1, .maxSpeed = 100});
    exit_condition({-20, -20}, 1);

    // Ring 2
    chassis.moveToPose(-42, -34, -170, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 100});
    exit_condition({-42, -34}, 1);
    delay(500);

    chassis.moveToPose(-42, -42, -180, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 100});
    exit_condition({-42, -42}, 1);
    delay(1000);

    lift.spinToPosition(200, 100);

    antiJamEnabled = false;
    chassis.moveToPose(-30, -20, -135, 3000, {.forwards = false, .lead = 0.1, .maxSpeed = 100});
    exit_condition({-30, -20}, 1);

    chassis.moveToPose(-30, -5, -180, 3000, {.forwards = false, .lead = 0.1, .maxSpeed = 100}, false);
    intake.stop();
    delay(500);
    lift.coast();
};

void skills() {
    float start = pros::millis();
    lift.reset();

    // set pose
    chassis.setPose(-58, 7, -135);

    lift.spinToPosition(900, 100);
    delay(1000);

    // clamp mogo
    chassis.moveToPoint(-45, 26, 3000, {.forwards = false, .maxSpeed = 70}, false);
    // exit_condition({-45, 28}, 1);
    clamp.set_value(1);
    delay(1000);
    lift.spinToPosition(-200, 100);

    // turn to ring
    chassisForTurns.turnToHeading(90, 500);

    intake.in();

    // first ring
    chassis.moveToPoint(-17, 28, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({-17, 28}, 2);

    // turn toward wall
    chassisForTurns.turnToHeading(50, 900, {}, false);

    // checkpoint to 2nd ring
    chassis.moveToPoint(-21, 38, 3000, {.forwards = true, .maxSpeed = 100}, false);

    // go to 2nd ring
    chassis.moveToPoint(20, 53, 3000, {.forwards = true, .maxSpeed = 80});
    exit_condition({20, 53}, 2);
    hardStop();

    // navigate to neutral stake
    chassis.moveToPoint(-13, 45, 5000, {.forwards = true, .maxSpeed = 70}, false);
    lift.handleDrivercontrol(false, true, false);
    lift.reset();
    lift.spinToPosition(0, 100);
    chassisForTurns.turnToHeading(-7, 1000, {}, false);
    lift.spinToPosition(270, 100);

    /// go to netutral stake
    antiJamEnabled = false;
    chassis.moveToPose(-13, 63, -9, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 20}, false);
    lift.spinToPosition(270, 100);
    delay(1100);
    intake.stop();
    delay(500);
    lift.spinToPosition(900, 100);
    delay(500);
    chassis.moveToPoint(-13, 50, 3000, {.forwards = false, .maxSpeed = 100});
    exit_condition({-13, 50}, 2);
    antiJamEnabled = true;
    lift.spinToPosition(0, 100);

    // go to 4th ring
    chassis.turnToHeading(-90, 500);
    intake.in();
    chassis.moveToPoint(-25, 42, 3000, {.forwards = true, .maxSpeed = 40});
    exit_condition({-25, 42}, 2);
    chassisForTurns.turnToHeading(-90, 1000, {}, false);

    // go to 5th ring
    chassis.moveToPose(-90, 42, 0, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 30});
    exit_condition({-90, 42}, 2);

    // swing to 6th ring
    chassis.moveToPose(-54, 58, -60, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 50});
    exit_condition({-54, 58}, 2);

    // Store in corner
    // chassis.moveToPose(-50, 58, 0, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 50}, false);
    // exit_condition({-50, 58}, 2);
    chassis.moveToPoint(-65, 50, 3000, {.forwards = false, .maxSpeed = 100});
    exit_condition({-65, 50}, 2);
    chassis.moveToPose(-74, 51, 45, 3000, {.forwards = false, .maxSpeed = 100});
    exit_condition({-74, 51}, 2);
    clamp.set_value(0);
    delay(500);
    intake.stop();

    float end = pros::millis();
    printf("Quadrant 1: %f\n", end - start);

    /// 2ND QUADRANT
    start = pros::millis();

    // Aim Towards 2nd Quadrant
    chassis.moveToPoint(-60, 40, 3000, {.forwards = true, .maxSpeed = 70});
    exit_condition({-60, 40}, 2);
    chassisForTurns.turnToHeading(180, 1000, {}, false);

    // Clamp
    chassis.moveToPoint(-68, -8, 7000, {.forwards = true, .maxSpeed = 30});
    exit_condition({-68, -8}, 2);
    chassisForTurns.turnToHeading(0, 1000, {}, false);
    chassis.moveToPose(-68, -16, 0, 7000, {.forwards = false, .lead = 0.1, .maxSpeed = 30});
    exit_condition({-68, -16}, 2);
    clamp.set_value(1);
    delay(50);

    // turn toward first ring
    chassisForTurns.turnToHeading(90, 500, {}, false);

    intake.in();

    // first ring
    chassis.moveToPoint(-30, -10, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({-30, -10}, 2);
    chassisForTurns.turnToHeading(155, 700, {}, false);

    hardStop();
    return;

    // transition to 2nd ring
    chassis.moveToPoint(0, -23, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({0, -23}, 2);

    // go to 2nd ring
    chassis.moveToPoint(13, -27, 3000, {.forwards = true, .maxSpeed = 100});
    exit_condition({18, -27}, 2);
    delay(500);

    // go to neutral ring
    chassisForTurns.turnToHeading(-80, 1000, {}, false);
    chassis.moveToPoint(-1, -29, 3000, {.forwards = true, .maxSpeed = 70});
    exit_condition({-1, -27}, 2);

    // turn to neutral stake & score
    chassisForTurns.turnToHeading(180, 1500, {}, false);
    chassis.moveToPoint(-1, -47, 3000, {.forwards = true, .maxSpeed = 80});
    exit_condition({-1, -47}, 1);
    chassisForTurns.turnToHeading(180, 1500, {}, false);
    delay(500);
    chassis.moveToPoint(-1, -39, 3000, {.forwards = false, .maxSpeed = 70});
    exit_condition({-1, -39}, 1);
    chassis.turnToHeading(-90, 1000);

    // go to 3rd ring
    chassis.moveToPoint(-10, -43, 3000, {.forwards = true, .maxSpeed = 70});
    exit_condition({-10, -43}, 2);

    // go to 4th ring
    chassis.moveToPoint(-25, -43, 3000, {.forwards = true, .maxSpeed = 50});
    exit_condition({-25, -43}, 2);

    // go to 5th ring
    chassis.moveToPoint(-52, -43, 3000, {.forwards = true, .maxSpeed = 50});
    exit_condition({-52, -43}, 2);

    // checkpoint for 6th ring
    chassis.moveToPose(-43, -35, 160, 3000, {.forwards = false, .lead = 0.1, .maxSpeed = 30});
    exit_condition({-43, -35}, 2);

    // go to 6th ring
    chassis.moveToPose(-35, -60, 90, 3000, {.forwards = true, .lead = 0.1, .maxSpeed = 50}, false);
    exit_condition({-35, -60}, 2);
    delay(500);

    // store in corner
    clamp.set_value(0);
    chassis.moveToPose(-50, -55, 55, 3000, {.forwards = false, .lead = 0.1, .maxSpeed = 50}, false);
    exit_condition({-50, -55}, 2);

    intake.stop();

    hardStop();
    end = pros::millis();
    printf("Quadrant 2: %f", end - start);
}

void autonomous() {
    float start = pros::millis();
    pros::Task antiJamTask(antiJamTaskF);
    red_awp();
    float end = pros::millis();
    printf("Auton took: %f milliseconds in total", end - start);
}

void opcontrol() {
    int knockerState = 0;
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        intake.handleDrivercontrol(controller.get_digital(E_CONTROLLER_DIGITAL_R2) == 1, controller.get_digital(E_CONTROLLER_DIGITAL_R1) == 1);
        lift.handleDrivercontrol(controller.get_digital(E_CONTROLLER_DIGITAL_L2) == 1,
                                 controller.get_digital(E_CONTROLLER_DIGITAL_L1) == 1,
                                 controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP) == 1);

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
            clamp.set_value(1);
        } else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
            clamp.set_value(0);
        };

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
            knockerState = knockerState == 0 ? 1 : 0;
            knocker.set_value(knockerState);
        };
        delay(10);
    }
}
