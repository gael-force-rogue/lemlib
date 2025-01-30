#include "main.h"

#include "devices.h"
#include "lemlib/api.hpp"  // IWYU pragma: keep

using namespace lemlib;

Controller controller(E_CONTROLLER_MASTER);

Intake intake(-19);
LiftWithPID lift(-18, 14);
pros::ADIDigitalOut clamp('B');
pros::ADIDigitalOut knocker('A');
pros::Optical optical(15);

MotorGroup leftMotors({20, -16, 1}, MotorGearset::blue);
MotorGroup rightMotors({-9, 8, -10}, MotorGearset::blue);

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
                                     10,   // derivative gain (kD)
                                     3,    // anti windup
                                     2,    // small error range, in degrees
                                     200,  // small error range timeout, in milliseconds
                                     5,    // large error range, in degrees
                                     400,  // large error range timeout, in milliseconds
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
    lift.reset();
    optical.set_integration_time(20);
    optical.set_led_pwm(100);

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            delay(50);
        }
    });
};

void disabled() {}
void competition_initialize() {};

void exit_condition(Pose target, double exit_range) {
    double distance = fabs(chassis.getPose().distance(target));
    chassis.waitUntil(distance - exit_range);
    chassis.cancelMotion();
};

void hardstop() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.cancelMotion();
    chassis.arcade(0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
};

enum IntakeControllerState {
    IN,
    OUT,
    STOP,
    STOP_ON_RED,
    STOP_ON_BLUE,
    REVERSE_ON_RED,
    REVERSE_ON_BLUE
};
bool antiJamEnabled = true;
bool intakeControllerEnabled = true;
IntakeControllerState intakeState = STOP;
void intakeControllerStateTaskF() {
    while (intakeControllerEnabled) {
        auto hue = optical.get_hue();
        bool isBlue = hue > 200 && hue < 260;
        bool isRed = hue < 15;
        // printf("Hue: %f %f %f\n", hue, isBlue, isRed);

        if (antiJamEnabled) {
            intake.handleAntiJam();
        };

        switch (intakeState) {
            case IN:
                intake.in();
                break;
            case OUT:
                intake.out();
                break;
            case STOP:
                intake.stop();
                break;
            case STOP_ON_RED:
                if (isRed) {
                    intake.stop();
                }
                break;
            case STOP_ON_BLUE:
                if (isBlue) {
                    intake.stop();
                };
                break;
            case REVERSE_ON_RED:
                if (isRed) {
                    intake.out();
                    delay(1000);
                } else {
                    intake.in();
                };
                break;
            case REVERSE_ON_BLUE:
                if (isBlue) {
                    intake.out();
                    delay(1000);
                } else {
                    intake.in();
                };
                break;
        };

        delay(10);
    };
};

void red_sig_awp() {
    // Turn To Alliance & Score
    chassisForTurns.swingToHeading(70, DriveSide::RIGHT, 700, {.maxSpeed = 100}, false);
    lift.spinToPosition(950);
    delay(700);
    lift.spinToPosition(-30);
    delay(500);

    // Clamp
    const Pose clampPose = {-23, -9.5, 60};
    chassis.moveToPose(clampPose.x, clampPose.y, clampPose.theta, 3000, {.forwards = false, .lead = 0.3, .maxSpeed = 70});
    exit_condition({clampPose.x, clampPose.y}, 1);
    clamp.set_value(1);
    lift.spinToPosition(-30);
    delay(100);

    // Ring 1
    chassisForTurns.turnToHeading(220, 800, {.maxSpeed = 70}, false);
    lift.hold();
    intakeState = IN;
    chassis.moveToPose(-39, -25, -170, 900, {.forwards = true, .lead = 0.2, .maxSpeed = 80}, false);
    exit_condition({-39, -25}, 1);
    delay(700);

    // Ring 2
    chassis.moveToPose(-38, -36, -185, 400, {.forwards = true, .lead = 0.1, .maxSpeed = 80});
    exit_condition({-38, -36}, 1);
    delay(500);

    // Ring 3
    intakeState = REVERSE_ON_BLUE;
    chassis.moveToPose(-36, -25, -170, 700, {.forwards = false, .lead = 0.1, .maxSpeed = 80}, false);
    chassis.moveToPoint(-23, -30, 2000, {.forwards = true, .maxSpeed = 80});
    exit_condition({-23, -30}, 1);

    // Line up for ring 4
    chassis.moveToPose(2, 10, 0, 3000, {.forwards = true, .lead = 0.3, .maxSpeed = 127});
    exit_condition({2, 10}, 1);
    clamp.set_value(0);

    // Ring 4
    intake.in();
    intakeState = STOP_ON_RED;
    chassis.moveToPose(2, 44, 0, 1000, {.forwards = true, .maxSpeed = 90});
    exit_condition({2, 44}, 1);
    delay(700);

    // Turn to Mogo
    chassisForTurns.turnToHeading(90, 700, {.maxSpeed = 100}, false);
    intake.stop();

    // Go to mogo
    chassis.moveToPose(-20, 47, 90, 600, {.forwards = false, .lead = 0.3, .maxSpeed = 70});
    exit_condition({-20, 47}, 1);
    delay(500);
    clamp.set_value(1);
    delay(200);
    intakeState = IN;

    antiJamEnabled = false;

    // Standalone
    chassisForTurns.turnToHeading(0, 600, {.maxSpeed = 100}, false);
    chassis.moveToPose(-26, 75, 0, 1000, {.forwards = true, .lead = 0.1, .maxSpeed = 127});
    exit_condition({-26, 75}, 1);

    // Ladder
    lift.spinToPosition(250);
    chassis.turnToHeading(20, 200, {.maxSpeed = 100});
    chassis.moveToPoint(-35, 40, 2000, {.forwards = false, .maxSpeed = 127});
    exit_condition({-35, 40}, 3);
    chassis.moveToPose(-45, 10, 45, 2000, {.forwards = false, .lead = 0.2, .maxSpeed = 127});
    exit_condition({-45, 10}, 1);
};

void blue_sig_awp() {
    // Turn To Alliance & Score
    chassisForTurns.swingToHeading(-67, DriveSide::LEFT, 700, {.maxSpeed = 100}, false);
    lift.spinToPosition(950);
    delay(700);
    // lift.spinToPosition(-30);
    // delay(500);

    // Clamp
    const Pose clampPose = {26, -15, -60};
    chassis.moveToPose(clampPose.x, clampPose.y, clampPose.theta, 3000, {.forwards = false, .lead = 0.3, .maxSpeed = 70});
    exit_condition({clampPose.x, clampPose.y}, 1);
    clamp.set_value(1);
    lift.spinToPosition(-30);
    delay(100);

    // Ring 1
    intakeState = IN;
    chassisForTurns.turnToHeading(-220, 800, {.maxSpeed = 70}, false);
    lift.hold();
    chassis.moveToPose(45, -33, 170, 1200, {.forwards = true, .lead = 0.2, .maxSpeed = 80}, false);
    antiJamEnabled = false;
    delay(300);

    // Ring 2
    chassis.moveToPose(45, -44, 180, 1200, {.forwards = true, .lead = 0.3, .maxSpeed = 80});
    exit_condition({44, -44}, 1);
    delay(500);

    // Ring 3
    chassis.moveToPose(43, -32, 175, 900, {.forwards = false, .lead = 0.2, .maxSpeed = 80}, false);
    chassis.moveToPoint(23, -40, 2000, {.forwards = true, .maxSpeed = 80});
    exit_condition({23, -37}, 1);

    // Line up for ring 4
    chassis.moveToPose(2, 10, 0, 3000, {.forwards = true, .lead = 0.3, .maxSpeed = 100});
    exit_condition({2, 10}, 1);
    clamp.set_value(0);

    // Ring 4
    intake.in();
    intakeState = STOP_ON_BLUE;
    chassis.moveToPose(2, 36, 0, 900, {.forwards = true, .maxSpeed = 90});
    exit_condition({2, 36}, 1);
    // delay(700);

    // Turn to Mogo
    chassisForTurns.turnToHeading(-90, 600, {.maxSpeed = 110}, false);

    // Go to mogo
    antiJamEnabled = true;
    chassis.moveToPose(26, 28, -90, 1000, {.forwards = false, .lead = 0.2, .maxSpeed = 80});
    exit_condition({26, 28}, 1);
    delay(100);
    clamp.set_value(1);
    delay(200);

    // Standalone
    chassisForTurns.turnToHeading(0, 600, {.maxSpeed = 100}, false);
    intakeState = IN;
    intake.in();
    chassis.moveToPose(20, 63, 0, 1000, {.forwards = true, .lead = 0.1, .maxSpeed = 127});
    exit_condition({20, 63}, 1);
    delay(200);

    // Ladder
    lift.spinToPosition(400);
    chassis.moveToPose(32, 10, 0, 3000, {.forwards = false, .lead = 0.4, .maxSpeed = 127});
    exit_condition({32, 10}, 1);
};

void red_elims() {
    knocker.set_value(1);
    chassis.moveToPose(-5, 45, -13, 1100, {.forwards = true, .lead = 0.3, .maxSpeed = 127}, false);
    knocker.set_value(0);
    delay(300);
    chassisForTurns.swingToHeading(-100,
                                   DriveSide::RIGHT,
                                   900,
                                   {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 127},
                                   false);
    lift.spinToPosition(1000);
    delay(500);
    lift.spinToPosition(-30);
    chassisForTurns.turnToHeading(-180, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 100}, false);
    knocker.set_value(1);
    chassisForTurns.turnToHeading(-135, 2000, {.maxSpeed = 100}, false);
    intakeState = IN;
    knocker.set_value(0);
    chassis.moveToPoint(-10, 8, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassisForTurns.swingToHeading(-180, DriveSide::LEFT, 1000, {.maxSpeed = 100}, false);
    chassisForTurns.turnToHeading(90, 2000, {.maxSpeed = 100}, false);

    // chassis.moveToPose(0, 50, -180, 2000, {.forwards = false, .lead = 0.1, .maxSpeed = 127}, false);
    // chassis.swingToHeading(150, DriveSide::RIGHT, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 100}, false);
    // knocker.set_value(1);
};

void blue_elims() {
    knocker.set_value(1);
    chassis.moveToPose(5, 45, 13, 2000, {.forwards = true, .lead = 0.3, .maxSpeed = 127}, false);
    knocker.set_value(0);
    delay(100);
    chassisForTurns.swingToHeading(90,
                                   DriveSide::LEFT,
                                   900,
                                   {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 100},
                                   false);
    chassisForTurns.swingToHeading(105,
                                   DriveSide::RIGHT,
                                   600,
                                   {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 127},
                                   false);
    lift.spinToPosition(900);
};

void autonomous() {
    float start = pros::millis(), end;
    pros::Task intakeControllerStateTask(intakeControllerStateTaskF);

    chassis.setPose({0, 0, 0});

    red_elims();

    end = pros::millis();
    printf("Auton took: %f milliseconds in total\n", end - start);
};

void opcontrol() {
    int knockerState = 0;
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    lift.reset();
    intakeControllerEnabled = false;
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        intake.handleDrivercontrol(controller.get_digital(E_CONTROLLER_DIGITAL_R2) == 1, controller.get_digital(E_CONTROLLER_DIGITAL_R1) == 1);
        lift.handleDrivercontrol(controller.get_digital(E_CONTROLLER_DIGITAL_L2) == 1,
                                 controller.get_digital(E_CONTROLLER_DIGITAL_L1) == 1,
                                 controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP) == 1,
                                 controller.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN) == 1);

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
