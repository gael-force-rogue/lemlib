#pragma once

#include "main.h"

enum LiftState {
    STANDBY = 0,
    LOAD = 180,
    SCORE = 600,
    ALLIANCE = 800,
};

class LiftWithPID {
   private:
    Motor motor;
    bool driverInterrupt = false;

   public:
    LiftState state = STANDBY;

    LiftWithPID(int port) : motor(port, MotorGearset::red) {
        this->motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        this->motor.set_zero_position(0);
    };

    void reset() {
        motor.tare_position();
    }

    void coast() {
        motor.set_brake_mode(E_MOTOR_BRAKE_COAST);
        motor.brake();
    };
    void hold() {
        motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        motor.brake();
    };

    inline float position() {
        return motor.get_position();
    };

    inline void handleDrivercontrol(bool upB, bool downB, bool macroB) {
        if (upB) {
            this->driverInterrupt = true;
            this->motor.move(127);
        } else if (downB) {
            this->driverInterrupt = true;
            this->motor.move(-127);
        } else if (macroB) {
            if (driverInterrupt) {
                state = LOAD;
            } else if (state == LOAD) {
                state = STANDBY;
            } else if (state == STANDBY) {
                state = LOAD;
            };

            motor.move_absolute(state, 50);

            printf("Position: %f Target: %f\n", this->position(), this->state);

            driverInterrupt = false;
        } else if (driverInterrupt) {
            motor.brake();
        };
    };

    inline void spinToState(LiftState state) {
        if (state == STANDBY) {
            motor.move_absolute(state, 400);
        } else if (state == SCORE) {
            motor.move_absolute(state, 600);
        } else
            motor.move_absolute(state, 100);
    };

    inline void spinToPosition(float position, float velocity) {
        motor.move_absolute(position, velocity);
    };
};

class Intake {
   private:
    Motor motor;
    bool antiJamIsRunning = false;

   public:
    Intake(int port) : motor(port, pros::MotorGearset::blue) {};

    inline void in() {
        motor.move(127);
    };

    inline void out() {
        motor.move(-127);
    };

    inline void stop() {
        motor.move(0);
    };

    inline void handleDrivercontrol(bool inB, bool outB) {
        if (inB) {
            motor.move(127);
        } else if (outB) {
            motor.move(-127);
        } else {
            motor.move(0);
        };
    };

    inline void handleAntiJam() {
        if (motor.get_actual_velocity() == 0 && motor.get_target_velocity() != 0) {
            antiJamIsRunning = true;

            motor.move(-127);
            pros::delay(100);
            motor.move(127);

            antiJamIsRunning = false;
        };
    };
};