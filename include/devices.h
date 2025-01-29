#pragma once

#include "main.h"

enum LiftState {
    STANDBY = -100,
    LOAD = 35,
    SCORE = 600,
    ALLIANCE = 800,
};

class LiftWithPID {
   private:
    Motor motor;
    Rotation rotation;
    bool driverInterrupt = false;

   public:
    LiftState state = STANDBY;

    LiftWithPID(int port, int rPort) : motor(port, MotorGearset::red), rotation(rPort) {};
    void reset() {
        this->motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        motor.tare_position();
        rotation.reset_position();
    };

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

    inline void handleDrivercontrol(bool upB, bool downB, bool macroB, bool scoreMacroB = false) {
        if (upB) {
            this->driverInterrupt = true;
            this->motor.move(127);
        } else if (downB) {
            this->driverInterrupt = true;
            this->motor.move(-127);
        } else if (macroB) {
            if (scoreMacroB) {
                this->state = SCORE;
            } else if (driverInterrupt) {
                state = LOAD;
            } else if (state == LOAD) {
                state = STANDBY;
            } else if (state == STANDBY) {
                state = LOAD;
            };

            motor.move_absolute(state, 100);

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

    inline void spinToPosition(float position, float velocity = 100) {
        motor.move_absolute(position, velocity);
    };
};

class Intake {
   private:
    Motor motor;
    bool antiJamIsRunning = false;

   public:
    Intake(int port) : motor(port, pros::MotorGearset::blue) {};

    inline void in(float velocity = 127) {
        motor.move(velocity);
    };

    inline void out(float velocity = 127) {
        motor.move(-velocity);
    };

    inline void stop() {
        motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        motor.move(0);
        motor.brake();
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
            delay(100);
            motor.move(127);

            antiJamIsRunning = false;
        };
    };
};