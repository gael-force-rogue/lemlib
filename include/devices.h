#include "main.h"

enum LiftState {
    STANDBY = 0,
    LOAD = 65
};

class Lift {
   private:
    Motor motor;
    bool macroIsRunning = false;

   public:
    Lift(int port) : motor(port, MotorGearset::red) {};

    inline void init() {
        motor.tare_position();
        motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    };

    inline void up() {
        motor.move(127);
    };

    inline void down() {
        motor.move(-127);
    };

    inline void stop() {
        motor.move(0);
    };

    inline void handleDrivercontrol(bool inB, bool outB) {
        if (inB) {
            macroIsRunning = false;
            motor.move(127);
        } else if (outB) {
            macroIsRunning = false;
            motor.move(-127);
        } else if (!macroIsRunning) {
            motor.move(0);
        };
    };

    inline void handleMacro() {
        macroIsRunning = true;

        printf("Lift position: %f\n", motor.get_position());

        // If not in the usual states, return to load
        if (fabs(motor.get_position() - LOAD) < 2) {
            motor.move_absolute(STANDBY, 127);
        } else {
            motor.move_absolute(LOAD, 127);
        };

        macroIsRunning = false;
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
            pros::delay(50);
            motor.move(127);

            antiJamIsRunning = false;
        };
    };
};