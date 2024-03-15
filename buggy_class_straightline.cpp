#include <cmath> // Include the math library for PI constant

// Class definitions and other code...

class Buggy {
private:
    Wheel L, R;
    PIDController pidController; // Add PID controller instance

public:
    Buggy(PinName L_BP, PinName L_WD, PinName L_PWM, PinName R_BP, PinName R_WD, PinName R_PWM) 
        : L(L_BP, L_WD, L_PWM), R(R_BP, R_WD, R_PWM), pidController(Kp, Ki, Kd) // Initialize PID controller with appropriate constants
    {
        pidController.setSetpoint(0); // Set desired angular velocity to 0 for straight line
    }

    void moveStraight() {
        int getPulses_M1 = motorEncoders.getPulses_M1();
        int getPulses_M2 = motorEncoders.getPulses_M2();
        double wheelVelocity_M1 = motorEncoders.wheelVelocity(count_rate_M1_l, 1024);
        double wheelVelocity_M2 = motorEncoders.wheelVelocity(count_rate_M2_r, 1024);

        // Calculate angular velocity
        double angularVelocity = motorEncoders.angularVelocity(wheelVelocity_M1, wheelVelocity_M2);

        // Compute PID output based on the angular velocity
        double pidOutput = pidController.compute(angularVelocity);

        // Adjust duty cycles of both wheels using PID output
        dutyCycleLeft = 0.5 + pidOutput;
        dutyCycleRight = 0.5 - pidOutput; // Reverse direction for the right wheel to move straight

        // Configure wheels
        L.configure(dutyCycleLeft, 50);
        R.configure(dutyCycleRight, 50);
    }
};

int main() {
    // Other setup code...

    Buggy buggy(PA_13, PA_14, PA_15, PB_2, PB_1, PB_15);

    while (true) {
        buggy.moveStraight(); // Call moveStraight() to make the buggy move in a straight line
        wait(0.01); // Adjust this delay according to your requirements
    }
}
