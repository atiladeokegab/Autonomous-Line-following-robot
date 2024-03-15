class Buggy {
private:
    Wheel L, R;

public:
    Buggy(PinName L_BP, PinName L_WD, PinName L_PWM, PinName R_BP, PinName R_WD, PinName R_PWM) 
        : L(L_BP, L_WD, L_PWM), R(R_BP, R_WD, R_PWM) {}

    // Method to move the buggy in a straight line
    void moveStraight(float targetRPM) {
        int pulses_M1 = motorEncoders.getPulses_M1();
        int pulses_M2 = motorEncoders.getPulses_M2();
        double wheelVelocity_M1 = motorEncoders.wheelVelocity(count_rate_M1_l, 1024);
        double wheelVelocity_M2 = motorEncoders.wheelVelocity(count_rate_M2_r, 1024);

        // Calculate the current RPM for both wheels
        double currentRPM_M1 = motorEncoders.getRPM_M1();
        double currentRPM_M2 = motorEncoders.getRPM_M2();

        // Calculate the error in RPM
        double error_RPM = targetRPM - currentRPM_M1;

        // Calculate PID output
        double pidOutput = Kp * error_RPM;

        // Adjust duty cycles of both wheels
        dutyCycleLeft = 0.5 + pidOutput;
        dutyCycleRight = 0.5;

        // Configure wheels
        L.configure(dutyCycleLeft, 50);
        R.configure(dutyCycleRight, 50);
    }
};

int main() {
    // Setup code...

    Buggy buggy(PA_13, PA_14, PA_15, PB_2, PB_1, PB_15);

    while (true) {
        // Move the buggy in a straight line at 100 RPM for the left wheel
        buggy.moveStraight(100);

        // Adjust the delay based on your requirements
        wait(0.01);
    }
}
