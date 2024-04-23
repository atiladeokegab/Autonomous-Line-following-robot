#include "mbed.h"

#define Enable PB_14

#define Left_Bipolar PA_13
#define Left_Wheel_Direction NC
#define Left_PWM PA_15
#define SensorL PB_0

#define Right_Bipolar PB_2
#define Right_Wheel_Direction NC
#define Right_PWM PB_15
#define SensorR PA_1

class Side {

private:

    AnalogIn S; PwmOut P;

    float threshold, F_DC, B_DC;

public:

    Side(PinName S_In, PinName P_Out, int period_us, float setThreshold, float setF_DC, float setB_DC) : S(S_In), P(P_Out), threshold(setThreshold), F_DC(setF_DC), B_DC(setB_DC) {

        P.period_us(period_us);

    }
     
    void update() {

        (S >= threshold) ? P.write(F_DC) : P.write(B_DC);

    }

};

int main() {

    DigitalOut enable(Enable), LBP(Left_Bipolar), LWD(Left_Wheel_Direction), RBP(Right_Bipolar), RWD(Right_Wheel_Direction);
    enable = 1, LBP = 1, LWD = 0, RBP = 1, RWD = 0;

    Side Left(SensorR, Left_PWM, 10, 2.0f, 0.75f, 0.50f), Right(SensorL, Right_PWM, 10, 2.0f, 0.80f, 0.50f);

    while (true) {

        Left.update(); Right.update();

    }

}