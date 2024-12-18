#include "mbed.h"
#include "QEI.h"
#include "C12832.h"


class Wheel {

private:

    DigitalOut BP, WD;
    PwmOut P;

public:

    // Constructor setting the bipolar, direction and pwm pins for the wheel motor.
    Wheel(PinName bipolar, PinName direction, PinName PWM) : BP(bipolar), WD(direction), P(PWM) {

        initialise();

    };

    void initialise() {

        // Setting the wheel motor to operate in bipolar mode.
        BP = 1;
        //wheel direction unused
        // WD = 1;

    }

    // Setting the duty cycle and the period of the wheel motor.
    void configure(float dutyCycle, int period_us) {

        P.write(dutyCycle);
        P.period_us(period_us);

    }
    
    // NOT IN USE
    // Calulating and setting the duty cyle by using the desired average voltage and the max voltage. Negative = opposite direction. Need to specify the period.
    void setAvgVoltage(float avgVoltage, float maxVoltage, int period_us) {

        float DC = 0.5 * ( ( avgVoltage / maxVoltage ) + 1 );

        configure(DC, period_us);

    }

    // NOT IN USE
    // Set the velocity using a percentage of the max speed. Negative = opposite direction. Need to specify the period.
    void setVelPerc(float percentage, int period_us) {

        setAvgVoltage(percentage, 1.0f, period_us);

    }

};

int PPR = 600;
int LPC = 2000;//2387;//100;//4775; [LPC = Line Pulse Count]
int TPC = 300;//347;//100;//694; [TPC = Turn Pulse Count]
//800 p -> 270 degrees
// 190 p - > 10 degrees
// 300 p -> almost 90 degrees

// LEFT ENCODER
QEI Enc(PC_2, PC_3, NC, PPR);

class Buggy {

    private:

        Wheel L, R;

    public:

    // Constructor setting the bipolar, direction and pwm pins for the left and right wheels.
    Buggy(PinName L_BP, PinName L_WD, PinName L_PWM, PinName R_BP, PinName R_WD, PinName R_PWM) : L(L_BP, L_WD, L_PWM), R(R_BP, R_WD, R_PWM) {}

    void turn(float DC, int t) {

        Enc.reset();

        L.configure(DC, t);
        R.configure(DC, t);

    }

    void line2(float DC_L, int period_L, float DC_R, int period_R) {

        L.configure(DC_L, period_L);
        R.configure(DC_R, period_R);

        wait(1.0f);
 

    }
   
   void halt(float DC_L_h, int period_L_h, float DC_R_h, int period_R_h) {

        L.configure(DC_L_h, period_L_h);
        R.configure(DC_R_h, period_R_h);

        wait(1.0f);

    }

};


//BLUETOOTH HAS NOT BEEN TESTED YET

// 0.185 m = distance between wheels 
// 0.04 m  = wheel radius

/*
___________________  assuming the buggy starts from here 
|                 |
|                 |
|                 |
|                 |
|                 |
|                 |
|                 |
___________________
Length of tape is 1m
angle is 90 degrees
*/

class Potentiometer // Begin Potentiometer class definition
{
private:                                              // Private data member declaration
    AnalogIn inputSignal;                             // Declaration of AnalogIn object
    float VDD, currentSampleNorm, currentSampleVolts; // Float variables to speficy the value of VDD and most recent samples

public:                                                               // Public declarations
    Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {} // Constructor - user provided pin name assigned to AnalogIn...
                                                                      // VDD is also provided to determine maximum measurable voltage
    float amplitudeVolts(void)                                        // Public member function to measure the amplitude in volts
    {
        return (inputSignal.read() * VDD); // Scales the 0.0-1.0 value by VDD to read the input in volts
    }

    float amplitudeNorm(void) // Public member function to measure the normalised amplitude
    {
        return inputSignal.read(); // Returns the ADC value normalised to range 0.0 - 1.0
    }

    void sample(void) // Public member function to sample an analogue voltage
    {
        currentSampleNorm = inputSignal.read();       // Stores the current ADC value to the class's data member for normalised values (0.0 - 1.0)
        currentSampleVolts = currentSampleNorm * VDD; // Converts the normalised value to the equivalent voltage (0.0 - 3.3 V) and stores this information
    }

    float getCurrentSampleVolts(void) // Public member function to return the most recent sample from the potentiometer (in volts)
    {
        return currentSampleVolts; // Return the contents of the data member currentSampleVolts
    }

    float getCurrentSampleNorm(void) // Public member function to return the most recent sample from the potentiometer (normalised)
    {
        return currentSampleNorm; // Return the contents of the data member currentSampleNorm
    }
};

class SamplingPotentiometer : public Potentiometer
{
private:
    float samplingFrequency, samplingPeriod;
    Ticker sampler;

public:
    SamplingPotentiometer(PinName p, float v, float fs) : Potentiometer(p, v), samplingFrequency(fs)
    {
        samplingPeriod = 1.0 / samplingFrequency;
        sampler.attach(callback(this, &SamplingPotentiometer::sample), samplingPeriod);
    }
};


int main() {
    SamplingPotentiometer leftPot(A0, 3.3, 100);
    SamplingPotentiometer rightPot(A1, 3.3, 100);
    C12832 lcd(D11, D13, D12, D7, D10);
    // Enable pin of the motor drive board
    DigitalOut enable(PB_12);
    enable = 1;

    // Creating an object of type "Buggy" while specifying the pins for the wheel motors.
    Buggy B(PA_13, NC, PA_15, PB_2, NC, PB_15);

    Enc.reset();

        // make the buggy move in a straight line at a slower speed.
while(true){
        lcd.cls();
        lcd.locate(0, 0);  
        float LPNorm = float(leftPot.getCurrentSampleNorm() );
        float RPNorm= float(rightPot.getCurrentSampleNorm() );
        lcd.printf("%02f:::%02f", LPNorm, RPNorm); // Print the time to the LCD screen
        B.line2( LPNorm, 10, RPNorm, 10);

}

}
