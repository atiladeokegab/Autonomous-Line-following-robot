#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
#include <iostream>
#include <fstream> // Include the file stream library
#ifndef M_PI
#define M_PI 3.14159265358979323846
#ifndef RADIUS
#define RADIUS 0.08
#endif
Timer timer_wheel1;
int previous_pulse_count_M1  = 0;
Timer timer_wheel2;      
int previous_pulse_count_M2  = 0;
float count_rate_M1_l = 0;
float count_rate_M2_r = 0;
float dutyCycleLeft = 0.0;
float dutyCycleRight = 0.0;
float pidOutputLeft = 0.0; // Base duty cycle + PID output
float  pidOutputRight = 0.0; // Base duty cycle + PID output
Serial pc(USBTX, NC);

class Wheel {
private:
    DigitalOut BP, WD;
    PwmOut P;

public:
    Wheel(PinName bipolar, PinName direction, PinName PWM) : BP(bipolar), WD(direction), P(PWM) {
        initialise();
    };

    void initialise() {
        BP = 1;
        // WD = 1; // Not used in this example
    }

    void configure(float dutyCycle, float period_ms) {
        P.write(dutyCycle);
        P.period_us(period_ms);
    }


};


class Buggy {

private:
    Wheel L, R;

public:

    Buggy(PinName L_BP, PinName L_WD, PinName L_PWM, PinName R_BP, PinName R_WD, PinName R_PWM) : L(L_BP, L_WD, L_PWM), R(R_BP, R_WD, R_PWM) {}

    void turn(float DC, int t) {


        L.configure(DC, t);
        R.configure(DC, t);

    }

    void line2(float DC_L, int period_L, float DC_R, int period_R) {

        L.configure(DC_L, period_L);
        R.configure(DC_R, period_R);



    }

    void halt() {

        L.configure(0.5, 1);
        R.configure(0.5, 1);

        wait(1.0f);

    }

};

class Potentiometer 
{
private:                                              
    AnalogIn inputSignal;                             
    float VDD, currentSampleNorm, currentSampleVolts; 

public:                                                               
    Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {} 

    float amplitudeVolts(void) {                                      
        return (inputSignal.read() * VDD); 
    }

    float amplitudeNorm(void) {                                       
        return inputSignal.read(); 
    }

    void sample(void) { 
        currentSampleNorm = inputSignal.read();       
        currentSampleVolts = currentSampleNorm * VDD; 
    }

    float getCurrentSampleVolts(void) { 
        return currentSampleVolts; 
    }

    float getCurrentSampleNorm(void) { 
        return currentSampleNorm; 
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
        double samplingPeriod = 1.0 / samplingFrequency;
        sampler.attach(callback(this, &SamplingPotentiometer::sample), samplingPeriod);
    }
};

class MotorEncoder {
public:
    MotorEncoder(PinName pinA_M1, PinName pinB_M1, PinName pinA_M2, PinName pinB_M2, int pulsesPerRev, QEI::Encoding encoding = QEI::X4_ENCODING)
        : encoder_M1(pinA_M1, pinB_M1, NC, pulsesPerRev, encoding), 
          encoder_M2(pinA_M2, pinB_M2, NC, pulsesPerRev, encoding) {
        timer.start();
    }
    // Destructor
    ~MotorEncoder() {
        timer.stop();
    }


    uint64_t getTimeElapsed() {
        return timer.read_ms();
    }

    // Method to reset the encoders
    void reset() {
        encoder_M1.reset();
        encoder_M2.reset();
    }


    int getPulses_M1() {
        return abs(encoder_M1.getPulses());
    }
 
    // Method to get pulse count for motor 2
    int getPulses_M2() {
        return abs(encoder_M2.getPulses());
    }

    // Method to get revolutions for motor 1
    int getRevolutions_M1() {
        // can not be obtained because the index channel is not connected
        return encoder_M1.getRevolutions();
    }

    // Method to get revolutions for motor 2
    int getRevolutions_M2() {
        // can not be obtained because the index channel is not connected
        return encoder_M2.getRevolutions();
    }

    // Method to calculate wheel velocity
   
    void count_rate_M1() {
           
        timer_wheel1.start();
        int current_pulse_count_M1 = encoder_M1.getPulses();
        float sample_time = timer_wheel1.read();
        int delta_pulse_count = current_pulse_count_M1 - previous_pulse_count_M1;
        previous_pulse_count_M1 = current_pulse_count_M1; 
        timer_wheel1.reset();
    
      count_rate_M1_l = static_cast<float>(delta_pulse_count) / (sample_time);   
    }
    
   
    void count_rate_M2() {
        timer_wheel2.start();
        int current_pulse_count_M2 = encoder_M2.getPulses();
        float sample_time = timer_wheel2.read();
        int delta_pulse_count = current_pulse_count_M2 - previous_pulse_count_M2;
        previous_pulse_count_M2 = current_pulse_count_M2;
        timer_wheel2.reset();
        count_rate_M2_r = static_cast<float>(delta_pulse_count) / (sample_time);     
    }


    // Method to calculate linear velocity
    double wheelVelocity(double count_rate,int pulseperrev) {

        double actual_wheel_diameter =  0.04; // converting millimeters to meters
        double gear_ratio = 10.8;
        return (count_rate * M_PI * actual_wheel_diameter )/pulseperrev * 4;
    }


    // Method to calculate angular velocity of the wheel
    double angularVelocity(double left_wheel_velocity, double right_wheel_velocity) {
        //wheelbase
        //12cm wheel base= 0.12m
        double wheelbase = 0.12 ;
        return (right_wheel_velocity - left_wheel_velocity) / wheelbase;
    }

    // Method to calculate translational velocity of the robot
    double translational_velocity(double left_wheel_velocity, double right_wheel_velocity){
           return (right_wheel_velocity + left_wheel_velocity) / 2;
    }

private:
    QEI encoder_M1;
    QEI encoder_M2;
    Timer timer;
};


class PIDController {
private:
    float Kp; // helps to increase or decrease the voltage
    float Ki; // helps to remove steady state error
    float Kd; // helps to prevent overshoot
    float setpoint; // the set point here refers to the desired power we wise to achieve
    float integral; //is expressed as error integral + the accumlation of errror over time
    float prevError;// error previous 
    float dt; //

public:
    PIDController(float Kp) : Kp(Kp), Ki(Ki), Kd(Kd), setpoint(0), integral(0), prevError(0) {}

    void setSetpoint(float setpoint) {
        this->setpoint = setpoint;
    }
float compute(float velocity) {
    float error = setpoint - velocity;
    integral += error;
    dt = 0.01;
    float derivative = (error - prevError) / dt;
    prevError = error;
    float output = (Kp * error + Ki * integral+ Kd * derivative)/1000;


    return output;
}


    void reset() {
        integral = 0;
        prevError = 0;
    }
};

int main() {
    C12832 lcd(D11, D13, D12, D7, D10);
    Buggy buggy(PA_13, PA_14, PA_15, PB_2, PB_1, PB_15);
    DigitalOut enable(PA_8);
    enable = 1;
    MotorEncoder motorEncoders(PC_5, PC_4, PC_2, PC_3, 1024, QEI::X4_ENCODING);
    // Initialize PID controllers for each wheel
    
    PIDController pidControllerLeft(1); // Adjust PID constants as needed
    PIDController pidControllerRight(1); // Adjust PID constants as needed
    pidControllerLeft.setSetpoint(1.5);
    pidControllerRight.setSetpoint(1.5);

Ticker ticker;    
ticker.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M1), 0.01); // Attach count_rate_M1
Ticker ticker2;
ticker2.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M2), 0.01); // Attach count_rate_M2



lcd.cls();


    while (true) {
                // Measure current velocities of both wheels
        int getPulses_M1 =   motorEncoders.getPulses_M1();
        int getPulses_M2 =   motorEncoders.getPulses_M2();
        double wheelVelocity_M1 = motorEncoders.wheelVelocity(count_rate_M1_l, 1024);
        double wheelVelocity_M2 = motorEncoders.wheelVelocity(count_rate_M2_r, 1024);
        double translational_velocity = motorEncoders.translational_velocity(wheelVelocity_M1, wheelVelocity_M2);
        //    double angularVelocity(double left_wheel_velocity, double right_wheel_velocity) {
        double angularVelocity = motorEncoders.angularVelocity(wheelVelocity_M1,wheelVelocity_M2);
        // set for the desired speed 

        float pidOutputLeft = pidControllerLeft.compute(wheelVelocity_M1);
        float pidOutputRight = pidControllerRight.compute(wheelVelocity_M2);
        float dutyCycleLeft =  0.5 +pidOutputLeft; // Base duty cycle + PID output
        float dutyCycleRight = 0.5 - pidOutputRight; // Base duty cycle - PID output
      
   
         pc.printf("left motor info\r\n");
         pc.printf("////////////////////////////\r\n");
         pc.printf("speed_left is %.2f\r\n",wheelVelocity_M1);
         pc.printf("getPulses_M1 is %i\r\n",getPulses_M1);
         pc.printf("dutyCycleLeft is %.2f\r\n",dutyCycleLeft);
         pc.printf("pidOutputLeft is %.2f\r\n",pidOutputLeft);
         pc.printf("////////////////////////////\r\n");
        buggy.line2(dutyCycleLeft,50,dutyCycleRight,50);

        pc.printf("Right motor info\r\n");
        pc.printf("////////////////////////////\r\n");
        pc.printf("speed_right is %.2f\r\n",wheelVelocity_M2);
        pc.printf("getPulses_M2 is %i\r\n",getPulses_M2);
        pc.printf("dutyCycleRight is %.5f\r\n",dutyCycleRight);
        pc.printf("pidOutputRight is %.5f\r\n",pidOutputRight);
        pc.printf("////////////////////////////\r\n");
         
      wait(1);
    }         
}
