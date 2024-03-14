#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
#include <iostream>
#include <fstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#define SRR_pin PA_0
#define SR_pin  PA_1
#define SC_pin  PA_4
#define SL_pin  PB_0
#define SLL_pin PC_1
#endif

Timer timer_wheel1;
int previous_pulse_count_M1 = 0;
Timer timer_wheel2;
int previous_pulse_count_M2 = 0;
float count_rate_M1_l = 0;
float count_rate_M2_r = 0;
float dutyCycleLeft = 0;
float dutyCycleRight = 0;
float errorValue = 0;
float speed_right = 3;
float speed_left = 3;
float correction = 0;
float Kp = 0.4; // Proportional gain
float Kd = 12; // Derivative gain
float line_position = 0;
float last_line_position = 0;

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

class Sensor {
private:
    AnalogIn inputSignal;

public:
    Sensor(PinName Pin) : inputSignal(Pin) {}

    float value(void) {
        return (inputSignal.read() * 1000);
    }
};

class SensorManager {
private:
    Sensor SC;
    Sensor SLL;
    Sensor SL;
    Sensor SR;
    Sensor SRR;
    int CNYmin[5];
    int CNYmax[5];

    void updateMinMax(Sensor& sensor, int index) {
        int sensorValue = sensor.value();
        CNYmin[index] = (sensorValue < CNYmin[index]) ? sensorValue : CNYmin[index];
        CNYmax[index] = (sensorValue > CNYmax[index]) ? sensorValue : CNYmax[index];
    }

public:
    SensorManager(PinName SC_pin, PinName SLL_pin, PinName SL_pin, PinName SR_pin, PinName SRR_pin)
        : SC(SC_pin), SLL(SLL_pin), SL(SL_pin), SR(SR_pin), SRR(SRR_pin) {}

     void calculateWeightedAverage() {
        float x_sum = 0, y_sum = 0;
        float sensor_positions[5] = {0, 1000, 2000, 3000, 4000};
        float sensor_outputs[5] = {SC.value(), SLL.value(), SL.value(), SR.value(), SRR.value()};

        for (int i = 0; i < 5; ++i) {
            x_sum += sensor_positions[i] * sensor_outputs[i];
            y_sum += sensor_outputs[i];
        }

        line_position = x_sum / y_sum;
    }

    void calculateError() {
        float weighted_average = line_position;
        float errorValue = 1000 * (weighted_average - 4350);
    }

    void sensor_calibration() {
        for (int i = 0; i < 5; ++i) {
            CNYmin[i] = INT_MAX;
            CNYmax[i] = INT_MIN;
        }

        updateMinMax(SC, 0);
        updateMinMax(SLL, 1);

        updateMinMax(SL, 2);
        updateMinMax(SR, 3);
        updateMinMax(SRR, 4);
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

    // Method to get the current time elapsed in milliseconds
    uint64_t getTimeElapsed() {
        return timer.read_ms();
    }

    // Method to reset the encoders
    void reset() {
        encoder_M1.reset();
        encoder_M2.reset();
    }

    // Method to get pulse count for motor 1
   /* uint64_t timeBetweenRisingEdges() {
        // Wait for the first rising edge
        while (encoder_M1.getB() != 1);
        
        // Record timestamp of the first rising edge
        uint64_t first_edge_time = timer.read_us();
        
        // Wait for the second rising edge
        while (encoder_M1.getB() != 1);
        
        // Record timestamp of the second rising edge
        uint64_t second_edge_time = timer.read_us();
        
        // Calculate the time difference
        return second_edge_time - first_edge_time;
        //  second per count
    }*/

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
                //encoder rate
        // there are two methods
        //1) count pulses per time interval
        //2) measure the time elapsed between triggers     
        timer_wheel1.start();
        int current_pulse_count_M1 = encoder_M1.getPulses();
        float sample_time = timer_wheel1.read();
        int delta_pulse_count = current_pulse_count_M1 - previous_pulse_count_M1;
        previous_pulse_count_M1 = current_pulse_count_M1; 
        timer_wheel1.reset();
    
      count_rate_M1_l = static_cast<float>(delta_pulse_count) / (sample_time);   
    }
    
   
    void count_rate_M2() {
        //encoder rate
        // there are two methods
        //1) count pulses per time interval
        //2) measure the time elapsed between triggers

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
        //actual wheel diameter in meters
        // 0.185 m = distance between wheels 
        // 0.04 m  = wheel radius
        //  pratical gear ratio gear box two gear ratio = 15 
        //actual is 10.8
        //12cm wheel base= 0.12m
        //Linear Velocity = Encoder Tick Rate * (Ï€ * D / 100) * G
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
    float dt; // ???????

public:
    PIDController(float Kp, float Ki, float Kd, float dt) : Kp(Kp), Ki(Ki), Kd(Kd), setpoint(0), integral(0), prevError(0), dt(dt) {}

    void setSetpoint(float setpoint) {
        this->setpoint = setpoint;
    }

    float compute(float velocity) {
        /*
************************************************************************
The funtions is to calcilate the required voltage to get the dc motot to
the required speed.
************************************************************************
*/

        float error = setpoint - velocity;
        //float error = desired velocity - actual velocity;

        integral += error * dt;
        //float e integral = e intergral + e * change in time

        float derivative = (error - prevError) / dt;
        prevError = error;

        float output = Kp * error + Ki * integral + Kd * derivative;

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
    SensorManager sensorManager(SRR_pin, SR_pin, SC_pin, SL_pin, SLL_pin);
    MotorEncoder motorEncoders(PC_5, PC_4, PC_2, PC_3, 1024, QEI::X4_ENCODING);
    double wheelVelocity_M1 = motorEncoders.wheelVelocity(count_rate_M1_l, 1024);
    double wheelVelocity_M2 = motorEncoders.wheelVelocity(count_rate_M2_r, 1024);
    double translational_velocity = motorEncoders.translational_velocity(wheelVelocity_M1, wheelVelocity_M2);
    double angularVelocity = motorEncoders.angularVelocity(wheelVelocity_M1,wheelVelocity_M2);
    PIDController speed_left(0.1,0.001,12,0.001);
    PIDController speed_right(0.1,0.001,12,0.001);
    speed_right.setSetpoint(1.5);
    speed_right.setSetpoint(1.5);

    
    
    Ticker sensorupdate_M1;
    sensorupdate_M1.attach(callback(&sensorManager, &SensorManager::calculateError), 0.01);
    Ticker ticker;    
    ticker.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M1), 0.1); // Attach count_rate_M1
    Ticker ticker2;
    ticker2.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M2), 0.1); // Attach count_rate_M2

    Sensor SC(PA_4);
    Sensor SLL(PC_1);
    Sensor SL(PB_0);
    Sensor SR(PA_1);
    Sensor SRR(PA_0);

while (true) {
    sensorManager.calculateWeightedAverage(); // Update line_position

    float proportional_correction = Kp * line_position;
    float dt = 0.01;
    float derivative_correction = Kd *( (line_position - last_line_position) / dt);
    last_line_position = line_position;
    float correction = proportional_correction + derivative_correction;

    if (correction > 2000) {
        buggy.line2(speed_left.compute(wheelVelocity_M1), 50, speed_right.compute(wheelVelocity_M2)- correction, 50);
    } else {
       buggy.line2(speed_left.compute(wheelVelocity_M1) - correction, 50, speed_right.compute(wheelVelocity_M2), 50);
    }
    
  
    pc.printf("////////////////////////////\r\n");
    pc.printf("Sll is %.2f\r\n", SLL.value());
    pc.printf("Sl is %.2f\r\n", SL.value());
    pc.printf("SC is %.2f\r\n", SC.value());
    pc.printf("SR is %.2f\r\n", SR.value());
    pc.printf("SRR is %.2f\r\n", SRR.value());
    
    pc.printf("wheelVelocity_M1 is %.2f\r\n", wheelVelocity_M1);
    pc.printf("wheelVelocity_M2 is %.2f\r\n", wheelVelocity_M2);

     pc.printf("speed_left.compute(wheelVelocity_M1) is %.2f\r\n", speed_left.compute(wheelVelocity_M1));
    pc.printf("speed_left.compute(wheelVelocity_M2) is %.2f\r\n", speed_left.compute(wheelVelocity_M2));
           if (correction > 2000) {
        pc.printf("speed_left.compute(wheelVelocity_M1) is %.2f\r\n", speed_left.compute(wheelVelocity_M1));
        pc.printf("speed_right.compute(wheelVelocity_M2)- correction is %.2f\r\n", speed_right.compute(wheelVelocity_M2)- correction);
    } else {
       buggy.line2(speed_left.compute(wheelVelocity_M1) - correction, 50, speed_right.compute(wheelVelocity_M2), 50);
       pc.printf("speed_left.compute(wheelVelocity_M1) - correction is %.2f\r\n", speed_left.compute(wheelVelocity_M1) - correction);
       pc.printf("speed_right.compute(wheelVelocity_M2) - correction is %.2f\r\n", speed_right.compute(wheelVelocity_M2) - correction);
    }
    pc.printf("line_position is %f\r\n", line_position);
    pc.printf("proportional_correction is %f\r\n", proportional_correction);
    pc.printf("derivative_correction is %f\r\n", derivative_correction);
    pc.printf("correction is %f\r\n", correction);
    pc.printf("////////////////////////////\r\n");

    wait(1);
}
}
