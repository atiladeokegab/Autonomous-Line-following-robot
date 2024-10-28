#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
#include <cstdio>
#include <iostream>
#include <fstream> // Include the file stream library

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define SRR_pin PA_0
#define SR_pin  PA_1
#define SC_pin  PA_4
#define SL_pin  PB_0
#define SLL_pin PC_1
#ifndef RADIUS
#define RADIUS 0.08
#endif
#ifndef SET_SPEED
#define SET_SPEED 1.0
#endif
float count_rate_M1_l = 0;
float count_rate_M2_r = 0;

 Timer timer_wheel1;
int previous_pulse_count_M1  = 0;
 Timer timer_wheel2;      
       int previous_pulse_count_M2  = 0;
       float correction = 0;
       float value = 0;

Serial pc(USBTX, NC);


class Sensor {
private:
    AnalogIn inputSignal;

public:
    Sensor(PinName Pin) : inputSignal(Pin) {}

    float value(void) {
        return (inputSignal.read() * 256);
    }
};

class SensorManager {
private:
    Sensor SC;
    Sensor SLL;
    Sensor SL;
    Sensor SR;
    Sensor SRR;

public:
    SensorManager(PinName SC_pin, PinName SLL_pin, PinName SL_pin, PinName SR_pin, PinName SRR_pin)
        : SC(SC_pin), SLL(SLL_pin), SL(SL_pin), SR(SR_pin), SRR(SRR_pin) {}

float calculateWeightedAverage() {
    float x_sum = 0, y_sum = 0, result = 0;

    // Sensor positions and their output values
    float sensor_positions[5] = {-2000, -1000, 0, 1000, 2000};
    float sensor_outputs[5] = {SLL.value(), SL.value(), SC.value(), SR.value(), SRR.value()};
    // Calculate weighted sum
    for (int i = 0; i < 5; ++i) {
        x_sum += sensor_positions[i] * sensor_outputs[i];
        y_sum += sensor_outputs[i];
    }
    return result = x_sum / y_sum;
    return result - 2000;
}
  // Function to calculate error
    void calculateError() {
        float weighted_average = calculateWeightedAverage();
        float errorValue  = 1000 * (weighted_average - 4350);
        
    }
};

class PIDController_Sensors {
private:
    float Kp; // helps to increase or decrease the voltage
    float Ki; // helps to remove steady state error
    float Kd; // helps to prevent overshoot
    float setpoint; // the set point here refers to the desired power we wise to achieve
    float integral; //is expressed as error integral + the accumlation of errror over time
    float prevError;// error previous 
    float dt; //

public:
    PIDController_Sensors(float Kp, float Ki) : Kp(Kp),  Kd(Ki), integral(0), prevError(0) {}

    float compute(float line_position) {
        /*
************************************************************************
The funtions is to calcilate the required voltage to get the dc motot to
the required speed.
************************************************************************
*/      static float last_line_position =0 ;
        float proportional_correction  =Kp*line_position ;
        //float error = setpoint - velocity;
        //float error = desired velocity - actual velocity;
        dt= 0.01;
        //float e integral = e intergral + e * change in time
        float derivative_correction  = Kd* (line_position  - last_line_position) / dt;
        last_line_position = line_position;
        //float output = Kp * error ;
        integral += (line_position-last_line_position)/dt;
        float correction = proportional_correction + derivative_correction; 
        return correction;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};


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
            WD = 0;
             // Not used in this example
            
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
            static int previous_pulse_count_M1 = 0;       
            //timer_wheel2.start();
            int current_pulse_count_M1 = encoder_M1.getPulses();
            //float sample_time = timer_wheel2.read();
            float sample_time_m1 = 0.1;
            int delta_pulse_count_m1 = current_pulse_count_M1 - previous_pulse_count_M1;
            previous_pulse_count_M1 = current_pulse_count_M1;
            //timer_wheel2.reset();
            
        count_rate_M1_l = static_cast<float>(delta_pulse_count_m1) / (sample_time_m1);   
        }
        
    
        void count_rate_M2() {
            static int previous_pulse_count_M2 = 0;
            //timer_wheel2.start();
            int current_pulse_count_M2 = encoder_M2.getPulses();
            //float sample_time = timer_wheel2.read();
            float sample_time = 0.1;
            int delta_pulse_count = current_pulse_count_M2 - previous_pulse_count_M2;
            previous_pulse_count_M2 = current_pulse_count_M2;
            //timer_wheel2.reset();
            
        count_rate_M2_r = static_cast<float>(delta_pulse_count) / (sample_time);     
        }


        // Method to calculate linear velocity
        double wheelVelocity(double count_rate,int pulseperrev) {

            double actual_wheel_diameter =  0.04; // converting millimeters to meters
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
        PIDController(float Kp,float Ki,float Kd) : Kp(Kp), Ki(Ki), Kd(Kd), setpoint(0), integral(0), prevError(0) {}

        void setSetpoint(float setpoint) {
            this->setpoint = setpoint;
        }
    float compute(float velocity) {
        float error = setpoint - velocity;
        dt = 0.1;
        integral += error/dt;
     
        float derivative = (error - prevError) / dt;
        prevError = error;
        float output = ((Kp * error) + (Ki * integral)+ (Kd * derivative));


        return output;
    }


        void reset() {
            integral = 0;
            prevError = 0;
        }
    };


int main() {
    Sensor SC(PA_4);
    Sensor SLL(PC_1);
    Sensor SL(PB_0);
    Sensor SR(PA_1);
    Sensor SRR(PA_0);

    SensorManager sensorManager(SC_pin,SLL_pin,SL_pin,SR_pin,SRR_pin);

    Ticker sensorupdate_M1;
    sensorupdate_M1.attach(callback(&sensorManager, &SensorManager::calculateError), 0.01);

    C12832 lcd(D11, D13, D12, D7, D10);
    Buggy buggy(PA_13, PA_14, PA_15, PB_2, PB_1, PB_15);
    DigitalOut enable(PB_14);
    enable = 1;
    MotorEncoder motorEncoders(PC_5, PC_4, PC_2, PC_3, 1024, QEI::X4_ENCODING);

    Ticker ticker;  
    ticker.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M1), 0.1);
    Ticker ticker2;
    ticker2.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M2), 0.1);

    PIDController pidControllerLeft(0.055, 0.0, 0.0);
    PIDController pidControllerRight(0.055, 0.0, 0.0);
    pidControllerLeft.setSetpoint(1.1);
    pidControllerRight.setSetpoint(1.1);
    PIDController_Sensors PIDController_sensors(0.00063, 0.0);

    while (true) {
        float currentr_lp = sensorManager.calculateWeightedAverage();
        float correction = PIDController_sensors.compute(currentr_lp);
 
        double wheelVelocity_M2 = motorEncoders.wheelVelocity(count_rate_M2_r, 1024);
        double wheelVelocity_M1 = motorEncoders.wheelVelocity(count_rate_M1_l, 1024);
        float pidOutputLeft = pidControllerLeft.compute(wheelVelocity_M1);  
        float pidOutputRight = pidControllerRight.compute(wheelVelocity_M2);
        double pidOutputRight_forward = 0.325 -pidOutputRight; // Explicitly cast pidOutputLeft to double
        double pidOutputLeft_forward = 0.675+pidOutputLeft  ;  // Explicitly cast pidOutputRight to double
        //buggy.line2(, 50,, 50);
        // buggy.line2(pidOutputLeft_forward, 50,  pidOutputRight_forward, 50); // Corrected line
        printf("%.02f, %.02f\r\n", wheelVelocity_M1, wheelVelocity_M2);     
 if (correction > 0) {
        
        buggy.line2(pidOutputLeft_forward - abs(static_cast<float>(correction)), 50,  pidOutputRight_forward, 50); // Corrected line
       
    } else if (correction < 0) {
        buggy.line2(pidOutputLeft_forward, 50,  pidOutputRight_forward    + -(static_cast<float>(correction)), 50);
    }
       
        //printf("%.02f,%.02f\r\n", currentr_lp, correction); // Add the missing format arguments for printf
    }
}

