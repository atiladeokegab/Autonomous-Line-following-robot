#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
#include <iostream>
#include <fstream> // Include the file stream library

#ifndef M_PI
#define M_PI 3.14159265358979323846
#define SRR_pin PA_0
#define SR_pin  PA_1
#define SC_pin  PA_4
#define SL_pin  PB_0
#define SLL_pin PC_1

#endif
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

    // Function to calculate weighted average
    float calculateWeightedAverage() {
        float x_sum = 0, y_sum = 0;

        // Sensor positions and their output values
        float sensor_positions[5] = {0, 1000, 2000, 3000, 4000};
        float sensor_outputs[5] = {SC.value(), SLL.value(), SL.value(), SR.value(), SRR.value()};

        // Calculate weighted sum
        for (int i = 0; i < 5; ++i) {
            x_sum += sensor_positions[i] * sensor_outputs[i];
            y_sum += sensor_outputs[i];
        }

        // Calculate weighted average
        return x_sum / y_sum;
    }

    // Function to calculate error
    void calculateError() {
        float weighted_average = calculateWeightedAverage();
        float errorValue  = 1000 * (weighted_average - 4350);
        ;
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
    PIDController_Sensors(float Kp, float Kd) : Kp(Kp),  Kd(Kd), integral(0), prevError(0) {}



    float compute(float line_position,float last_line_position) {
        /*
************************************************************************
The funtions is to calcilate the required voltage to get the dc motot to
the required speed.
************************************************************************
*/
        float proportional_correction  =Kp*line_position;
        //float error = setpoint - velocity;
        //float error = desired velocity - actual velocity;
        dt= 0.01;

        //float e integral = e intergral + e * change in time

        float derivative_correction  = Kd* (line_position  - last_line_position) / dt;
        last_line_position = line_position;
        //float output = Kp * error ;
        float correction = proportional_correction + derivative_correction;

        
        return correction;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};





int main() {
    SensorManager sensorManager(SRR_pin,SR_pin,SC_pin,SL_pin,SLL_pin);
    PIDController_Sensors PIDController_sensors(0.1, 0.01); // Adjust PID constants as needed
    //
    Ticker sensorupdate_M1;
    sensorupdate_M1.attach(callback(&sensorManager, &SensorManager::calculateError), 0.01); // Attach count_rate_M1

    Sensor SC(PA_4);
    Sensor SLL(PC_1);
    Sensor SL(PB_0);
    Sensor SR(PA_1);
    Sensor SRR(PA_0);
    

    while (true) {

float value = sensorManager.calculateWeightedAverage() ;
        // Scale down the output
value /= 2;

// Shift the scaled output
value -= 1000;

// Multiply the shifted output by 2
value *= 2;
  
      pc.printf("////////////////////////////\r\n");
      pc.printf("Sll is %.2f\r\n",SLL.value());
      pc.printf("Sl is %.2f\r\n", SL.value());       
      pc.printf("SC is %.2f\r\n", SC.value());
      pc.printf("SR is %.2f\r\n", SR.value());
      pc.printf("SRR is %.2f\r\n",SRR.value());
      pc.printf("correction is %.2f\r\n",correction);
      pc.printf("line_postion is %.2f\r\n",value);
      pc.printf("////////////////////////////\r\n");

      wait(1);
    }   
}
