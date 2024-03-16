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
       bool line_detected = 1;
       float correction = 0;
       float value = 0;

Serial pc(USBTX, NC);

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

    float min_values[5]; // Array to store minimum values for each sensor
    float max_values[5]; // Array to store maximum values for each sensor

public:
    SensorManager(PinName SC_pin, PinName SLL_pin, PinName SL_pin, PinName SR_pin, PinName SRR_pin)
        : SC(SC_pin), SLL(SLL_pin), SL(SL_pin), SR(SR_pin), SRR(SRR_pin) {
        // Initialize min and max values for each sensor
        min_values[0] = 0;  // Minimum value for SC
        min_values[1] = 0;  // Minimum value for SLL
        min_values[2] = 0;  // Minimum value for SL
        min_values[3] = 0;  // Minimum value for SR
        min_values[4] = 0;  // Minimum value for SRR

        max_values[0] = 256;  // Maximum value for SC
        max_values[1] = 256;  // Maximum value for SLL
        max_values[2] = 256;  // Maximum value for SL
        max_values[3] = 256;  // Maximum value for SR
        max_values[4] = 256;  // Maximum value for SRR
    }

    // Function to calculate weighted average
    float calculateWeightedAverage() {
        float x_sum = 0, y_sum = 0;

        // Sensor positions and their output values
        float sensor_positions[5] = {0, 1000, 2000, 3000, 4000};
        float sensor_outputs[5] = {
            normalize(SC.value(), min_values[0], max_values[0]),
            normalize(SLL.value(), min_values[1], max_values[1]),
            normalize(SL.value(), min_values[2], max_values[2]),
            normalize(SR.value(), min_values[3], max_values[3]),
            normalize(SRR.value(), min_values[4], max_values[4])
        };

        // Calculate weighted sum
        for (int i = 0; i < 5; ++i) {
            x_sum += sensor_positions[i] * sensor_outputs[i];
            y_sum += sensor_outputs[i];
            if(CNYesc[i] > 200){       //detecta si hay una linea
             line_detected = 1;
            }     

        }
        // Calculate weighted average
        return x_sum / y_sum;
    }

    
    // Function to normalize sensor output
    float normalize(float value, float min_value, float max_value) {
        // Perform normalization
        return (value - min_value) / (max_value - min_value);
    }

    // Function to calculate error
    void calculateError() {
        float weighted_average = calculateWeightedAverage();
        float errorValue  = 1000 * (weighted_average - 4350);
        // Handle error calculation as needed
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

    while (true) {
     float line_postion = sensorManager.calculateWeightedAverage() ;
              if(line_detected == 1) {
            // If there is a line_detected
            line_postion = line_postion - (5 - 1) / 2 * 1000; // Convert to error range
            last_dir = (line_postion >= 0) ? 1 : 0; // Update the last direction
        } else {
            // If no line is detected
            line_postion = 0 + (5 - 1) * 1000 * last_dir; // Use last direction information
            line_postion = line_postion - (5 - 1) / 2 * 1000; // Convert to error range
        }
        float correction = PIDController_sensors.compute(line_position, last_line_position);
        last_line_position = line_position; // Update last_line_position

        
      pc.printf("////////////////////////////\r\n");
      pc.printf("Sll is %.2f\r\n",SLL.value());
      pc.printf("Sl is %.2f\r\n", SL.value());       
      pc.printf("SC is %.2f\r\n", SC.value());
      pc.printf("SR is %.2f\r\n", SR.value());
      pc.printf("SRR is %.2f\r\n",SRR.value());
      pc.printf("correction is %.2f\r\n",correction);
      pc.printf("line_postion is %.2f\r\n",line_postion);
      pc.printf("////////////////////////////\r\n");

      wait(1);
    }   
}
