#include "mbed.h"
#include "QEI.h"
#include "C12832.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#define SRR_pin PA_0
#define SR_pin  PA_1
#define SC_pin  PA_4
#define SL_pin  PB_0
#define SLL_pin PC_1
#endif
       bool line_detected = 0;
       float correction = 0;
       float value = 0;
       bool last_dir= 0 ;
       float last_line_position = 0;
       int CNYesc[5];
       Serial pc(USBTX, NC);
       const float LINE_THRESHOLD = 0.70;

// Function to normalize sensor output
float normalize(float value, float min_value, float max_value) {
// Perform normalization
return (value - min_value) / (max_value - min_value);
}
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
     // Array to store sensor readings
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
        float weighted_sum  = 0, sum_weights = 0;

        // Sensor positions and their output values
        float sensor_positions[5] = {0, 1000, 2000, 3000, 4000};
        float sensor_outputs[5] = {
            normalize(SC.value(), min_values[0], max_values[0]),
            normalize(SLL.value(), min_values[1], max_values[1]),
            normalize(SL.value(), min_values[2], max_values[2]),
            normalize(SR.value(), min_values[3], max_values[3]),
            normalize(SRR.value(), min_values[4], max_values[4])
        };
        // Store sensor readings in CNYesc[]
        for (int i = 0; i < 5; ++i) {
            CNYesc[i] = sensor_outputs[i] ;
        }
        // Calculate weighted sum
        for (int i = 0; i < 5; ++i) {
            weighted_sum += sensor_positions[i] * sensor_outputs[i];
            sum_weights += sensor_outputs[i];
            if(CNYesc[i] > 0.70){       
             line_detected = 1;
            }     

        }
        // Calculate weighted average
        return weighted_sum / sum_weights;
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



    float compute(float line_position) {
        /*
************************************************************************
The funtions is to calcilate the required voltage to get the dc motot to
the required speed.
************************************************************************
*/
        float proportional_correction  =Kp*line_position;
        dt= 0.01;
        //float e integral = e intergral + e * change in time
        float derivative_correction  = Kd* (line_position) / dt;
        //float output = Kp * error ;
        float correction = proportional_correction + derivative_correction; 
        return correction;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};




// Define constants for array indices
const int SC_INDEX = 0;
const int SLL_INDEX = 1;
const int SL_INDEX = 2;
const int SR_INDEX = 3;
const int SRR_INDEX = 4;



//main
int main() {
    SensorManager sensorManager(SC_pin,SLL_pin,SL_pin,SR_pin,SRR_pin);
    PIDController_Sensors PIDController_sensors(0.1, 0.01); // Adjust PID constants as needed
    //
    //Ticker sensorupdate_M1;
    //sensorupdate_M1.attach(callback(&sensorManager, &SensorManager::calculateError), 0.01); // Attach count_rate_M1

    while (true) {
     float line_position = sensorManager.calculateWeightedAverage() ;
     bool any_sensor_detects_line = false;
    for (int i = 0; i < 5; ++i) {
        if (CNYesc[i] > LINE_THRESHOLD) {
            any_sensor_detects_line = true;
            break;
        }
    }
       if (any_sensor_detects_line) {
        // If a line is detected
        line_position -= 2000; // Assuming the position range is [-2000, 2000]
        last_dir = (line_position >= 0) ? 1 : 0; // Update the last direction
    } else {
        // If no line is detected, maintain the previous direction
        line_position = last_dir ? 2000 : -2000; // Assuming the position range is [-2000, 2000]
    }
        float correction = PIDController_sensors.compute(line_position);

      pc.printf("////////////////////////////\r\n");
      pc.printf("Sll is %i\r\n",CNYesc[SLL_INDEX]);
      pc.printf("Sl is %i\r\n", CNYesc[SL_INDEX]);       
      pc.printf("SC is %i\r\n",  CNYesc[SC_INDEX]);
      pc.printf("SR is %.i\r\n",  CNYesc[SR_INDEX]);
      pc.printf("SRR is %i\r\n",CNYesc[SRR_INDEX]);
      pc.printf("correction is %.2f\r\n",correction);
      pc.printf("line_detected is %d\r\n",line_detected);
      pc.printf("line_detected is %d\r\n",any_sensor_detects_line); // Indicate whether any sensor detects the line
      pc.printf("line_postion is %.2f\r\n",line_position);
      pc.printf("last_dir is %d\r\n",last_dir);
      pc.printf("////////////////////////////\r\n");

      wait(1);
    }   
}
