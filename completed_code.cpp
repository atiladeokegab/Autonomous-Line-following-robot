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
bool line_detected = 1;
float correction = 0;
float value = 0;
bool last_dir ;
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
int CNYesc[5];
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
        // Store sensor readings in CNYesc[]
        for (int i = 0; i < 5; ++i) {
            CNYesc[i] = sensor_outputs[i];
        }
        // Calculate weighted sum
        for (int i = 0; i < 5; ++i) {
            x_sum += sensor_positions[i] * sensor_outputs[i];
            y_sum += sensor_outputs[i];
            if(CNYesc[i] > 750){       //needs testing
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
    Buggy buggy(PA_13, PA_14, PA_15, PB_2, PB_1, PB_15);
    DigitalOut enable(PA_8);
    enable = 1;
    MotorEncoder motorEncoders(PC_5, PC_4, PC_2, PC_3, 1024, QEI::X4_ENCODING);
    // Initialize PID controllers for each wheel
    
    PIDController pidControllerLeft(1); // Adjust PID constants as needed
    PIDController pidControllerRight(1); // Adjust PID constants as needed
    pidControllerLeft.setSetpoint(1.5);
    pidControllerRight.setSetpoint(1.5);
    SensorManager sensorManager(SRR_pin,SR_pin,SC_pin,SL_pin,SLL_pin);
    PIDController_Sensors PIDController_sensors(0.1, 0.01); // Adjust PID constants as needed
    //
    Ticker sensorupdate_M1;
    sensorupdate_M1.attach(callback(&sensorManager, &SensorManager::calculateError), 0.001); // Attach count_rate_M1
    Ticker ticker;    
    ticker.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M1), 0.01); // Attach count_rate_M1
    Ticker ticker2;
    ticker2.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M2), 0.01); // Attach count_rate_M2

    while (true) {
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
      
   
     float line_position = sensorManager.calculateWeightedAverage() ;
              if(line_detected == 1) {
            // If there is a line_detected
            line_position = line_position - (5 - 1) / 2 * 1000; // Convert to error range
            last_dir = (line_position >= 0) ? 1 : 0; // Update the last direction
        } else {
            // If no line is detected
            line_position = 0 + (5 - 1) * 1000 * last_dir; // Use last direction information
            line_position = line_position - (5 - 1) / 2 * 1000; // Convert to error range
        }
        //float correction = PIDController_sensors.compute(line_position, last_line_position);
        //last_line_position = line_position; // Update last_line_position
    if (correction >0){
         buggy.line2(dutyCycleLeft,50,dutyCycleRight-correction,50);
    }
       else {
          buggy.line2(dutyCycleLeft + correction,50,dutyCycleRight,50);
       }
          
if (wheelVelocity_M1 < wheelVelocity_M1_past - 0.3 && wheelVelocity_M2 == wheelVelocity_M2_past - 0.3 && line_detected == 1) {
    pidControllerLeft.setSetpoint(2.5);
    pidControllerRight.setSetpoint(2.5);
}
    pc.printf("Sensor info\r\n");
    pc.printf("////////////////////////////\r\n");
    pc.printf("correction is %.2f\r\n",correction);
    pc.printf("line_detected is %.2f\r\n",line_detected);
    pc.printf("line_postion is %.2f\r\n",line_position);
    pc.printf("last_dir is %.2f\r\n",last_dir);
    pc.printf("////////////////////////////\r\n");
    pc.printf("left motor info\r\n");
    pc.printf("////////////////////////////\r\n");
    pc.printf("speed_left is %.2f\r\n",wheelVelocity_M1);
    pc.printf("dutyCycleLeft is %.2f\r\n",dutyCycleLeft);
    pc.printf("////////////////////////////\r\n");
    pc.printf("Right motor info\r\n");
    pc.printf("////////////////////////////\r\n");
    pc.printf("speed_right is %.2f\r\n",wheelVelocity_M2);
    pc.printf("dutyCycleRight is %.5f\r\n",dutyCycleRight);
    pc.printf("////////////////////////////\r\n");
        
      wait(1);
    }   
}
