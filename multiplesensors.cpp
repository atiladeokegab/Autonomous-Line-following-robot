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
       float count_rate_M1_l = 0;
       float count_rate_M2_r = 0;
       float dutyCycleLeft = 0;
       float dutyCycleRight = 0;
       float errorValue  = 0;
       float speed_right = 3;
       float speed_left = 3 ;
       float correction = 0;

Serial pc(USBTX, NC);

/*
************************************************************************
In this program we aim to adjust the veloicity of each motor indepndatlty to 
achieve a desired speed.
step 1
get encoder reading
step 2 
calulate the encoder_rate
step 3
calculate the velocity
step 4
when desired velocity is reached  the buggy the buggy will make a staight line
step 5 
calculate the angular velocity
step 6
make 90 degreee turn to make the buggy move in the intended direction
step 7 
with this draw a square with the algorithm below

___________________  *assuming the buggy starts from here 
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
final implementation should look like 

        B.line2(0.85f, 10, 0.2f, 10);

        // make the buggy turn
        B.turn(0.85f, 10);

        // make the buggy move in a straight line.
        B.line2(0.85f, 10, 0.2f, 10);
    
         // make the buggy turn
        B.turn(0.85f, 10);
        
        // make the buggy move in a straight line.
        B.line2(0.85f, 10, 0.2f, 10);

        // make the buggy turn
        B.turn(0.85f, 10);

        // make the buggy move in a straight line.
        B.line2(0.85f, 10, 0.2f, 10);

        // make the buggy turn
        B.turn(0.85f, 10);

        //make buggy stop
        B.halt(0.85f, 10)
*************************************************************
                      pseudo code
*************************************************************
function main():
    initialize_buggy()  // initialize the buggy and its components

    for i = 1 to 4:  // repeat 4 times to draw a square
        move_forward()  // move the buggy forward
        make_turn()     // make a 90-degree turn

    halt_buggy()  // stop the buggy after drawing the square

function move_forward():
    desired_velocity = calculate_velocity()  // calculate desired velocity
    while not at_desired_velocity():
        encoder_reading = get_encoder_reading()
        encoder_rate = calculate_encoder_rate(encoder_reading)
        velocity = calculate_velocity(encoder_rate)
        adjust_motor_velocity(velocity)

function make_turn():
    desired_angular_velocity = calculate_angular_velocity()  // calculate desired angular velocity
    while not at_desired_angular_velocity():
        encoder_reading = get_encoder_reading()
        encoder_rate = calculate_encoder_rate(encoder_reading)
        angular_velocity = calculate_angular_velocity(encoder_rate)
        adjust_motor_angular_velocity(angular_velocity)

// Functions below are placeholders and need to be implemented based on hardware and algorithms
function get_encoder_reading():
    // read encoder values from the motors and return them

function calculate_encoder_rate(encoder_reading):
    // calculate the rate of change of encoder readings

function calculate_velocity(encoder_rate):
    // calculate the linear velocity based on encoder rates

function adjust_motor_velocity(velocity):
    // adjust motor velocities to achieve the desired linear velocity

function calculate_angular_velocity(encoder_rate):
    // calculate the angular velocity based on encoder rates

function adjust_motor_angular_velocity(angular_velocity):
    // adjust motor velocities to achieve the desired angular velocity

function at_desired_velocity():
    // check if the buggy has reached the desired linear velocity

function at_desired_angular_velocity():
    // check if the buggy has reached the desired angular velocity

function initialize_buggy():
    // initialize the buggy and its components

function halt_buggy():
    // stop the buggy

// Call the main function to start the program
main()
************************************************************************
*/
/*
start of code
*/
//Wheel class to control individual wheels
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

// Buggy class to control the entire buggy
class Buggy {
    /*
    Duty Cycle:
    Effect on speed: Increasing the duty cycle increases the average voltage applied to the motor, 
    which in turn increases the speed. Conversely, decreasing the duty cycle decreases the average 
    voltage and thus the speed.

    Period:
    Effect on speed: 
    The period affects the frequency of the PWM signal.
    A shorter period means the PWM signal switches on and off more rapidly.
    This rapid switching can affect motor performance, particularly at lower speeds.
    However, the period itself doesn't directly affect the speed as much as the duty cycle does.
    */

private:
    Wheel L, R;

public:

    // Constructor setting the bipolar, direction and pwm pins for the left and right wheels.
    Buggy(PinName L_BP, PinName L_WD, PinName L_PWM, PinName R_BP, PinName R_WD, PinName R_PWM) : L(L_BP, L_WD, L_PWM), R(R_BP, R_WD, R_PWM) {}

    void turn(float DC, int t) {

    // This line is commented out because 'Enc' is not defined in this scope.

        L.configure(DC, t);
        R.configure(DC, t);

    }

    void line2(float DC_L, int period_L, float DC_R, int period_R) {

        L.configure(DC_L, period_L);
        R.configure(DC_R, period_R);



    }

    void halt() {
         // 0.5, 1, 0.5, 1
        L.configure(0.5, 1);
        R.configure(0.5, 1);

        wait(1.0f);

    }

};

class Potentiometer // Begin Potentiometer class definition
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
        //2) measure the time elapsed between triggerss

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


/*
************************************************************************
The Pid controller class aim to help the buggy to get to a desired speed.
The idea here is to adjust the duty cylce to achieve a desired speed.


************************************************************************
*/
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
    PIDController(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd), setpoint(0), integral(0), prevError(0) {}

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
        float error =setpoint- velocity;
        //float error = setpoint - velocity;
        //float error = desired velocity - actual velocity;
        dt= 0.1;
        integral += (error*dt) ;

        //float e integral = e intergral + e * change in time

        float derivative = (error - prevError) / dt;
        prevError = error;
        //float output = Kp * error ;
        float output = Kp * error + Ki * integral + Kd * derivative;

        if (output < -0.5){ 
        output == 0.5;
        }
        else if (output> 0.5) {
        output == 0.5;
        }

        return output;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};
// code to get the sensors to work
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



/*
int main() {
    float center_val;
    Sensor center(PC_5);

    while (1) {
        center_val = center.value();

        lcd.locate(0,0);
        lcd.printf("value is %.2f", center_val);

    
    }

}
*/
/*
function move_forward():
    desired_velocity = calculate_velocity()  // calculate desired velocity
    while not at_desired_velocity():
        encoder_reading = get_encoder_reading()
        encoder_rate = calculate_encoder_rate(encoder_reading)
        velocity = calculate_velocity(encoder_rate)
        adjust_motor_velocity(velocity)

function make_turn():
    desired_angular_velocity = calculate_angular_velocity()  // calculate desired angular velocity
    while not at_desired_ang+ular_velocity():
        encoder_reading = get_encoder_reading()
        encoder_rate = calculate_encoder_rate(encoder_reading)
        angular_velocity = calculate_angular_velocity(encoder_rate)
        adjust_motor_angular_velocity(angular_velocity)
        */



int main() {
   
    C12832 lcd(D11, D13, D12, D7, D10);
    Buggy buggy(PA_13, PA_14, PA_15, PB_2, PB_1, PB_15);
    DigitalOut enable(PA_8);
    SensorManager sensorManager(SRR_pin,SR_pin,SC_pin,SL_pin,SLL_pin);
 
   // enable = 1;
    MotorEncoder motorEncoders(PC_5, PC_4, PC_2, PC_3, 1024, QEI::X4_ENCODING);
    // Initialize PID controllers for each wheel
    PIDController pidControllerLeft(0.1,10 , 1); // Adjust PID constants as needed
    PIDController pidControllerRight(0.1,10, 1); // Adjust PID constants as needed
    PIDController pidControllerLeft_turn(0.1, 0.01, 0.01); // Adjust PID constants as needed
    PIDController pidControllerRight_turn(0.1, 0.01, 0.01); // Adjust PID constants as needed
    PIDController_Sensors PIDController_sensors(0.1, 0.01); // Adjust PID constants as needed
    //
    Ticker sensorupdate_M1;
    sensorupdate_M1.attach(callback(&sensorManager, &SensorManager::calculateError), 0.01); // Attach count_rate_M1
    Ticker ticker;    
    ticker.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M1), 0.1); // Attach count_rate_M1
    Ticker ticker2;
    ticker2.attach(callback(&motorEncoders, &MotorEncoder::count_rate_M2), 0.1); // Attach count_rate_M2
  
    //pidControllerRight_turn.setSetpoint(3.3);
    //pidControllerLeft_turn.setSetpoint(-3.3);
// code that should draw a box p1 -F
    //int moveIndex = 0;

    pidControllerRight.setSetpoint(speed_right);
    pidControllerLeft.setSetpoint(speed_left);
lcd.cls();
    Sensor SC(PA_4);
    Sensor SLL(PC_1);
    Sensor SL(PB_0);
    Sensor SR(PA_1);
    Sensor SRR(PA_0);
    

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

        float setpointLeft = 1.0 - errorValue ;
        float setpointRight = 1.0 + errorValue ;
        float pidOutputLeft = pidControllerLeft.compute(setpointLeft);
        float pidOutputRight = pidControllerRight.compute(setpointRight);
        float pidControllerLeft_turn_k = pidControllerLeft_turn.compute(wheelVelocity_M1);
        float pidControllerRight_turn_k = pidControllerRight_turn.compute(wheelVelocity_M2);
        float dutyCycleLeft =  0.5 + pidOutputLeft; // Base duty cycle + PID output
        float dutyCycleRight = 0.5 + pidOutputRight; // Base duty cycle + PID output
        //float dutyCycleRight_turn = 0.5 + pidControllerLeft_turn_k; // Base duty cycle + PID output
        //float dutyCycleleft_turn = 0.5 + pidControllerLeft_turn_k; // Base duty cycle + PID output
        // Compute PID controller outputs for each whee
        
        if (correction > 0){ 
        buggy.line2(speed_left,50, speed_right-correction,50);
        }
    else {
        buggy.line2(speed_left+correction,50 , speed_right, 50);
        }

        //lcd.locate(0,0);
       // lcd.printf(" %i, %.2f,%.2f   ",getPulses_M1, count_rate_M1_l,wheelVelocity_M1 );
       // lcd.locate(0,10);
       // lcd.printf(" %i, %.2f, %.2f ",getPulses_M2, count_rate_M2_r, wheelVelocity_M2);
       // lcd.locate(0,20);
       // lcd.printf("%.2f, %.2f ",translational_velocity, angularVelocity);

        //lcd.locate(0,0);
       // lcd.printf("%.2f,%.2f   ",dutyCycleLeft, dutyCycleRight);
        //lcd.locate(0,20);
       // lcd.printf("%.2f,%.2f   ",wheelVelocity_M1, wheelVelocity_M2);
  
       // buggy.halt();     
    pc.printf("////////////////////////////\r\n");
      pc.printf("Sll is %.2f\r\n",SLL.value());
      pc.printf("Sl is %.2f\r\n", SL.value());       
      pc.printf("SC is %.2f\r\n", SC.value());
      pc.printf("SR is %.2f\r\n", SR.value());
      pc.printf("SRR is %.2f\r\n",SRR.value());
      pc.printf("speed_right is %.2f\r\n",wheelVelocity_M1);
      pc.printf("speed_left is %.2f\r\n",wheelVelocity_M2);
     pc.printf("correction is %.2f\r\n",correction);
    pc.printf("Sll is %.2f\r\n",SLL.value());
    pc.printf("////////////////////////////\r\n");

      wait(2);
    }   
}
