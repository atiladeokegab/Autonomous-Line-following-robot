

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
 double getRPM_M1() {
        int pulses = getPulses_M1();
        double timeElapsed = getTimeElapsed() / 1000.0; // Convert milliseconds to seconds
        double angularVelocity = (2 * M_PI * pulses) / (pulsesPerRev * timeElapsed);
        double rpm = (angularVelocity / (2 * M_PI)) * 60; // Convert angular velocity to RPM
        return rpm;
    }

    // Method to calculate revolutions per minute for motor 2
    double getRPM_M2() {
        int pulses = getPulses_M2();
        double timeElapsed = getTimeElapsed() / 1000.0; // Convert milliseconds to seconds
        double angularVelocity = (2 * M_PI * pulses) / (pulsesPerRev * timeElapsed);
        double rpm = (angularVelocity / (2 * M_PI)) * 60; // Convert angular velocity to RPM
        return rpm;
    }

    // Method to calculate distance traveled for motor 1
    double getDistanceTraveled_M1() {
        int revolutions = getRevolutions_M1();
        double distance = wheelCircumference * revolutions;
        return distance;
    }

    // Method to calculate distance traveled for motor 2
    double getDistanceTraveled_M2() {
        int revolutions = getRevolutions_M2();
        double distance = wheelCircumference * revolutions;
        return distance;
    }

   
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

    // Method to calculate the translational velocity of the robot
    double translational_velocity(double left_wheel_velocity, double right_wheel_velocity){
           return (right_wheel_velocity + left_wheel_velocity) / 2;
    }

private:
    QEI encoder_M1;
    QEI encoder_M2;
    Timer timer;
    double wheel circumference = 2 * M_PI *RADIUS ;
};


