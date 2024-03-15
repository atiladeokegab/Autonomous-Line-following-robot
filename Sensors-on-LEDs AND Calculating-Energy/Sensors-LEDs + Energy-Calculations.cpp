#include "mbed.h"
#include "ds2781.h"
#include "OneWire_Methods.h"

AnalogIn S1_In(NC);
AnalogIn S2_In(NC);
AnalogIn S3_In(NC);
AnalogIn S4_In(NC);
AnalogIn S5_In(NC);

DigitalOut S1_LED(NC);
DigitalOut S2_LED(NC);
DigitalOut S3_LED(NC);
DigitalOut S4_LED(NC);
DigitalOut S5_LED(NC);

DigitalInOut one_wire_pin(NC);

Ticker T;
float threshold = 1.0f, tickerTime_ms = 10.0f, totalEnergy = 0.0f;

void getEnergy() {

    int voltageReading = 0, currentReading = 0;
    float voltage = 0.0f, current = 0.0f;

    voltageReading = ReadVoltage();
    currentReading = ReadCurrent();

    voltage = voltageReading * 0.00967f;
    current = currentReading / 6400.0f;

    totalEnergy += voltageReading * currentReading * tickerTime_ms;

}

int main() {

    T.attach(&getEnergy, tickerTime_ms);

    while (true) {

        S1_LED = (S1_In >= threshold);
        S2_LED = (S2_In >= threshold);
        S3_LED = (S3_In >= threshold);
        S4_LED = (S4_In >= threshold);
        S5_LED = (S5_In >= threshold);
        
    }

}
