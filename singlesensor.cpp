
// sensor to micro code
// sensor tcrt 5000
// micro stm32f4re


#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
#include <iostream>
#include <fstream> // Include the file stream library

C12832 lcd(D11, D13, D12, D7, D10);
Serial pc(USBTX, NC);
AnalogIn ain(PC_4);


int main(){
    // pins

    lcd.cls();
    while(1){
        // 0 TO 5
        wait(0.1);
        lcd.locate(0,0);
        //lcd.printf("the value is %0.2f/n", ain.read());
        pc.printf("the value is %0.2f\r\n", ain.read()*3.3);
        
    
}
}
