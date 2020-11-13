#include <stdio.h>    //  Standard Input Output
#include <string.h>   //  String
#include <stdlib.h>   // exit()
#include <wiringPi.h> // Wiring Pi Library
#include <softPwm.h>  // PWM signal on GPIO

#define BUTTON_PIN 0         // GPIO 17, Pin 11 , 6Units down
#define DIRECTION_MOTOR_A 26 // 26(Physical 32-GPIO12)
#define VOLTAGE_MOTOR_A 22   // 22 Physical 31 - GPIO6

int main()
{
    /* 
    -Read Button Input
    -Run Forward 2s 
    -Slow it Down 1s 
    -Stop it 1s
    -Reverse it 
    -Stop it 
    */
    wiringPiSetup();
    pinMode(DIRECTION_MOTOR_A, OUTPUT);
    pinMode(VOLTAGE_MOTOR_A, OUTPUT);
    pinMode(BUTTON_PIN, OUTPUT);
    softPwmCreate(VOLTAGE_MOTOR_A, 0, 100);
    digitalWrite(BUTTON_PIN, 1);
    while (1)
    {
        //Button Pressed
        if (digitalRead(BUTTON_PIN) == 0)
        {

            //ClockWise(Forward) HIGH
            //Anti_ClockWise(BackWard) LOW

            delay(5000); //5S to Set 9V in Position
            //Motor Forward
            digitalWrite(DIRECTION_MOTOR_A, HIGH);
            softPwmWrite(VOLTAGE_MOTOR_A, 100);
            delay(2000);
            //Slow the Motor
            softPwmWrite(VOLTAGE_MOTOR_A, 40);
            delay(1000); //Off for 1S
            //Stop Motor
            softPwmWrite(VOLTAGE_MOTOR_A, 0);
            delay(2000);
            //Gradually Reverse Motor
            digitalWrite(DIRECTION_MOTOR_A, LOW);
            int index = 25;
            while (index != 100)
            {
                softPwmWrite(VOLTAGE_MOTOR_A, index);
                index = index + 25;
                delay(1000);
            }
            //Turn Off Motor
            softPwmWrite(VOLTAGE_MOTOR_A, 0);
            delay(2000);
            exit(0);
        }
    }
}
