/**************************************************************
* Class:  CSC-615 Fall 2020
* Name:   RadicalOutputz
* Student ID:**********
* Project: Vehicle
*
* File: motors.c
*
* Description:
* Motor Functions
***************************************************************/
#include "motors.h"

/****** RPI L298P MOTOR SHIELD************************************/
//DMX:Direction Motor Type , VMX:Voltage Motor Type, Direction: 0 or 1, High-Forward,Low-Reverse
#define START_MOTOR(DMX, VMX, DIRECTION)           \
    for (int index = 25; index < 125; index += 25) \
    {                                              \
        digitalWrite(DMX, DIRECTION);              \
        softPwmWrite(VMX, index);                  \
        delay(500);                                \
    }
#define STOP_MOTOR(DMX, VMX, DIRECTION)            \
    for (int index = 100; index != 0; index -= 25) \
    {                                              \
        digitalWrite(DMX, DIRECTION);              \
        softPwmWrite(VMX, index);                  \
        delay(500);                                \
    }
/***END****/
/***ANDREW's MOTOR SHIELD************************************/
//MOT_X: Type Motor:Reverse&Forward
#define START_MOTOR_V2(MOT_X)      \
    softPwmWrite(VOLT_MOT_A, 100); \
    softPwmWrite(MOT_X, 100);
#define STOP_MOTOR_V2(MOT_X)     \
    softPwmWrite(VOLT_MOT_A, 0); \
    softPwmWrite(MOT_X, 0);
/***END****/

//Motor A & B - Forward Motion
void *runMotorA()
{
    do
    {
        if (triggerForward)
        {
            START_MOTOR_V2(F_MOT_A)
            START_MOTOR_V2(F_MOT_B)
        }
        if (obstacle)
        {
            STOP_MOTOR_V2(F_MOT_A)
            STOP_MOTOR_V2(F_MOT_B)
            triggerForward = false;
        }
        if (trail)
        {
            STOP_MOTOR_V2(F_MOT_A)
            STOP_MOTOR_V2(F_MOT_B)
            triggerForward = false;
        }
    } while (endProgram == false);
    STOP_MOTOR_V2(F_MOT_A)
    STOP_MOTOR_V2(F_MOT_B)
}
//Motor C & D - Reverse Motion
void *runMotorC()
{
    do
    {
        if (triggerReverse)
        {
            START_MOTOR_V2(R_MOT_A)
            START_MOTOR_V2(R_MOT_B)
        }
        if (obstacle)
        {
            STOP_MOTOR_V2(R_MOT_A)
            STOP_MOTOR_V2(R_MOT_B)
            triggerReverse = false;
        }
        if (trail)
        {
            STOP_MOTOR_V2(R_MOT_A)
            STOP_MOTOR_V2(R_MOT_B)
            triggerReverse = false;
        }
    } while (endProgram == false);
    STOP_MOTOR_V2(R_MOT_A)
    STOP_MOTOR_V2(R_MOT_B)
}