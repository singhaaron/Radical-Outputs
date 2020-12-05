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
#include <stdio.h>
//#include <wiringPi.h>
//#include <softPwm.h>

bool endRun = false;
bool obstacle = false;
bool A_triggerForward = false;
bool A_triggerReverse = false;
bool B_triggerForward = false;
bool B_triggerReverse = false;
bool C_triggerForward = false;
bool C_triggerReverse = false;
bool D_triggerForward = false;
bool D_triggerReverse = false;

void *runMotorA(void *u)
{
    do
    {
        if (A_triggerForward == true)
        {
            softPwmWrite(VOLT_MOT_A, 100);
            softPwmWrite(F_MOT_A, 100);
        }
        if (A_triggerForward == false | obstacle == true)
        {
            softPwmWrite(VOLT_MOT_A, 0);
            softPwmWrite(F_MOT_A, 0);
        }
        if (A_triggerReverse == true)
        {
            softPwmWrite(VOLT_MOT_A, 100);
            softPwmWrite(R_MOT_A, 100);
        }
        if (A_triggerReverse == false | obstacle == true)
        {
            softPwmWrite(VOLT_MOT_A, 0);
            softPwmWrite(R_MOT_A, 0);
        }
    } while (endRun == false);
    softPwmWrite(VOLT_MOT_A, 0);
    softPwmWrite(F_MOT_A, 0);
    softPwmWrite(R_MOT_A, 0);
    return NULL;
}

void *runMotorB(void *u)
{
    do
    {
        if (B_triggerForward == true)
        {
            softPwmWrite(VOLT_MOT_B, 100);
            softPwmWrite(F_MOT_B, 100);
        }
        if (B_triggerForward == false | obstacle == true)
        {
            softPwmWrite(VOLT_MOT_B, 0);
            softPwmWrite(F_MOT_B, 0);
        }
        if (B_triggerReverse == true)
        {
            softPwmWrite(VOLT_MOT_B, 100);
            softPwmWrite(R_MOT_B, 100);
        }
        if (B_triggerReverse == false | obstacle == true)
        {
            softPwmWrite(VOLT_MOT_B, 0);
            softPwmWrite(R_MOT_B, 0);
        }
    } while (endRun == false);
    softPwmWrite(VOLT_MOT_B, 0);
    softPwmWrite(F_MOT_B, 0);
    softPwmWrite(R_MOT_B, 0);
    return NULL;
}
void *runMotorC(void *u)
{
    do
    {
        if (C_triggerForward == true)
        {
            softPwmWrite(VOLT_MOT_C, 100);
            softPwmWrite(F_MOT_C, 100);
        }
        if (C_triggerForward == false | obstacle == true)
        {
            softPwmWrite(VOLT_MOT_C, 0);
            softPwmWrite(F_MOT_C, 0);
        }
        if (C_triggerReverse == true)
        {
            softPwmWrite(VOLT_MOT_C, 100);
            softPwmWrite(R_MOT_C, 100);
        }
        if (C_triggerReverse == false | obstacle == true)
        {
            softPwmWrite(VOLT_MOT_C, 0);
            softPwmWrite(R_MOT_C, 0);
        }
    } while (endRun == false);
    softPwmWrite(VOLT_MOT_C, 0);
    softPwmWrite(F_MOT_C, 0);
    softPwmWrite(R_MOT_C, 0);
    return NULL;
}
void *runMotorD(void *u)
{
    do
    {
        if (D_triggerForward == true)
        {
            softPwmWrite(VOLT_MOT_D, 100);
            softPwmWrite(F_MOT_D, 100);
        }
        if (D_triggerForward == false | obstacle == true)
        {
            softPwmWrite(VOLT_MOT_D, 0);
            softPwmWrite(F_MOT_D, 0);
        }
        if (D_triggerReverse == true)
        {
            softPwmWrite(VOLT_MOT_D, 100);
            softPwmWrite(R_MOT_D, 100);
        }
        if (D_triggerReverse == false | obstacle == true)
        {
            softPwmWrite(VOLT_MOT_D, 0);
            softPwmWrite(R_MOT_D, 0);
        }
    } while (endRun == false);
    softPwmWrite(VOLT_MOT_D, 0);
    softPwmWrite(F_MOT_D, 0);
    softPwmWrite(R_MOT_D, 0);
    return NULL;
}
