/**************************************************************
 * Class: CSC-615-01 Spring 2020
 * Name: Cody Camp
 * Student ID: *********
 * Github ID: camper4834
 * Project: Robot Car Final Project
 *
 * File: line.c
 *
 * Description: This holds the code for everthing about the line
 * sensors. It is pretty simple. There is a four dimensional matrix
 * that has every line sensor scenario hard coded into it. "What to
 * do if both the left and middle sensors are ON." I simply use
 * the wiringPi reading to call the value from the matrix. This file
 * also makes it easy to start/stop the pthread.
 * 
 **************************************************************/

#include <wiringPi.h>
#include <pthread.h>
#include <stdbool.h>
#include "line.h"

//variable holding the direction to drive
enum direction driveDirection;

//matrix constants
#define _ 0
#define L 1
#define M 1
#define R 1
#define B 1
#define LEFT 2
#define MIDDLE 2
#define RIGHT 2
#define BOTTOM 2

//matrix holding info for what to do at each line sensor situation
const enum direction lineMatrix[LEFT][MIDDLE][RIGHT][BOTTOM] = {
  [_][_][_][_] = None,              // 0
  [L][_][R][_] = None,              // 6
  [L][M][R][_] = None,              //11
  [L][_][R][B] = None,              //13
  [_][M][_][_] = Forward,           // 2
  [_][M][_][B] = Forward,           // 9
  [L][M][R][B] = Forward,           //15
  [_][_][_][B] = Backward,          // 4
  [L][_][_][_] = Left,              // 1
  [L][_][_][B] = LeftLeft,          // 7
  [_][M][R][_] = LeftLeft,          // 8
  [L][M][_][B] = LeftLeftLeft,      //12
  [_][_][R][_] = Right,             // 3
  [L][M][_][_] = RightRight,        // 5
  [_][_][R][B] = RightRight,        //10
  [_][M][R][B] = RightRightRight,   //14
};

bool isRunning = true;
void* lineSensorThread(void* arg) {
  while( isRunning ) {
    driveDirection = lineMatrix
                          [digitalRead(LINE_LEFT_PIN)]
                          [digitalRead(LINE_MIDDLE_PIN)]
                          [digitalRead(LINE_RIGHT_PIN)]
                          [digitalRead(LINE_BOTTOM_PIN)];
  }
  return NULL;
}

pthread_t lineSensorPID;
void startLineSensorThread() {
  pthread_create( &lineSensorPID, NULL, lineSensorThread, NULL );
}

void stopLineSensorThread() {
  isRunning = false;
  pthread_join( lineSensorPID,  NULL );
}