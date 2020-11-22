//
// Created by Brooke Porter on 11/19/20.
//

#ifndef RADICAL_OUTPUTS_OBSTACLE_H
#define RADICAL_OUTPUTS_OBSTACLE_H

// Sensor Pins
static const int ECHO_SENSOR_TRIGGER = 1;
static const int ECHO_SENSOR_RECEIVER = 5;
static const int CLOSE_RANGE_SENSOR = 7;

static const double DISTANCE_THRESHOLD = 30.0;

// Variables
int shouldRun;
int isBlockedByObstacle;

// Methods
void initSensors();
void checkObstacleSensors();
void checkSensors();
void setShouldRun(int flag);

#endif //RADICAL_OUTPUTS_OBSTACLE_H
