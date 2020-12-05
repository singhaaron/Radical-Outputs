//
// Created by Brooke Porter on 11/19/20.
//

#include "obstacle.h"
#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <softPwm.h>

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void initSensors() {
    pinMode(ECHO_SENSOR_TRIGGER, OUTPUT);
    pinMode(ECHO_SENSOR_RECEIVER, INPUT);
    pinMode(CLOSE_RANGE_SENSOR, INPUT);

    pinMode(SERVO_TRIGGER, SOFT_PWM_OUTPUT);
    pullUpDnControl(SERVO_TRIGGER, PUD_OFF);
    softPwmCreate(SERVO_TRIGGER, 0, 50);
    softPwmWrite(SERVO_TRIGGER, 15);

    shouldRun = 1;
    isBlockedByObstacle = 0;
}

void setShouldRun(int flag) {
    shouldRun = flag;
}

void checkObstacleSensors() {
    pthread_t id;
    pthread_create(&id, NULL, (void *(*)(void *)) &checkSensors, NULL);
}

void checkSensors() {
    int isWaitingForObstacle = 0;

    while (shouldRun) {
        // Check close range sensor first, echo sensor has trouble at this range
        if (!digitalRead(CLOSE_RANGE_SENSOR)) {
            pthread_mutex_lock(&mutex);
            isBlockedByObstacle = 1;
            // Wait 5 seconds to check if the obstacle has moved.
            if (!isWaitingForObstacle) {
                delay(5000);
                isWaitingForObstacle = 1;
            } else {
//                moveAroundObstacle();
            }
            pthread_mutex_unlock(&mutex);
        } else {
            // If we're not preparing for collision, check the distance.
            pthread_mutex_lock(&mutex);
            if (echoSensorDistance() <= DISTANCE_THRESHOLD) {
                isBlockedByObstacle = 1;
                // Wait 5 seconds to check if the obstacle has moved.
                if (!isWaitingForObstacle) {
                    delay(5000);
                    isWaitingForObstacle = 1;
                } else {
//                    moveAroundObstacle();
                }
            } else {
                isBlockedByObstacle = 0;
                isWaitingForObstacle = 0;
            }
            pthread_mutex_unlock(&mutex);
        }
    }

    pthread_exit(0);
}

float echoSensorDistance() {
    digitalWrite(ECHO_SENSOR_TRIGGER, HIGH);
    delay(1);
    digitalWrite(ECHO_SENSOR_TRIGGER, LOW);

    clock_t start = clock();
    clock_t stop = clock();

    // As long as the echo pin reads 0, keep resetting the start time.
    // Once the echo pin reads 1, we got a hit, so we should start the timer
    // only once the pin switches.
    while (digitalRead(ECHO_SENSOR_RECEIVER) == 0) {
        start = clock();
    }

    while (digitalRead(ECHO_SENSOR_RECEIVER) == 1) {
        stop = clock();
    }

    double timeDifference = ((double) (stop - start)) / CLOCKS_PER_SEC;
    return (((float) timeDifference * 340) / 2) * 100;
}

void moveAroundObstacle() {
    // Turn the echo sensor fully to the right.
    softPwmWrite(SERVO_TRIGGER, 25);
    // Hard turn to the left, in place.
//    turnLeft();

    // If we can still see the obstacle on the right of the vehicle, turn soft right while moving forward.
    // TODO: this will need to be fine tuned once we see how the car handles.
    while (echoSensorDistance() <= DISTANCE_THRESHOLD) {
//        turnRight();
    }

    // Soft turn right until we get back to the line.
    // TODO: I think this will just turn circles around the obstacle forever. Will need some work.
//    while (offTheLine) {
//        moveRight();
//    }

}

void testServoMotor() {
    printf("Testing Servo motor\n");
    // Max to one side
    softPwmWrite(SERVO_TRIGGER, 25);
    delay(2000);
    // Max to the other
    softPwmWrite(SERVO_TRIGGER, 5);
}
