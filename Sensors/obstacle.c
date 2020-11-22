//
// Created by Brooke Porter on 11/19/20.
//

#include "obstacle.h"
#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void initSensors() {
    pinMode(ECHO_SENSOR_TRIGGER, OUTPUT);
    pinMode(ECHO_SENSOR_RECEIVER, INPUT);
    pinMode(CLOSE_RANGE_SENSOR, INPUT);

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
    while (shouldRun) {
        // Check close range sensor first, echo sensor has trouble at this range
        if (!digitalRead(CLOSE_RANGE_SENSOR)) {
            printf("Collision Imminent!\n");
            pthread_mutex_lock(&mutex);
            isBlockedByObstacle = 1;
            while (!digitalRead(CLOSE_RANGE_SENSOR));
            isBlockedByObstacle = 0;
            pthread_mutex_unlock(&mutex);
        } else {
            // If we're not preparing for collision, check the distance.
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

            float distance = (((float) timeDifference * 340) / 2) * 100;

            pthread_mutex_lock(&mutex);
            if (distance <= DISTANCE_THRESHOLD) {
                isBlockedByObstacle = 1;
            } else {
                isBlockedByObstacle = 0;
            }
            pthread_mutex_unlock(&mutex);
        }
    }

    pthread_exit(0);
}
