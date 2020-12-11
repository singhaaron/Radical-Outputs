//Preprocessor Directives
#include <stdio.h>    //Input&Output
#include <stdlib.h>   //Malloc,Atoi(converting string commandLine argv[2] into a integer)
#include <pthread.h>  //Threading
#include <string.h>   //MemSet,Strcopy,STRMP
#include <time.h>     //Time
#include <unistd.h>   // Close(),Read()
#include <fcntl.h>    //Open(),O_Rdly(flags)
#include <stdbool.h>  //Bool
#include <signal.h>   //Interupt
#include <wiringPi.h> //Pins
#include <softPwm.h>  //Power Output

//Motor Macros
//MOTOR-1
#define F_MOT_A 3
#define R_MOT_A 2
#define VOLT_MOT_A 0
//MOTOR-2
#define F_MOT_B 4
#define R_MOT_B 5
#define VOLT_MOT_B 6
//MOTOR-3
#define F_MOT_C 13
#define R_MOT_C 14
#define VOLT_MOT_C 12
//MOTOR-4
#define F_MOT_D 11
#define R_MOT_D 10
#define VOLT_MOT_D 26

// Sensors
#define ECHO_SENSOR_TRIGGER = 1;
#define ECHO_SENSOR_RECEIVER = 5;
#define CLOSE_RANGE_SENSOR = 7;
#define SERVO_TRIGGER = 25;
#define DISTANCE_THRESHOLD = 30.0;

//Direction
enum direction
{
    None,
    Forward,
    Backward,
    Left,
    LeftLeft,
    LeftLeftLeft,
    Right,
    RightRight,
    RightRightRight
};
//Matrix constants
#define _ 0
#define L 1
#define M 1
#define R 1
#define B 1
#define LEFT 2
#define MIDDLE 2
#define RIGHT 2
#define BOTTOM 2

enum direction lineMatrix[LEFT][MIDDLE][RIGHT][BOTTOM] = {
    [_][_][_][_] = None,            // 0
    [L][_][R][_] = None,            // 6
    [L][M][R][_] = None,            //11
    [L][_][R][B] = None,            //13
    [_][M][_][_] = Forward,         // 2
    [_][M][_][B] = Forward,         // 9
    [L][M][R][B] = Forward,         //15
    [_][_][_][B] = Backward,        // 4
    [L][_][_][_] = Left,            // 1
    [L][_][_][B] = LeftLeft,        // 7
    [_][M][R][_] = LeftLeft,        // 8
    [L][M][_][B] = LeftLeftLeft,    //12
    [_][_][R][_] = Right,           // 3
    [L][M][_][_] = RightRight,      // 5
    [_][_][R][B] = RightRight,      //10
    [_][M][R][B] = RightRightRight, //14
};

//Global Variables
bool haltProgram = false;                                  //Terminate All
enum direction driveDirection;                             //Drive Direction
bool ninetyDegLeft, ninetyDegRight, threesixtyDeg = false; //Static Turns
bool isBlockedByObstacle = false;                          //Obstacle Blocking
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;         //Threading

//Wiring Pi Starter
static void setup()
{
    printf("setting up the program!\n");
    pinMode(F_MOT_A, SOFT_PWM_OUTPUT);
    pinMode(R_MOT_A, SOFT_PWM_OUTPUT);
    pinMode(VOLT_MOT_A, SOFT_PWM_OUTPUT);
    softPwmCreate(F_MOT_A, 0, 100);
    softPwmCreate(R_MOT_A, 0, 100);
    softPwmCreate(VOLT_MOT_A, 0, 100);
    //B
    pinMode(F_MOT_B, SOFT_PWM_OUTPUT);
    pinMode(R_MOT_B, SOFT_PWM_OUTPUT);
    pinMode(VOLT_MOT_B, SOFT_PWM_OUTPUT);
    softPwmCreate(F_MOT_B, 0, 100);
    softPwmCreate(R_MOT_B, 0, 100);
    softPwmCreate(VOLT_MOT_B, 0, 100);
    //C
    pinMode(F_MOT_C, SOFT_PWM_OUTPUT);
    pinMode(R_MOT_C, SOFT_PWM_OUTPUT);
    pinMode(VOLT_MOT_C, SOFT_PWM_OUTPUT);
    softPwmCreate(F_MOT_C, 0, 100);
    softPwmCreate(R_MOT_C, 0, 100);
    softPwmCreate(VOLT_MOT_C, 0, 100);
    //D
    pinMode(F_MOT_D, SOFT_PWM_OUTPUT);
    pinMode(R_MOT_D, SOFT_PWM_OUTPUT);
    pinMode(VOLT_MOT_D, SOFT_PWM_OUTPUT);
    softPwmCreate(F_MOT_D, 0, 100);
    softPwmCreate(R_MOT_D, 0, 100);
    softPwmCreate(VOLT_MOT_D, 0, 100);

    // Sensors
    pinMode(ECHO_SENSOR_TRIGGER, OUTPUT);
    pinMode(ECHO_SENSOR_RECEIVER, INPUT);
    pinMode(CLOSE_RANGE_SENSOR, INPUT);

    // Servo
    pinMode(SERVO_TRIGGER, SOFT_PWM_OUTPUT);
    pullUpDnControl(SERVO_TRIGGER, PUD_OFF);
    softPwmCreate(SERVO_TRIGGER, 0, 50);
    softPwmWrite(SERVO_TRIGGER, 15);
}

static void allOff()
{
    digitalWrite(F_MOT_A, LOW);
    digitalWrite(R_MOT_A, LOW);
    digitalWrite(VOLT_MOT_A, LOW);
    digitalWrite(F_MOT_B, LOW);
    digitalWrite(R_MOT_B, LOW);
    digitalWrite(VOLT_MOT_B, LOW);
    digitalWrite(F_MOT_C, LOW);
    digitalWrite(R_MOT_C, LOW);
    digitalWrite(VOLT_MOT_C, LOW);
    digitalWrite(F_MOT_D, LOW);
    digitalWrite(R_MOT_D, LOW);
    digitalWrite(VOLT_MOT_D, LOW);
}
static void interruptHandlers(const int signals)
{
    printf("turning everything off!\n");
    allOff();
    exit(0);
}
//Motor Function
void *runMotor(void *u)
{
    do
    {
        //Motor Directions
        if (driveDirection == LeftLeft)
        {
            // softPwmWrite(VOLT_MOT_A, 0);
            // softPwmWrite(VOLT_MOT_B, 0);
            // softPwmWrite(VOLT_MOT_C, 0);
            // softPwmWrite(VOLT_MOT_D, 0);
            // softPwmWrite(F_MOT_A, 0);
            // softPwmWrite(R_MOT_A, 0);
            // softPwmWrite(F_MOT_B, 0);
            // softPwmWrite(R_MOT_B, 0);
            // softPwmWrite(F_MOT_C, 0);
            // softPwmWrite(R_MOT_C, 0);
            // softPwmWrite(F_MOT_D, 0);
            // softPwmWrite(R_MOT_D, 0);
            // usleep(1000000); //1s
            //Sharper Left
            softPwmWrite(VOLT_MOT_A, 30);
            softPwmWrite(VOLT_MOT_B, 60);
            softPwmWrite(VOLT_MOT_C, 30);
            softPwmWrite(VOLT_MOT_D, 60);
            softPwmWrite(F_MOT_A, 100);
            softPwmWrite(R_MOT_A, 0);
            softPwmWrite(F_MOT_B, 100);
            softPwmWrite(R_MOT_B, 0);
            softPwmWrite(F_MOT_C, 100);
            softPwmWrite(R_MOT_C, 0);
            softPwmWrite(F_MOT_D, 100);
            softPwmWrite(R_MOT_D, 0);
            // usleep(1000000);                         //1S
            // driveDirection = lineMatrix[0][1][0][0]; //Set Back to Forward
        }
        else if (driveDirection == RightRight)
        {
            // softPwmWrite(VOLT_MOT_A, 0);
            // softPwmWrite(VOLT_MOT_B, 0);
            // softPwmWrite(VOLT_MOT_C, 0);
            // softPwmWrite(VOLT_MOT_D, 0);
            // softPwmWrite(F_MOT_A, 0);
            // softPwmWrite(R_MOT_A, 0);
            // softPwmWrite(F_MOT_B, 0);
            // softPwmWrite(R_MOT_B, 0);
            // softPwmWrite(F_MOT_C, 0);
            // softPwmWrite(R_MOT_C, 0);
            // softPwmWrite(F_MOT_D, 0);
            // softPwmWrite(R_MOT_D, 0);
            // usleep(1000000); //1s
            softPwmWrite(VOLT_MOT_A, 60);
            softPwmWrite(VOLT_MOT_B, 30);
            softPwmWrite(VOLT_MOT_C, 60);
            softPwmWrite(VOLT_MOT_D, 30);
            softPwmWrite(F_MOT_A, 100);
            softPwmWrite(R_MOT_A, 0);
            softPwmWrite(F_MOT_B, 100);
            softPwmWrite(R_MOT_B, 0);
            softPwmWrite(F_MOT_C, 100);
            softPwmWrite(R_MOT_C, 0);
            softPwmWrite(F_MOT_D, 100);
            softPwmWrite(R_MOT_D, 0);
            // usleep(1000000);                         //1s
            // driveDirection = lineMatrix[0][1][0][0]; //Set Back to Forward
        }
        else if (driveDirection == Forward)
        {
            //All Motors Charge Forward
            softPwmWrite(VOLT_MOT_A, 60);
            softPwmWrite(VOLT_MOT_B, 60);
            softPwmWrite(VOLT_MOT_C, 60);
            softPwmWrite(VOLT_MOT_D, 60);
            softPwmWrite(F_MOT_A, 100);
            softPwmWrite(R_MOT_A, 0);
            softPwmWrite(F_MOT_B, 100);
            softPwmWrite(R_MOT_B, 0);
            softPwmWrite(F_MOT_C, 100);
            softPwmWrite(R_MOT_C, 0);
            softPwmWrite(F_MOT_D, 100);
            softPwmWrite(R_MOT_D, 0);
        }
        else if (driveDirection == Backward)
        {
            //Stop 1S
            // usleep(1000000); //1s
            // softPwmWrite(VOLT_MOT_A, 0);
            // softPwmWrite(VOLT_MOT_B, 0);
            // softPwmWrite(VOLT_MOT_C, 0);
            // softPwmWrite(VOLT_MOT_D, 0);
            // softPwmWrite(F_MOT_A, 0);
            // softPwmWrite(R_MOT_A, 0);
            // softPwmWrite(F_MOT_B, 0);
            // softPwmWrite(R_MOT_B, 0);
            // softPwmWrite(F_MOT_C, 0);
            // softPwmWrite(R_MOT_C, 0);
            // softPwmWrite(F_MOT_D, 0);
            // softPwmWrite(R_MOT_D, 0);
            //All Motors Charge Backward
            softPwmWrite(VOLT_MOT_A, 60);
            softPwmWrite(VOLT_MOT_B, 60);
            softPwmWrite(VOLT_MOT_C, 60);
            softPwmWrite(VOLT_MOT_D, 60);
            softPwmWrite(F_MOT_A, 0);
            softPwmWrite(R_MOT_A, 100);
            softPwmWrite(F_MOT_B, 0);
            softPwmWrite(R_MOT_B, 100);
            softPwmWrite(F_MOT_C, 0);
            softPwmWrite(R_MOT_C, 100);
            softPwmWrite(F_MOT_D, 0);
            softPwmWrite(R_MOT_D, 100);
        }
        //Static Turns
        else if (ninetyDegLeft)
        {
            softPwmWrite(VOLT_MOT_A, 25);
            softPwmWrite(VOLT_MOT_B, 75);
            softPwmWrite(VOLT_MOT_C, 50);
            softPwmWrite(VOLT_MOT_D, 75);
            softPwmWrite(F_MOT_A, 10);
            softPwmWrite(F_MOT_B, 75);
            softPwmWrite(F_MOT_C, 25);
            softPwmWrite(F_MOT_D, 75);
            // usleep(1000000); //1S
            // softPwmWrite(F_MOT_A, 0);
            // softPwmWrite(R_MOT_A, 0);
            // softPwmWrite(F_MOT_B, 0);
            // softPwmWrite(R_MOT_B, 0);
            // softPwmWrite(F_MOT_C, 0);
            // softPwmWrite(R_MOT_C, 0);
            // softPwmWrite(F_MOT_D, 0);
            // softPwmWrite(R_MOT_D, 0);
        }

        else if (ninetyDegRight)
        {
            // softPwmWrite(F_MOT_A, 0);
            // softPwmWrite(R_MOT_A, 0);
            // softPwmWrite(F_MOT_B, 0);
            // softPwmWrite(R_MOT_B, 0);
            // softPwmWrite(F_MOT_C, 0);
            // softPwmWrite(R_MOT_C, 0);
            // softPwmWrite(F_MOT_D, 0);
            // softPwmWrite(R_MOT_D, 0);
            // usleep(1000000); //1sec
            softPwmWrite(VOLT_MOT_A, 75);
            softPwmWrite(VOLT_MOT_B, 25);
            softPwmWrite(VOLT_MOT_C, 75);
            softPwmWrite(VOLT_MOT_D, 50);
            softPwmWrite(F_MOT_A, 75);
            softPwmWrite(F_MOT_B, 10);
            softPwmWrite(F_MOT_C, 75);
            softPwmWrite(F_MOT_D, 25);
            // usleep(1000000); //1sec
            // softPwmWrite(F_MOT_A, 0);
            // softPwmWrite(R_MOT_A, 0);
            // softPwmWrite(F_MOT_B, 0);
            // softPwmWrite(R_MOT_B, 0);
            // softPwmWrite(F_MOT_C, 0);
            // softPwmWrite(R_MOT_C, 0);
            // softPwmWrite(F_MOT_D, 0);
            // softPwmWrite(R_MOT_D, 0);
        }
        else if (threesixtyDeg)
        {
            // softPwmWrite(F_MOT_A, 0);
            // softPwmWrite(R_MOT_A, 0);
            // softPwmWrite(F_MOT_B, 0);
            // softPwmWrite(R_MOT_B, 0);
            // softPwmWrite(F_MOT_C, 0);
            // softPwmWrite(R_MOT_C, 0);
            // softPwmWrite(F_MOT_D, 0);
            // softPwmWrite(R_MOT_D, 0);
            // usleep(1000000); //1sec
            softPwmWrite(VOLT_MOT_A, 100);
            softPwmWrite(VOLT_MOT_B, 100);
            softPwmWrite(VOLT_MOT_C, 100);
            softPwmWrite(VOLT_MOT_D, 100);
            softPwmWrite(F_MOT_A, 100);
            softPwmWrite(F_MOT_C, 100);
            softPwmWrite(R_MOT_B, 100);
            softPwmWrite(R_MOT_D, 100);
            usleep(1000000); //1sec
            softPwmWrite(F_MOT_A, 0);
            softPwmWrite(R_MOT_A, 0);
            softPwmWrite(F_MOT_B, 0);
            softPwmWrite(R_MOT_B, 0);
            softPwmWrite(F_MOT_C, 0);
            softPwmWrite(R_MOT_C, 0);
            softPwmWrite(F_MOT_D, 0);
            softPwmWrite(R_MOT_D, 0);
        }

    } while (!haltProgram);
    return NULL;
}

// Obstacle Sensing
void checkSensors() {
    int isWaitingForObstacle = 0;

    while (!haltProgram) {
        // Check close range sensor first, echo sensor has trouble at this range
        if (!digitalRead(CLOSE_RANGE_SENSOR)) {
            pthread_mutex_lock(&mutex);
            isBlockedByObstacle = 1;
            // Wait 5 seconds to check if the obstacle has moved.
            if (!isWaitingForObstacle) {
                delay(5000);
                isWaitingForObstacle = 1;
            } else {
                moveAroundObstacle();
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
                    moveAroundObstacle();
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
    // Hard turn to the left, in place about 90 degrees.
    // This will depend on how far the servo allows the sensor to rotate.
    driveDirection = LeftLeftLeft;
    delay(1000);

    // Rotate right in place until the obstacle is present.
    while (echoSensorDistance() >= DISTANCE_THRESHOLD) {
        driveDirection = RightRightRight;
        delay(500);
    }

    // Soft turn right around the obstacle until we get back to the line.
    while (offTheLine) { // placeholder variable
        driveDirection = RightRight;
    }

    // Center the echo sensor.
    softPwmWrite(SERVO_TRIGGER, 15);
}

int main()
{
    signal(SIGINT, interruptHandlers);
	if(-1 == wiringPiSetup()){                     //check if wiring pi 
							//failed
		printf("Failed to setup Wiring Pi!\n");
		return 1;
	}
    setup();
    pthread_t MotorThread;
    pthread_create(&MotorThread, NULL, &runMotor, NULL);
    /*******************************Test************************/
    // driveDirection = Forward;
    // usleep(700000);
    // driveDirection = RightRight;
    // usleep(700000);
    // driveDirection = LeftLeft;
    // usleep(700000);
    // driveDirection = Backward;
    // usleep(700000);
    ninetyDegRight = true;
    usleep(1000000);
    haltProgram = true;
    allOff();
    /*******************************END************************/

    // Test Obstacle Sensor
    pthread_t obstacleThread;
    haltProgram = false;
    pthread_create(&obstacleThread, NULL, (void *(*)(void *)) &checkSensors, NULL);

    // End Test

    pthread_join(MotorThread, NULL); //Main Thread waits for the p1 thread to terminate before continuing main exeuction
    return 0;
}