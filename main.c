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
            usleep(1000000); //1S
            softPwmWrite(F_MOT_A, 0);
            softPwmWrite(R_MOT_A, 0);
            softPwmWrite(F_MOT_B, 0);
            softPwmWrite(R_MOT_B, 0);
            softPwmWrite(F_MOT_C, 0);
            softPwmWrite(R_MOT_C, 0);
            softPwmWrite(F_MOT_D, 0);
            softPwmWrite(R_MOT_D, 0);
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
            usleep(1000000); //1sec
            softPwmWrite(VOLT_MOT_A, 75);
            softPwmWrite(VOLT_MOT_B, 25);
            softPwmWrite(VOLT_MOT_C, 75);
            softPwmWrite(VOLT_MOT_D, 40);
            softPwmWrite(F_MOT_A, 75);
            softPwmWrite(F_MOT_B, 0);
            softPwmWrite(F_MOT_C, 75);
            softPwmWrite(F_MOT_D, 0);
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
    ninetyDegLeft = true;
    usleep(1000000);
    haltProgram = true;
    allOff();
    /*******************************END************************/
    pthread_join(MotorThread, NULL); //Main Thread waits for the p1 thread to terminate before continuing main exeuction
    return 0;
}