//Preprocessor Directives
#include <stdio.h>   //Input&Output
#include <stdlib.h>  //Malloc,Atoi(converting string commandLine argv[2] into a integer)
#include <pthread.h> //Threading
#include <string.h>  //MemSet,Strcopy,STRMP
#include <time.h>    //Time
#include <unistd.h>  // Close(),Read()
#include <fcntl.h>   //Open(),O_Rdly(flags)

#include "Sensors/obstacle.h"
#include "Motors/motors.h"
//Definition of Macro

#define PI_SYSTEM 0
//***************************************************************/
#if PI_SYSTEM
#include <wiringPi.h> //Wiring Pi Library
#include <softPwm.h>  //PWM Library
#endif

int main()
{
#if PI_SYSTEM
    //*********************WIRING SETUPS**************************//
    wiringPiSetup();

    initSensors();
    checkObstacleSensors();
//**********************Wiring_SetUp:End**************************//
#endif
    printf("MainThread\n");

    // Testing obstacle detection using both sensors
    for (int i = 0; i < 40; i++)
    {
        printf("Current state of collisions: %d\n", isBlockedByObstacle);
        delay(500);
    }
    setShouldRun(0);

    //MOTOR-TEST-START
    pthread_t MotorA;
    pthread_t MotorB;

    //Set Forward
    triggerForward = true;
    triggerReverse = false;

    //Motors Init
    pthread_create(&MotorA, NULL, &runMotorA, NULL);
    pthread_create(&MotorB, NULL, &runMotorC, NULL);

    //Turn Off Motors Post-5Seconds
    delay(5000);
    endProgram = true;
    //MOTOR_TEST-END
    pthread_join(&MotorA, NULL);
    pthread_join(&MotorB, NULL);

    return 1;
}
