//Preprocessor Directives
#include <stdio.h>   //Input&Output
#include <stdlib.h>  //Malloc,Atoi(Converting string commandLine argv[2] into a integer )
#include <pthread.h> //Threading
#include <string.h>  //MemSet,Strcopy,STRMP
#include <time.h>    //Time
#include <unistd.h>  // Close(),Read()
#include <fcntl.h>   //Open(),O_Rdly(flags)
#include <stdbool.h> //Bool
#include <signal.h>  //Signal

#include <Motors/motors.h>

int main()
{
    //*************WIRING_PI_SETUP***********//
    signal(SIGINT, interruptHandlers);
    if (-1 == wiringPiSetup())
    {
        printf("Failed to setup Wiring Pi!\n");
        return 1;
    }
    setup();
    //*************WIRING_PI_SETUP***********//
    pthread_t MotorA, MotorB;
    pthread_create(&MotorA, NULL, &runMotorA, NULL);
    pthread_create(&MotorB, NULL, &runMotorB, NULL);
    delay(2000); //2S
    A_triggerReverse = true;
    delay(2000); //2s
    A_triggerReverse = false;
    delay(2000); //2s
    A_triggerForward = true;
    delay(2000); //2s
    A_triggerForward = false;
    endRun = true;
    pthread_join(MotorA, NULL);
    pthread_join(MotorB, NULL);
    return 0;
}