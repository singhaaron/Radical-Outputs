//Preprocessor Directives
#include <stdio.h>   //Input&Output
#include <stdlib.h>  //Malloc,Atoi(converting string commandLine argv[2] into a integer)
#include <pthread.h> //Threading
#include <string.h>  //MemSet,Strcopy,STRMP
#include <time.h>    //Time
#include <unistd.h>  // Close(),Read()
#include <fcntl.h>   //Open(),O_Rdly(flags)
//Definition of Macros
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
//**********************Wiring_SetUp:End**************************//
#endif
    printf("MainThread\n");
    return 1;
}
