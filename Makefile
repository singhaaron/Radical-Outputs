LOGFILE=$(LOGPATH)$(shell date)
SENSORS = Sensors/*.c Sensors/*.h
MOTORS=Motors/motors.c

makeBuild:
	gcc main.c $(MOTORS) $(SENSORS) -o execMain -l wiringPi -lpthread
	gcc motorTest.c $(MOTORS) -o execMotorT -l wiringPi -lpthread

gitPush: 
	git add .
	git commit -m "Message-${MESSAGE}${LOGFILE}: ${USER} "
	git push -u origin master

gitPull:
	git pull origin master



