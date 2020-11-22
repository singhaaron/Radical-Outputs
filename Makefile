LOGFILE=$(LOGPATH)$(shell date)
SENSORS = Sensors/*.c Sensors/*.h


makeBuild:
	gcc main.c navigation.c $(SENSORS) -o execMain -l wiringPi -lpthread
# do gitPush USER="INPUT_NAME"
gitPush: 
	git add .
	git commit -m "${LOGFILE}: ${USER}"
	git push -u origin master

gitPull:
	git pull origin master


