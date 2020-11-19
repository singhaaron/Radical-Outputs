LOGFILE=$(LOGPATH)$(shell date)

makeBuild:
	gcc main.c navigation.c -o execMain -l wiringPi -lpthread
# do gitPush USER="INPUT_NAME"
gitPush: 
	git add .
	git commit -m "${LOGFILE}: ${USER}"
	git push -u origin master

gitPull:
	git pull origin master


