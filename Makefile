makeBuild:
	gcc main.c -o execMain -l wiringPi -lpthread

gitPush: 
	git add .
	git commit -m "Update"
	git push -u origin master

gitPull:
	git pull origin master

