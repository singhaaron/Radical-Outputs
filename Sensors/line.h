#ifndef LINE_H
#define LINE_H

#define LINE_LEFT_PIN 1
#define LINE_MIDDLE_PIN 2
#define LINE_RIGHT_PIN 3
#define LINE_BOTTOM_PIN 4

enum direction {
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

extern enum direction driveDirection;

void startLineSensorThread();
void stopLineSensorThread();

#endif // LINE_H