# Robot-maze-solver-
////// link //////
https://www.youtube.com/watch?v=sPMvF2fv_dU
///////////////

#pragma config(Sensor, S1,     Front,          sensorEV3_Ultrasonic)
#pragma config(Sensor, S2,     Right,          sensorEV3_Ultrasonic)
#pragma config(Sensor, S3,     Left,           sensorEV3_Ultrasonic)
#pragma config(Motor,  motorB,          leftServo,     tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorC,          rightServo,    tmotorEV3_Large, PIDControl, encoder)

int minimumSpace = 3;
int regularSpeed = 15;
int turningSpeed = regularSpeed * 2;
int waitTime = 800;
int minimumCourseDistance = 3;
int correctionSpeed = 10;
int adjustWaitTime = 100;
int minimumTurningSpace = 10;
int inifinity = 255;

void adjustCourse() {
//Correct left side of the robot
	if (SensorValue[Left] < minimumCourseDistance) {
		motor[leftServo] += correctionSpeed;
		while (SensorValue[Left] < minimumCourseDistance) {
			wait1Msec(adjustWaitTime);
		}

		while (SensorValue[Left] > minimumSpace && SensorValue[Left] < minimumCourseDistance) {
			motor[leftServo] -= correctionSpeed;
			wait1Msec(adjustWaitTime);
		}
		motor[leftServo] = regularSpeed;
	}

	// Correct right side of the robot
	if (SensorValue[Right] < minimumCourseDistance) {
		motor[rightServo] += correctionSpeed;
		while (SensorValue[Right] < minimumCourseDistance) {
			wait1Msec(adjustWaitTime);
		}
		while (SensorValue[Right] > minimumSpace && SensorValue[Right] < minimumCourseDistance) {
			motor[rightServo] -= correctionSpeed;
			wait1Msec(adjustWaitTime);
		}
		motor[rightServo] = regularSpeed;
	}
}

void TurnRight()
{
  motor[leftServo] = turningSpeed;
  motor[rightServo] = -1 * turningSpeed;
  wait1Msec(waitTime);
}

void TurnLeft()
{
  motor[leftServo] = -1 * turningSpeed;
  motor[rightServo] = turningSpeed;
  wait1Msec(waitTime);
}

void DriveBackward()
{
	  motor[leftServo] = -1 * regularSpeed  ;
	  motor[rightServo] = -1 * regularSpeed ;
    wait1Msec(2000);
    sleep(1);
    if(getUSDistance(Left) > getUSDistance(Right))
	   {
	  	TurnLeft();
	  	sleep(1);
	  }
	 else
	 {
	   TurnRight();
	   sleep(1);
}
}
void DriveForwardUntilWall(int reverseCounter) // main solving method +
{
	// As long as we can keep driving forward
	while(SensorValue[Front] > minimumSpace) {
  motor[leftServo] = regularSpeed;
  motor[rightServo] = regularSpeed;

 	// If there's no space to drive forward, but we can turn
  if ( SensorValue[Front] <= minimumSpace &&
  	SensorValue[Right] > minimumTurningSpace && SensorValue[Left] > minimumTurningSpace) {
     DriveBackward();
     sleep(1);
   }

   // If a wall wasn't discovered (sensor reached max value - 255) turn back
   if ( SensorValue[Front] == inifinity &&
  	SensorValue[Right] > minimumTurningSpace && SensorValue[Left] > minimumTurningSpace) {
     DriveBackward();
     sleep(1);
   }

   // If reached a dead-end, go back
   if ( SensorValue[Front] <= minimumSpace &&
  	SensorValue[Right] <= minimumSpace && SensorValue[Left] <= minimumSpace)
  {
     DriveBackward();
     sleep(1);
   }

   // In case the robot finished the maze
   if ( SensorValue[Front] >= 50 &&
  	SensorValue[Right] >= 50 && SensorValue[Left] >= 50)
     motor[leftServo] = 0;
     motor[rightServo] = 0;
  {

  adjustCourse();

task main()
{
	int reverseCounter = 10;
	while(1) {
		// Continuesly solve the maze
	  DriveForwardUntilWall(reverseCounter);
	  if(getUSDistance(Left) > getUSDistance(Right))
	  {
	  	TurnLeft();
	  	sleep(1);
	  }
	  else
	  {
	    TurnRight();
	    sleep(1);
	  }
  }
}
