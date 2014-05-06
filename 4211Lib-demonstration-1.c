#include "4211Lib_Gyro.c";
#include "4211Lib_PID.c";

task main()
{


	tSensorContainer c;
	c.sensor = S1;

	float gyroHeading = 0;
	float powerModifier=0;

	// add instances into handlers
	GyroRefrence gyroRef = addNewGyro(c, &gyroHeading);
	PIDRefrence pRef = addNewPID(1.0, 0.05, 10.01, &gyroHeading, &powerModifier);

	// setup the handlers
	setGyroTaskSettings(Hz_100, T3); // poll 50 times a second
	setPIDTaskSettings(Hz_200, T3); // adjust 100 times a second

	StartTask(pidHandler); // launch the handlers
	StartTask(gyroHandler);

	int heading=0; // target heading
	while (!isPIDTaskReady || !isGyroTaskReady)
	{
		nxtDisplayBigTextLine(2, "Waiting...");
	}
	while (true) // drive in a box turning 90 deg every 1 second
	{
		resetGyroHeading(gyroRef);
		setPIDTarget(pRef, 0); // set to no displacement
		time1[T1]=0;
		while(time1[T1] < 1000)
		{
			motor[motorB]=35+powerModifier;
			motor[motorC]=35-powerModifier;
			taskProcessingDone();
		}
		motor[motorB]=0;
		motor[motorC]=0;
		wait1Msec(100);
		resetGyroHeading(gyroRef);
		setPIDTarget(pRef, 90.0); // turn 90 deg right
		while(!pidAtTarget(pRef))
		{
			motor[motorB]=-trim(powerModifier, -20, 20); // max turn rate is +/- 15 power
			motor[motorC]=trim(powerModifier, -20, 20);
			taskProcessingDone();
		}
		motor[motorB]=0;
		motor[motorC]=0;
		wait1Msec(100);
	}
}
