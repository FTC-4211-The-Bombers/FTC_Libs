#ifndef LIB_GYRO
#define LIB_GYRO

#ifndef MAX_GYROS   // define custom value if more gyro controlers are needed
#define MAX_GYROS 3
#endif


#include "4211Lib_Timer.c";
#include "C:\Program Files (x86)\Robomatter Inc\ROBOTC Development Environment\Sample Programs\NXT\3rd Party Sensor Drivers\drivers\hitechnic-gyro.h";


typedef struct Gyro
{
	tSensorContainer gyroPort;
	float *currHeading; // linked to float val
	float localHeading; // self dependent
	bool active;
}Gyro;

typedef byte GyroRefrence;
GyroRefrence addNewGyro(tSensorContainer gyroPort, float * heading = NULL);
GyroRefrence addNewGyro(Gyro &g);
void calibrateGyro(GyroRefrence ref);
void resetGyroHeading(GyroRefrence ref);
float getGyroHeading(GyroRefrence ref);
void setHeadingLink(GyroRefrence ref, float *heading);
void setGyroTaskSettings(tLoopRate rate = Hz_100, TTimers timer = T4);
bool isGyroOnPort(tSensorContainer gyroPort);

Gyro Gyros[MAX_GYROS];
int __currGyroAddIndex = 0;
tLoopRate __gyroSampleRate = Hz_100;
TTimers __gyroTimer = T4;
bool isGyroTaskReady = false;

GyroRefrence addNewGyro(tSensorContainer gyroPort, float * heading) // returns refrence to the created gyro KEEP THE REFFERENCE TO ACCESS LATER.
{
	if (__currGyroAddIndex>=MAX_GYROS) // trying to create to many gyros
		return 255;
	Gyros[__currGyroAddIndex].gyroPort = S1;
	Gyros[__currGyroAddIndex].localHeading = 0;
	Gyros[__currGyroAddIndex].currHeading = heading;
	Gyros[__currGyroAddIndex].active = true;
	__currGyroAddIndex++;
	return __currGyroAddIndex-1;
}

GyroRefrence addNewGyro(Gyro &g) // returns refrence to the created gyro KEEP THE REFFERENCE TO ACCESS LATER
{
	if (__currGyroAddIndex>=MAX_GYROS) // trying to create to many gyros
		return 255;
	Gyros[__currGyroAddIndex].gyroPort = g.gyroPort;
	Gyros[__currGyroAddIndex].currHeading = g.currHeading;
	Gyros[__currGyroAddIndex].localHeading = g.localHeading;
	Gyros[__currGyroAddIndex].active = g.active;
	__currGyroAddIndex++;
	return __currGyroAddIndex-1;
}

void calibrateGyro(GyroRefrence ref)
{
	if (ref>=__currGyroAddIndex)
		return; // bad refrence given
	Gyros[ref].active = false;  // ensure no access made during callibration
	HTGYROstartCal(Gyros[ref].gyroPort.sensor);
	Gyros[ref].active = true;
}

void resetGyroHeading(GyroRefrence ref)
{
	if (ref>=__currGyroAddIndex)
		return; // bad refrence given
	Gyros[ref].active = false;  // ensure no access made during callibration

	Gyros[ref].localHeading=0;
	if (Gyros[ref].currHeading!=NULL)
		*Gyros[ref].currHeading = 0;
	Gyros[ref].active = true;
}

void setGyroTaskSettings(tLoopRate rate, TTimers timer) // call before starting task to ensure best results
{
	__gyroSampleRate = rate;
	__gyroTimer = timer;
}

void setGyroInactive(GyroRefrence ref)
{
	if (ref>=__currGyroAddIndex)
		return; // bad refrence given
	Gyros[ref].active = false;
}

void setGyroActive(GyroRefrence ref)
{
	if (ref>=__currGyroAddIndex)
		return; // bad refrence given
	Gyros[ref].active = true;
}

void setHeadingLink(GyroRefrence ref, float *heading)
{
	if (ref>=__currGyroAddIndex)
		return; // bad refrence given
	Gyros[ref].active = false;  // ensure no access made during callibration

	Gyros[ref].currHeading=heading;
	Gyros[ref].active = true;
}

bool isGyroOnPort(tSensorContainer gyroPort)
{
	return abs(SensorRaw[gyroPort.sensor] - 620)<20 ? true : false;
}

float getGyroHeading(GyroRefrence ref)
{
	if (ref>=__currGyroAddIndex)
		return 0; // bad refrence given
	return Gyros[ref].localHeading;
}

float __getGyroRotationRate(tSensorContainer gyroPort)
{
	return abs(HTGYROreadRot(gyroPort.sensor)) < 1.5 ? 0 : HTGYROreadRot(gyroPort.sensor); // remove small inconsiquential movements
}

float __getGyroRotationRate(GyroRefrence ref)
{
	if (ref>=__currGyroAddIndex)
		return 0; // bad refrence given
	return __getGyroRotationRate(Gyros[ref].gyroPort);
}

task gyroHandler() // task that handles all the processing for the gyros' headings
{
	tAsyncTimer t;
	newAsyncTimer(t, __gyroTimer);
	int i; //create earlier to maximize loop speeds
	while (__currGyroAddIndex == 0)
	{
		abortTimeslice(); // while nothing to do don't waste resources
	}
	for (i = 0; i < __currGyroAddIndex; i++) // calibrate the gyros
	{
		if (Gyros[i].active) // skip if something being done to it or not needed
		{
			calibrateGyro(i);
		}
	}
	isGyroTaskReady=true;
	while (true)
	{
		clearAsyncTimer(t);
		while (getTimeAsyncTimer(t) < (int) __gyroSampleRate)
		{
			abortTimeslice(); // don't need to process so skip
		}
		for (i = 0; i < __currGyroAddIndex; i++)
		{
			if (Gyros[i].active) // skip if something being done to it or not needed
			{
				Gyros[i].localHeading += __getGyroRotationRate(i)*(float)(getTimeAsyncTimer(t)/1000.0);
				if (Gyros[i].currHeading!=NULL)
					*Gyros[i].currHeading = Gyros[i].localHeading;
			}
		}
	}
}

#endif
