#ifndef LIB_ROOT
#define LIB_ROOT

#include "\drivers\hitechnic-sensormux.h";

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

typedef union tSensorContainer;
typedef enum tLoopRate;
int trim(float val, float upperBound, float lowerBound);
bool isInRange(float val, float upperBound, float lowerBound);
bool isOutOfRange(float val, float upperBound, float lowerBound);
bool isAtTarget(float val, float target, float range=1);
bool isValueInArray(int* arrayPtr, int arrayLength, int value);
bool isValueInArray(float* arrayPtr, int arrayLength, float value);
float averageOfArray(int* arrayPtr, int arrayLength);
float averageOfArray(float* arrayPtr, int arrayLength);
void printArray(int* arrayPtr, int arrayLength);
void printArray(float* arrayPtr, int arrayLength);

typedef union tSensorContainer // holds either SMUX or NXT connected sensor.  Simplifies code for accessing later
{
	tSensors sensor;
	tMUXSensor sensor;
}tSensorContainer;

typedef enum tLoopRate
{
	Hz_10 = 100,
	Hz_50 = 20,
	Hz_100 = 10,
	Hz_200 = 5,
	Hz_500 = 2,
	Hz_1000 = 1,
	Hz_Max = 0
}tLoopRate;

int trim(float val, float upperBound, float lowerBound)
{
	return val>upperBound ? upperBound : val<lowerBound ? lowerBound : val;
}

bool isInRange(float val, float upperBound, float lowerBound)
{
	return val<=upperBound && val>=lowerBound;
}

bool isOutOfRange(float val, float upperBound, float lowerBound)
{
	return !isInRange(val, upperBound, lowerBound);
}

bool isAtTarget(float val, float target, float range)
{
	return abs(val-target)<=range;
}

bool isValueInArray(int* arrayPtr, int arrayLength, int value)
{
	for (int i = 0; i < arrayLength; i++)
	{
		if (*(arrayPtr+i)==value)
		{
			return true;
		}
	}
	return false;
}

bool isValueInArray(float* arrayPtr, int arrayLength, float value)
{
	for (int i = 0; i < arrayLength; i++)
	{
		if (*(arrayPtr+i)==value)
		{
			return true;
		}
	}
	return false;
}

bool isValueInArray(char* arrayPtr, int arrayLength, signed char value)
{
	for (int i = 0; i < arrayLength; i++)
	{
		if (*(arrayPtr+i)==value)
		{
			return true;
		}
	}
	return false;
}

float averageOfArray(float* arrayPtr, int arrayLength)
{
	float sum = 0;
	for (int i = 0; i < arrayLength; i++)
	{
		sum += *(arrayPtr+i);
	}
	return sum/arrayLength;
}

float averageOfArray(int* arrayPtr, int arrayLength)
{
	float sum = 0;
	for (int i = 0; i < arrayLength; i++)
	{
		sum += *(arrayPtr+i);
	}
	return sum/arrayLength;
}

void printArray(float* arrayPtr, int arrayLength)
{
	for (int i = 0; i < arrayLength; i++)
	{
		writeDebugStream("%d ,", *(arrayPtr+i));
	}
}

void printArray(int* arrayPtr, int arrayLength)
{
	for (int i = 0; i < arrayLength; i++)
	{
		writeDebugStream("%d ,", *(arrayPtr+i));
	}
	writeDebugStream("\n");
}
#endif
