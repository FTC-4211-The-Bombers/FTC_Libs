#ifndef LIB_PID
#define LIB_PID

#ifndef MAX_PIDS   // define custom value if more PID controlers are needed
#define MAX_PIDS 8
#endif

#include "4211Lib_Timer.c";

typedef struct PID
{
	float kp;
	float ki;
	float kd;
	float lastError;
	float integral;
	float derivitive;
	float target;
	float *linkedInput; // dependent on another input var
	float *linkedOutput; // dependent on another output var. (Can be set to NULL if not used)
	float output;  // local stored copy of output
	bool active;
}PID;

typedef byte PIDRefrence;
void setPIDTaskSettings(tLoopRate rate = Hz_100, TTimers timer = T4);
PIDRefrence addNewPID(float kp, float ki, float kd, float *input, float *output = NULL);
PIDRefrence addNewPID(PID &p);
void setPIDTarget(PIDRefrence ref, float target = 0);
void resetPIDController(PIDRefrence ref);
void setPIDInactive(PIDRefrence ref);
void setPIDActive(PIDRefrence ref);
void setInputLink(PIDRefrence ref, float *input);
void setOutputLink(PIDRefrence ref, float *output);
bool pidAtTarget(PIDRefrence ref, float range=0.5);

PID PIDs[MAX_PIDS];
int __currPIDAddIndex = 0;
tLoopRate __pidSampleRate = Hz_100;
TTimers __pidTimer = T4;
bool isPIDTaskReady=false;

void setPIDTaskSettings(tLoopRate rate, TTimers timer) // call before starting task to ensure best results
{
	__pidSampleRate = rate;
	__pidTimer = timer;
}

PIDRefrence addNewPID(float kp, float ki, float kd, float *input, float *output)
{
	if (__currPIDAddIndex>=MAX_PIDS || input == NULL) // trying to create to many PID Controllers
		return 255;
	PIDs[__currPIDAddIndex].kp = kp;
	PIDs[__currPIDAddIndex].ki = ki;
	PIDs[__currPIDAddIndex].kd = kd;
	PIDs[__currPIDAddIndex].lastError = 0;
	PIDs[__currPIDAddIndex].integral = 0;
	PIDs[__currPIDAddIndex].derivitive = 0;
	PIDs[__currPIDAddIndex].target = 0;
	PIDs[__currPIDAddIndex].linkedInput = input;
	PIDs[__currPIDAddIndex].linkedOutput = output;
	*PIDs[__currPIDAddIndex].linkedOutput = 0;
	PIDs[__currPIDAddIndex].output = 0;
	PIDs[__currPIDAddIndex].active = true;
	__currPIDAddIndex++;
	return __currPIDAddIndex-1;
}

PIDRefrence addNewPID(PID &p)
{
	if (__currPIDAddIndex>=MAX_PIDS || p.linkedInput == NULL) // trying to create to many PID Controllers
		return 255;
	PIDs[__currPIDAddIndex].kp = p.kp;
	PIDs[__currPIDAddIndex].ki = p.ki;
	PIDs[__currPIDAddIndex].kd = p.kd;
	PIDs[__currPIDAddIndex].lastError = p.lastError;
	PIDs[__currPIDAddIndex].integral = p.integral;
	PIDs[__currPIDAddIndex].derivitive = p.derivitive;
	PIDs[__currPIDAddIndex].target = p.target;
	PIDs[__currPIDAddIndex].linkedInput = p.linkedInput;
	PIDs[__currPIDAddIndex].linkedOutput = p.linkedOutput;
	*PIDs[__currPIDAddIndex].linkedOutput = 0;
	PIDs[__currPIDAddIndex].output = p.output;
	PIDs[__currPIDAddIndex].active = p.active;
	__currPIDAddIndex++;
	return __currPIDAddIndex-1;
}

void resetPIDController(PIDRefrence ref)
{
	if (ref>=__currPIDAddIndex)
		return; // bad refrence given
	PIDs[ref].active = false;
	PIDs[ref].lastError = 0;
	PIDs[ref].integral = 0;
	PIDs[ref].derivitive = 0;
	*PIDs[ref].linkedOutput = 0;
	PIDs[ref].output = 0;
	PIDs[ref].active = true;
}

void setPIDTarget(PIDRefrence ref, float target)
{
	if (ref>=__currPIDAddIndex)
	  return; // bad refrence given
	PIDs[ref].active = false;
	PIDs[ref].lastError = 0;
	PIDs[ref].integral = 0;
	PIDs[ref].derivitive = 0;
	PIDs[ref].target = target;
	*PIDs[ref].linkedOutput = 0;
	PIDs[ref].output = 0;
	PIDs[ref].active = true;
}

void setPIDInactive(PIDRefrence ref)
{
	if (ref>=__currPIDAddIndex)
		return; // bad refrence given
	PIDs[ref].active = false;
}

void setPIDActive(PIDRefrence ref)
{
	if (ref>=__currPIDAddIndex)
		return; // bad refrence given
	PIDs[ref].active = true;
}

void setInputLink(PIDRefrence ref, float *input)
{
	if (ref>=__currPIDAddIndex)
		return; // bad refrence given
	PIDs[ref].active = false;  // ensure no access made during changing

	PIDs[ref].linkedInput=input;
	PIDs[ref].active = true;
}

void setOutputLink(PIDRefrence ref, float *output)
{
	if (ref>=__currPIDAddIndex)
		return; // bad refrence given
	PIDs[ref].active = false;  // ensure no access made during changing
	PIDs[ref].linkedOutput=output;
	PIDs[ref].active = true;
}

float getPIDOutput(PIDRefrence ref)
{
	if (ref>=__currPIDAddIndex)
		return 0; // bad refrence given
	return PIDs[ref].output;
}

bool pidAtTarget(PIDRefrence ref, float range)
{
	if (ref>=__currPIDAddIndex)
		return false; // bad refrence given
	return isAtTarget(*(PIDs[ref].linkedInput), PIDs[ref].target, range);
}
task pidHandler() // task that handles all the processing for the gyros' headings
{
	tAsyncTimer t;
	newAsyncTimer(t, __pidTimer);
	int i; //create earlier to maximize loop speeds
	float currError;
	while (__currPIDAddIndex == 0)
	{
		abortTimeslice(); // while nothing to do don't waste resources
	}
	isPIDTaskReady=true;
	while (true)
	{
		clearAsyncTimer(t);
		while (getTimeAsyncTimer(t) < (int) __pidSampleRate)
		{
			abortTimeslice(); // don't need to process so skip
		}
		for (i = 0; i < __currPIDAddIndex; i++)
		{
			if (PIDs[i].active) // skip if something being done to it or not needed
			{
				currError = PIDs[i].target-*PIDs[i].linkedInput;
				PIDs[i].integral+=currError*(getTimeAsyncTimer(t))/1000; // calculate integral term using elapsed time on specified timer
				PIDs[i].derivitive=(currError-(PIDs[i].lastError))/((float)getTimeAsyncTimer(t)); // calculate derivitive term using elapsed time on specified timer
				PIDs[i].lastError = currError;
				PIDs[i].output = (PIDs[i].kp) * currError + (PIDs[i].ki)*(PIDs[i].integral) + (PIDs[i].kd)*(PIDs[i].derivitive);
				if (PIDs[i].linkedOutput!=NULL)
					*PIDs[i].linkedOutput = PIDs[i].output;
			}
		}
	}
}

#endif
