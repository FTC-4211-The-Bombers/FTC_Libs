#ifndef LIB_TIMER
#define LIB_TIMER

#include "4211Lib_Root.c";

typedef struct tAsyncTimer;
void newAsyncTimer(tAsyncTimer &t, TTimers timer = T4);
void clearAsyncTimer(tAsyncTimer &t);
int getTimeAsyncTimer(tAsyncTimer &t);
bool timerAtTarget(tAsyncTimer &t, int targetTime);
bool timerAtTarget(TTimers timer, int targetTime);
void taskProcessingDone();

typedef struct tAsyncTimer
{
	TTimers timer;
	int lastTime;
}tAsyncTimer;

void newAsyncTimer(tAsyncTimer &t, TTimers timer)
{
	t.timer = timer;
	t.lastTime = time1[timer];
}

void clearAsyncTimer(tAsyncTimer &t)
{
	t.lastTime = time1[t.timer];
}

int getTimeAsyncTimer(tAsyncTimer &t)
{
	return time1[t.timer] - t.lastTime;
}

bool timerAtTarget(tAsyncTimer &t, int targetTime)
{
	return getTimeAsyncTimer(t) >= targetTime;
}

bool timerAtTarget(TTimers timer, int targetTime)
{
	return time1[timer] >= targetTime;
}

void taskProcessingDone()
{
	abortTimeslice();
}
#endif
