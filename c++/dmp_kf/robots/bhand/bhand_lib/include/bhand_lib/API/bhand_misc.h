#ifndef BHAND_MISC_H
#define BHAND_MISC_H

// some platform dependent includes
#ifdef LINUX
	#include <sys/ioctl.h>
	#include <sys/time.h>
	#include <unistd.h>
#else
	#include <Windows.h>
	#include <conio.h> // for _getch and _kbhit
#endif

#include "stdio.h"


////////////////////////////////////////////////////////////////
// Defines
////////////////////////////////////////////////////////////////

#ifdef LINUX

	#define INFINITE 4294967295U

	#define DELAY(msec) usleep((msec) * 1000);

#else

	//#define DELAY(msec) Sleep(msec);
	#define DELAY(msec) SleepEx(msec, false);

#endif

#ifndef MIN
	#define MIN(x, y) ((x < y) ? x : y)
#endif
#ifndef MAX
	#define MAX(x, y) ((x > y) ? x : y)
#endif

bool isspacechar(const char c);

bool isAlpha2(const char c);

bool searchstrn(char *dest, const char *src, int max);

int countWords(const char *str);

// OS Wrapper Functions
double TimeInSeconds();
int UnbufferedInputStart();
int UnbufferedInputEnd();
int UnbufferedGetChar();

#ifdef LINUX
	int _kbhit();
	int _getch();
#endif


#ifdef LINUX
	#include <sys/time.h>

	typedef struct BHiResTimerStruct
	{
		void Start()
		{
			gettimeofday(&mTimeStart, NULL);
		};
		void Stop()
		{
			gettimeofday(&mTimeStop, NULL);
		};
		double GetDurationInSecs()
		{
			double duration = (double)(mTimeStop.tv_sec - mTimeStart.tv_sec) + (double)(mTimeStop.tv_usec - mTimeStart.tv_usec) / 1000000;
			return duration;
		}
		double GetDurationInMilliSecs()
		{
			double duration = (double)(mTimeStop.tv_sec - mTimeStart.tv_sec) * 1000 + (double)(mTimeStop.tv_usec - mTimeStart.tv_usec) / 1000;
			return duration;
		}
		double GetElapsedInMilliSecs()
		{
			struct timeval mTimeNow;
			gettimeofday(&mTimeNow, NULL);
			return (double)(mTimeNow.tv_sec - mTimeStart.tv_sec) * 1000 + (double)(mTimeNow.tv_usec - mTimeStart.tv_usec) / 1000;
		}
		double GetElapsedInSecs()
		{
			struct timeval mTimeNow;
			gettimeofday(&mTimeNow, NULL);
			return (double)(mTimeNow.tv_sec - mTimeStart.tv_sec) + (double)(mTimeNow.tv_usec - mTimeStart.tv_usec) / 1000000;
		}

		struct timeval mTimeStart;
		struct timeval mTimeStop;
	} BHiResTimer;


#else
	#include <Windows.h>

	typedef struct BHiResTimerStruct
	{
		void Start()
		{
			QueryPerformanceCounter(&mTimeStart);
		};
		void Stop()
		{
			QueryPerformanceCounter(&mTimeStop);
		};
		double GetDurationInSecs()
		{
			LARGE_INTEGER freq;
			QueryPerformanceFrequency(&freq);
			double duration = (double)(mTimeStop.QuadPart - mTimeStart.QuadPart) / (double)freq.QuadPart;
			return duration;
		}
		double GetDurationInMilliSecs()
		{
			LARGE_INTEGER freq;
			QueryPerformanceFrequency(&freq);
			double duration = 1000 * (double)(mTimeStop.QuadPart - mTimeStart.QuadPart) / (double)freq.QuadPart;
			return duration;
		}
		double GetElapsedInMilliSecs()
		{
			LARGE_INTEGER mTimeNow;
			QueryPerformanceCounter(&mTimeNow);
			LARGE_INTEGER freq;
			QueryPerformanceFrequency(&freq);
			return 1000 * (double)(mTimeNow.QuadPart - mTimeStart.QuadPart) / (double)freq.QuadPart;
		}
		double GetElapsedInSecs()
		{
			LARGE_INTEGER mTimeNow;
			QueryPerformanceCounter(&mTimeNow);
			LARGE_INTEGER freq;
			QueryPerformanceFrequency(&freq);
			return (double)(mTimeNow.QuadPart - mTimeStart.QuadPart) / (double)freq.QuadPart;
		}

		LARGE_INTEGER mTimeStart;
		LARGE_INTEGER mTimeStop;
	} BHiResTimer;

#endif

#endif
