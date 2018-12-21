#ifndef TIMER_H
#define TIMER_H

#ifdef WIN32 // Windows system specific
//#include <windows.h>
#else // Unix based system specific
#include <sys/time.h>
#endif

#include <stdlib.h>


class Timer
{
public:
// default constructor
  Timer()
  {
#ifdef WIN32
    QueryPerformanceFrequency(&frequency);
    startCount.QuadPart = 0;
    endCount.QuadPart = 0;
#else
    startCount.tv_sec = startCount.tv_usec = 0;
    endCount.tv_sec = endCount.tv_usec = 0;
#endif

    stopped = 0;
    startTimeInMicroSec = 0;
    endTimeInMicroSec = 0;

    start();
  }

  // default destructor
  ~Timer()
  {
  }

  ///////////////////////////////////////////////////////////////////////////////
  // start timer.
  // startCount will be set at this point.
  ///////////////////////////////////////////////////////////////////////////////
  void start()
  {
    stopped = 0; // reset stop flag
#ifdef WIN32
    QueryPerformanceCounter(&startCount);
#else
    gettimeofday(&startCount, NULL);
#endif
  }

  ///////////////////////////////////////////////////////////////////////////////
  // stop the timer.
  // endCount will be set at this point.
  ///////////////////////////////////////////////////////////////////////////////
  void stop()
  {
    stopped = 1; // set timer stopped flag

#ifdef WIN32
    QueryPerformanceCounter(&endCount);
#else
    gettimeofday(&endCount, NULL);
#endif
  }

  ///////////////////////////////////////////////////////////////////////////////
  // same as getElapsedTimeInSec()
  ///////////////////////////////////////////////////////////////////////////////
  double getElapsedTime()
  {
    return this->getElapsedTimeInSec();
  }

  ///////////////////////////////////////////////////////////////////////////////
  // divide elapsedTimeInMicroSec by 1000000
  ///////////////////////////////////////////////////////////////////////////////
  double getElapsedTimeInSec()
  {
    return this->getElapsedTimeInMicroSec() * 0.000001;
  }

  ///////////////////////////////////////////////////////////////////////////////
  // divide elapsedTimeInMicroSec by 1000
  ///////////////////////////////////////////////////////////////////////////////
  double getElapsedTimeInMilliSec()
  {
    return this->getElapsedTimeInMicroSec() * 0.001;
  }

  ///////////////////////////////////////////////////////////////////////////////
  // compute elapsed time in micro-second resolution.
  // other getElapsedTime will call this first, then convert to correspond resolution.
  ///////////////////////////////////////////////////////////////////////////////
  double getElapsedTimeInMicroSec()
  {
#ifdef WIN32
    if(!stopped)
    QueryPerformanceCounter(&endCount);

    startTimeInMicroSec = startCount.QuadPart * (1000000.0 / frequency.QuadPart);
    endTimeInMicroSec = endCount.QuadPart * (1000000.0 / frequency.QuadPart);
#else
    if (!stopped)
      gettimeofday(&endCount, NULL);

    startTimeInMicroSec = (startCount.tv_sec * 1000000.0) + startCount.tv_usec;
    endTimeInMicroSec = (endCount.tv_sec * 1000000.0) + endCount.tv_usec;
#endif

    return endTimeInMicroSec - startTimeInMicroSec;
  }

protected:

private:
  double startTimeInMicroSec; // starting time in micro-second
  double endTimeInMicroSec; // ending time in micro-second
  int stopped; // stop flag
#ifdef WIN32
  LARGE_INTEGER frequency; // ticks per second
  LARGE_INTEGER startCount;//
  LARGE_INTEGER endCount;//
#else
  timeval startCount; //
  timeval endCount; //
#endif
};

#endif // TIMER_H
