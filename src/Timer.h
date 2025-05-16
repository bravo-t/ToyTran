#ifndef _NA_TIMER_H_
#define _NA_TIMER_H_

#include <ctime>

namespace NA {

inline uint64_t
timeDiffNs(const timespec& endTime, const timespec& startTime)
{
  return (endTime.tv_sec - startTime.tv_sec) * 1e9 + 
         (endTime.tv_nsec - startTime.tv_nsec);
}



}

#endif