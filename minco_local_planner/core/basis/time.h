/**
 * @Author: Yunkai Xia
 * @Date:   2023-08-24 13:19:51
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-08-24 14:16:58
 */
#include <stdint.h>

#ifndef __TIME_H__
#define __TIME_H__
#include <sys/time.h>
#include <time.h>

namespace minco_local_planner::basis {


static inline double GetTimeNowDouble() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double)tv.tv_sec + 1e-6 * tv.tv_usec;
}

}  // namespace minco_local_planner::basis

#endif /* __TIME_H__ */
