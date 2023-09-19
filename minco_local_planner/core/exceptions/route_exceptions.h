/**
 * @Author: Yunkai Xia
 * @Date:   2023-09-05 14:12:49
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2023-09-05 14:27:59
 */
#include <stdint.h>

#ifndef __ROUTE_EXCEPTIONS_H__
#define __ROUTE_EXCEPTIONS_H__

#include <memory>
#include <stdexcept>
#include <string>

namespace minco_local_planner::exceptions {

class RouteException : public std::runtime_error {
 public:
  explicit RouteException(const std::string& description)
      : std::runtime_error(description) {}
};

class NoValidGraph : public RouteException {
 public:
  explicit NoValidGraph(const std::string& description)
      : RouteException(description) {}
};

class NoValidRouteCouldBeFound : public RouteException {
 public:
  explicit NoValidRouteCouldBeFound(const std::string& description)
      : RouteException(description) {}
};

class TimedOut : public RouteException {
 public:
  explicit TimedOut(const std::string& description)
      : RouteException(description) {}
};
}  // namespace minco_local_planner::exceptions

#endif /* __ROUTE_EXCEPTIONS_H__ */
