#ifndef _TRAN_SIMRES_H_
#define _TRAN_SIMRES_H_

#include <vector>
#include <deque>
#include <limits>
#include "Base.h"

namespace Tran {

/// @brief The map between numbers in x of Ax=b, and the actual meaning of 
///        the number
///        size() should be the dimention of vector x
struct SimResultMap {
  static size_t invalidValue() { return static_cast<size_t>(-2); }
  size_t _dimension = 0;
  std::vector<size_t> _nodeVoltageMap; /// node ID to matrix index
  std::vector<size_t> _deviceCurrentMap; /// device ID to matrix index

  size_t size() const { return _dimension; }
  void setDimention(size_t val) { _dimension = val; }
};

/// @brief The solution data of every time step produced by solving Ax=b
struct SimResult {
  SimResultMap        _map;
  std::vector<double> _ticks;
  std::deque<double>  _values; /// size should be _map.size()*_ticks.size()

  /// @brief Get voltage or current of given node id or device id
  ///        This function returns the data in a forward manner
  ///        Means a timeStep of 0 gives the voltage/current @ 0 tick
  double nodeVoltage(size_t nodeId, size_t timeStep) const;
  double deviceCurrent(size_t devId, size_t timeStep) const;

  /// @brief Get the voltage of given node id
  /// @param nodeId 
  /// @param steps: number of steps BACK with respect to current time
  ///               0 means 1 simTick previous to current time
  ///               if requested steps are not available, initial condition 
  ///               is returned
  double nodeVoltageBackstep(size_t nodeId, size_t steps) const;
  /// @brief Get the current of given device id, parameters are interpreted 
  ///        the same way as nodeVoltageBackstep
  double deviceCurrentBackstep(size_t devId, size_t steps) const;
  /// @brief Get the index of the result vector x, which is also the row and column
  ///        Index of matrix A, from given node/device id
  size_t nodeVectorIndex(size_t nodeId) const;
  size_t deviceVectorIndex(size_t deviveId) const;
  /// @brief Get accumulated simulation time
  double currentTime() const;
  /// @brief Get size of the simulation result
  size_t size() const { return _ticks.size(); }
  /// @brief Get simulation time step size of previous n steps
  ///        If steps is -1, it means the step size of previous 
  ///        step to current step is returned
  double stepSize(size_t steps) const;
};
}

#endif