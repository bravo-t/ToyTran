#ifndef _TRAN_SIM_H_
#define _TRAN_SIM_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include "Base.h"

namespace Tran {

class Circuit;

enum class SimResultType : unsigned char {
  Voltage,
  Current,
};

/// @brief The map between numbers in x of Ax=b, and the actual meaning of 
///        the number
///        size() should be the dimention of vector x
struct SimResultMap {
  std::unordered_map<size_t, size_t> _nodeVoltageMap; /// node ID to matrix index
  std::unordered_map<size_t, size_t> _deviceCurrentMap; /// device ID to matrix index

  size_t size() const { return _nodeVoltageMap.size() + _deviceCurrentMap.size(); }
};

/// @brief The solution data of every time step produced by solving Ax=b
struct SimResult {
  SimResultMap        _map;
  std::vector<double> _ticks;
  std::deque<double>  _values; /// size should be _map.size()*_ticks.size()

  /// @brief Get the voltage of given node id
  /// @param nodeId 
  /// @param steps: number of steps BACK with respect to current time
  ///               1 means 1 simTick previous to current time
  ///               if requested steps are not available, initial condition 
  ///               is returned
  double nodeVoltage(size_t nodeId, size_t steps) const;
  /// @brief Get the current of given device id, parameters are interpreted 
  ///        the same way as nodeVoltage
  double deviceCurrent(size_t devId, size_t steps) const;
  /// @brief Get the index of the result vector x, which is also the row and column
  ///        Index of matrix A, from given node/device id
  size_t nodeVectorIndex(size_t nodeId) const;
  size_t deviceVectorIndex(size_t deviveId) const;
  /// @brief Get accumulated simulation time
  double currentTime() const;
};

class Simulator {
  public:
    Simulator(const Circuit& ckt)
    : _circuit(ckt) {}
    void setSimTick(double simTick);
    void setSimulationEndTime(double t);
    void setIntegrateMethod(IntegrateMethod intMethod);

    /// Normally initial conditions are computed by a DC OP simulation. 
    /// Here we use 0v for now
    double initialCondition(size_t /*nodeId*/) const { return 0; }
    const SimResult& simulationResult() const { return _result; }
    double simulationTick() const { return _simTick; }
    IntegrateMethod integrateMethod() const;
    const Circuit& circuit() const { return _circuit; }

    double nodeVoltage(size_t nodeId, size_t steps) const;
    double deviceCurrent(size_t devId, size_t steps) const;

    void run();

  private:
    void formulateEquation();
    void updateEquation();
    bool converged() const;
    void adjustSimTick();
    void solveEquation();

  private:
    double           _simTick = 1e-15;
    double           _simEnd;
    size_t           _eqnDim = 0;
    bool             _needIterate = true;
    bool             _needUpdateA = false;
    const Circuit&   _circuit;
    IntegrateMethod  _intMethod = IntegrateMethod::Gear2;
    SimResult        _result;
    Eigen::VectorXd  _b;
    /// Cache data
    Eigen::FullPivLU<Eigen::MatrixXd> _Alu;
};

}

#endif
