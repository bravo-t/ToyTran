#ifndef _TRAN_SIM_H_
#define _TRAN_SIM_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include "Base.h"
#include "SimResult.h"

namespace NA {

class Circuit;

class Simulator {
  public:
    Simulator(const Circuit& ckt, const AnalysisParameter& param);
    void initData();

    bool needRebuildEquation() const { return _needRebuild; }

    /// Normally initial conditions are computed by a DC OP simulation. 
    /// Here we use 0v for now
    double initialCondition(size_t /*nodeId*/) const { return 0; }
    const SimResult& simulationResult() const { return _result; }
    /// Choose integration method, and update _prevMethod;
    IntegrateMethod integrateMethod() const;
    const Circuit& circuit() const { return _circuit; }

    void run();

    double simulationTick() const { return _param._simTick; }
    double simEnd() const { return _param._simTime; }
    double relTotal() const { return _param._relTotal; }
    IntegrateMethod intMethod() const { return _param._intMethod; }

    void setTerminationNodeVoltage(size_t nodeId, double value) 
    {
      _termNodeId = nodeId;
      _termValue = value;
    }
    void setTerminationDeviceCurrent(size_t devId, double value) 
    {
      _termDeviceId = devId;
      _termValue = value;
    }

  private:
    void formulateEquation();
    void updateEquation();
    bool converged() const;
    void adjustSimTick();
    void solveEquation();
    void checkNeedRebuild();
    bool checkTerminateCondition();

  private:
    size_t             _eqnDim = 0;
    bool               _needIterate = true;
    bool               _needRebuild = false;
    bool               _needUpdateA = false;
    const Circuit&     _circuit;
    AnalysisParameter  _param;
    IntegrateMethod    _prevMethod = IntegrateMethod::None;
    SimResult          _result;
    Eigen::VectorXd    _b;
    /// Cache data
    Eigen::FullPivLU<Eigen::MatrixXd> _Alu;

    size_t             _termNodeId = static_cast<size_t>(-1);
    size_t             _termDeviceId = static_cast<size_t>(-1);
    double             _termValue = 0;
};

}

#endif
