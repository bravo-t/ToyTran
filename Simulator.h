#ifndef _TRAN_SIM_H_
#define _TRAN_SIM_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include "Base.h"
#include "SimResult.h"

namespace Tran {

class Circuit;

class Simulator {
  public:
    Simulator(const Circuit& ckt)
    : _circuit(ckt) {}
    void initData();
    void setSimTick(double simTick);
    void setSimulationEndTime(double t);
    void setIntegrateMethod(IntegrateMethod intMethod);

    bool needRebuildEquation() const { return _needRebuild; }

    /// Normally initial conditions are computed by a DC OP simulation. 
    /// Here we use 0v for now
    double initialCondition(size_t /*nodeId*/) const { return 0; }
    const SimResult& simulationResult() const { return _result; }
    double simulationTick() const { return _simTick; }
    /// Choose integration method, and update _prevMethod;
    IntegrateMethod integrateMethod() const;
    const Circuit& circuit() const { return _circuit; }

    double nodeVoltage(size_t nodeId, size_t steps) const;
    double deviceCurrent(size_t devId, size_t steps) const;
    
    /// @brief Get derivative of voltage and current.
    ///        order controls the order of derivative you need
    ///        order should be postive, 2 mean 2nd derivative for example
    double nodeVoltageDerivative(size_t posNodeId, size_t negNodeId, 
                                 size_t order, size_t steps) const;
    double deviceCurrentDerivative(size_t devId, size_t order, size_t steps) const;

    void run();

  private:
    void formulateEquation();
    void updateEquation();
    bool converged() const;
    void adjustSimTick();
    void solveEquation();
    void checkNeedRebuild();

  private:
    double           _simTick = 1e-15;
    double           _simEnd;
    size_t           _eqnDim = 0;
    bool             _needIterate = true;
    bool             _needRebuild = false;
    bool             _needUpdateA = false;
    const Circuit&   _circuit;
    IntegrateMethod  _prevMethod = IntegrateMethod::None;
    IntegrateMethod  _intMethod = IntegrateMethod::Gear2;
    SimResult        _result;
    Eigen::VectorXd  _b;
    /// Cache data
    Eigen::FullPivLU<Eigen::MatrixXd> _Alu;
};

}

#endif
