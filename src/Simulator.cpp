#include <algorithm>
#include <iostream>
#include "Simulator.h"
#include "Circuit.h"
#include "MNAStamper.h"
#include "StepControl.h"
#include "Debug.h"

namespace NA {

IntegrateMethod
Simulator::integrateMethod() const
{
  IntegrateMethod method = IntegrateMethod::None;
  if (intMethod() == IntegrateMethod::BackwardEuler) {
    method = IntegrateMethod::BackwardEuler;
  } else if (intMethod() == IntegrateMethod::Gear2) {
    if (_result.ticks().size() < 2) {
      method = IntegrateMethod::BackwardEuler;
    } else {
      method= IntegrateMethod::Gear2;
    }
  } else if (intMethod() == IntegrateMethod::Trapezoidal) {
    if (_result.ticks().size() < 2) {
      method = IntegrateMethod::BackwardEuler;
    } else {
      method= IntegrateMethod::Trapezoidal;
    }
/*} else if (intMethod == IntegrateMethod::RK4) {
    if (prevData.ticks().size() < 1) {
      return IntegrateMethod::BackwardEuler;
    } else if (prevData.ticks().size() < 3) {
      return IntegrateMethod::Gear2;
    } else {
      return IntegrateMethod::RK4;
    } */
  }
  if (method == IntegrateMethod::None) {
    method = IntegrateMethod::Gear2;
  }
  return method;
}
    
Simulator::Simulator(const Circuit& ckt, const AnalysisParameter& param)
: _circuit(ckt), _param(param), _result(&ckt, _param._name)
{}

void 
Simulator::updateEquation()
{
  //if (_needUpdateA) {
  //  MNAStamper::updateA(_Alu, this);
  //}
  if (_needRebuild) {
    formulateEquation();
  } else {
    MNAStamper stamper(_param, _circuit, _result);
    stamper.updateb(_b, integrateMethod());
    if (Debug::enabled(DebugModule::Sim)) {
      double prevTime = _result.ticks().back();
      Debug::printVector(prevTime+simulationTick(), "b", _b);
    }
  }
}

void 
Simulator::formulateEquation()
{
  Eigen::MatrixXd G;
  G.setZero(_eqnDim, _eqnDim);
  Eigen::MatrixXd C;
  C.setZero(_eqnDim, _eqnDim);
  _b.setZero(_eqnDim);
  MNAStamper stamper(_param, _circuit, _result);
  stamper.stamp(G, C, _b, integrateMethod());

  Eigen::MatrixXd A = G + C;
  
  if (Debug::enabled(DebugModule::Sim)) {
    Debug::printEquation(A, _b);
    /*Eigen::EigenSolver<Eigen::MatrixXd> es(A);
    printf("Eigenvalues of A: \n");
    for (int i=0; i<A.rows(); ++i) {
      std::complex<double> value = es.eigenvalues().col(0)[i];
      printf("  %f+%fi\n", std::real(value), std::imag(value));
    }
    */
  }
  _Alu = A.fullPivLu();
}

void 
Simulator::initData()
{
  _eqnDim = _result.indexMap().size();
}

void 
Simulator::solveEquation()
{
  Eigen::VectorXd x(_eqnDim);
  x = _Alu.solve(_b);
  
  std::vector<double>& ticks = _result.ticks();
  std::deque<double>& values = _result.values();
  double prevTime = 0;
  if (ticks.empty() == false) {
    prevTime = ticks.back();
  }
  ticks.push_back(prevTime + simulationTick());
  values.insert(values.end(), x.begin(), x.end());
  if (Debug::enabled(DebugModule::Sim)) {
    Debug::printSolution(prevTime+simulationTick(), "x", x, _result.indexMap(), _circuit);
  }
}

bool
Simulator::converged() const
{
  if (checkTerminateCondition()) {
    return true;
  }
  double simTime = 0;
  if (_result.ticks().empty() == false) {
    simTime = _result.ticks().back();
  }
  bool converge = true;
  return simTime > simEnd() && converge;
}

void
Simulator::adjustSimTick()
{
  /*
  double stepSizeLimit = StepControl::stepLimit(this, relTotal());
  if (simulationTick() > stepSizeLimit) {
    printf("WARNING: Current step size %G is larger the LTE limit %G\n", simulationTick(), stepSizeLimit);
  }
  */
  /// Implement LTE calculation and adjust simulationTick() based on it
  /// and set _needUpdateA to true
  return;
}

void 
Simulator::run()
{
  initData();
  if (_updateFunc) {
    _needRebuild = _updateFunc();
  }
  formulateEquation();
  solveEquation();
  while (!converged()) {
    checkNeedRebuild();
    if (_updateFunc) {
      _needRebuild = _updateFunc();
    }
    adjustSimTick();
    updateEquation();
    solveEquation();
  }
}

void
Simulator::checkNeedRebuild() 
{
  _needRebuild = false;
  if (_prevMethod != integrateMethod()) {
    _prevMethod = integrateMethod();
    _needRebuild = true;
  }
}

static bool
checkTermCondition(size_t id, bool isNodeId, bool isRise, double value, const SimResult& result)
{
  double val1, val2;
  if (isNodeId) {
    val1 = result.nodeVoltageBackstep(id, 1);
    val2 = result.nodeVoltageBackstep(id, 2);
  } else {
    val1 = result.deviceCurrentBackstep(id, 1);
    val2 = result.deviceCurrentBackstep(id, 2);
  }
  if (isRise && ((val1 <= value && val2 >= value) || (val1 >= value && val2 >= value))) {
    return true;
  }
  if (isRise == false && ((val1 >= value && val2 <= value) || (val1 <= value && val2 <= value))) {
    return true;
  }
  return false;
}

bool
Simulator::checkTerminateCondition() const
{
  if (_termVoltages.empty() && _termCurrents.empty()) {
    return false;
  }
  for (const auto& kv : _termVoltages) {
    const TermVoltage& v = kv.second;
    if (checkTermCondition(kv.first, true, v.first, v.second, _result) == false) {
      return false;
    }
  }
  /*
  for (const auto& kv : _termCurrents) {
    if (checkTermCondition(kv.first, false, kv.second, _result) == false) {
      return false;
    }
  }
  */
  return true;
}


}
