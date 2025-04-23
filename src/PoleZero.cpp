#include <cstdio>
#include "PoleZero.h"
#include "Eigen/src/LU/FullPivLU.h"
#include "MNAStamper.h"

namespace NA {

using namespace Eigen;

PoleZeroAnalysis::PoleZeroAnalysis(const Circuit& circuit, const AnalysisParameter& param)
: _circuit(circuit), _param(param), _result(circuit)
{
  _inNode = _circuit.findNodeByName(*(_param._inNode));
  _outNode = _circuit.findNodeByName(*(_param._outNode));
  _eqnDim = _result.indexMap().size();
}

bool 
PoleZeroAnalysis::check() const
{
  if (_inNode._nodeId == static_cast<size_t>(-1)) {
    printf("ERROR: Input node specified as \"%s\" does not exist\n", _param._inNode->data());
    return false;
  }
  if (_outNode._nodeId == static_cast<size_t>(-1)) {
    printf("ERROR: Output node specified as \"%s\" does not exist\n", _param._outNode->data());
    return false;
  }
  return true;
}

bool
PoleZeroAnalysis::momentMatching(const MatrixXd& G, 
                                 const MatrixXd& C, 
                                 const VectorXd& E,
                                 std::vector<double>& inputMoments,
                                 std::vector<double>& outputMoments) const
{
  size_t order = _param._order;
  inputMoments.clear();
  inputMoments.reserve(order+1);
  outputMoments.clear();
  outputMoments.reserve(order+1);
  size_t inputIndex = _result.nodeVectorIndex(_inNode._nodeId);
  size_t outputIndex = _result.nodeVectorIndex(_outNode._nodeId);
  Eigen::FullPivLU<Eigen::MatrixXd> GLU = G.fullPivLu();
  Eigen::VectorXd Vprev(_eqnDim);
  Vprev = GLU.solve(E);
  inputMoments.push_back(Vprev(inputIndex));
  outputMoments.push_back(Vprev(outputIndex));
  for (size_t i=0; i<order; ++i) {
    Eigen::VectorXd V(_eqnDim);
    V = GLU.solve(-C*Vprev);
    inputMoments.push_back(V(inputIndex));
    outputMoments.push_back(V(outputIndex));
    Vprev = V;
  }
  std::reverse(inputMoments.begin(), inputMoments.end());
  std::reverse(outputMoments.begin(), outputMoments.end());
  return true;
}


void
PoleZeroAnalysis::run()
{
  if (check() == false) {
    return;
  }
  MNAStamper stamper(_param, _circuit, _result);
  Eigen::MatrixXd G;
  G.setZero(_eqnDim, _eqnDim);
  Eigen::MatrixXd C;
  C.setZero(_eqnDim, _eqnDim);
  Eigen::VectorXd E;
  E.setZero(_eqnDim);
  stamper.stamp(G, C, E);
  std::vector<double> inputMoments;
  std::vector<double> outputMoments;
  momentMatching(G, C, E, inputMoments, outputMoments);
}




}