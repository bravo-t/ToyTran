#include <cstdio>
#include <algorithm>
#include <Eigen/Eigenvalues>
#include "PoleZero.h"
#include "MNAStamper.h"
#include "Debug.h"
#include "rpoly.h"

#include <iostream>

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
PoleZeroAnalysis::check()
{
  if (_inNode._nodeId == static_cast<size_t>(-1)) {
    printf("ERROR: Input node specified as \"%s\" does not exist\n", _param._inNode->data());
    return false;
  }
  if (_outNode._nodeId == static_cast<size_t>(-1)) {
    printf("ERROR: Output node specified as \"%s\" does not exist\n", _param._outNode->data());
    return false;
  }
  if (_param._order > _circuit.order()) {
    printf("WARNING: User specified order %u is larger than circuit order %lu, circuit order is used\n", 
           _param._order, _circuit.order());
    _param._order = _circuit.order();
  }
  if (_circuit.scalingFactor() != 1) {
    printf("Moment scaling factor of 1e%lu will be used to improve numerical stability\n", _circuit.scalingFactor());
  }
  return true;
}

bool
PoleZeroAnalysis::calcMoments(const MatrixXd& G, 
                              const MatrixXd& C, 
                              const VectorXd& E,
                              std::vector<double>& inputMoments,
                              std::vector<double>& outputMoments) const
{
  /// Number of moments should be twice the number of poles to be calculated
  size_t order = _param._order * 2 + 1;
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
  if (Debug::enabled()) {
    Debug::printEquation(G, E);
    Debug::printSolution(0, "V0", Vprev, _result.indexMap(), _circuit);
    Debug::printEquation(C, E);
  }
  for (size_t i=1; i<order; ++i) {
    Eigen::VectorXd V(_eqnDim);
    Eigen::VectorXd RHS = -C * Vprev;
    V = GLU.solve(RHS);
    inputMoments.push_back(V(inputIndex));
    outputMoments.push_back(V(outputIndex));
    if (Debug::enabled()) {
      char buf[50];
      sprintf(buf, "V%lu", i);
      Debug::printEquation(G, RHS);
      Debug::printSolution(0, buf, V, _result.indexMap(), _circuit);
    }
    Vprev = V;
  }

  return true;
}
    
bool 
PoleZeroAnalysis::calcTFDenominatorCoeff(const std::vector<double>& moments, 
                                         std::vector<double>& coeff) const
{
  size_t order = _param._order;
  Eigen::MatrixXd M(order, order);
  Eigen::VectorXd V(order);
  Eigen::MatrixXi DEBUG(order, order);
  Eigen::VectorXi DEBUGV(order);
  for (size_t i=0; i<order; ++i) {
    for (size_t j=0; j<order; ++j) {
      size_t index = i + j;
      M(i,j) = moments[index];
      DEBUG(i, j) = index;
    }
    V(i) = -moments[i+order];
    DEBUGV(i) = -(i+order);
  }
  Eigen::FullPivLU<Eigen::MatrixXd> LU = M.fullPivLu();
  Eigen::VectorXd B = LU.solve(V);
  if (Debug::enabled()) {
    Debug::printEquation(M, V);
    Debug::printSolution("B", B);
    std::cout << DEBUG << std::endl;
    std::cout << DEBUGV << std::endl;
  }
  coeff.clear();
  coeff.reserve(order);
  /// B is in 
  for (size_t i=0; i<order; ++i) {
    coeff.push_back(B(i));
  }
  coeff.push_back(1.0);
  if (Debug::enabled()) {
    printf("Denominator coeffcients in decreasing order:\n");
    for (double c : coeff) printf("%.6G ", c);
    printf("\n");
  }
  return true;
}

bool 
PoleZeroAnalysis::calcTFNumeratorCoeff(const std::vector<double>& moments, 
                                       const std::vector<double>& denomCoeff,
                                       std::vector<double>& coeff) const
{
  size_t order = _param._order;
  coeff.clear();
  coeff.assign(order, 0);
  for (size_t i=0; i<order; ++i) {
    double a = moments[i];
    //printf("a_%lu = m_%lu", i, i);
    size_t denomIndex = order - 1;
    for (int j=i-1; j>=0; --j) {
      a += (moments[j]*denomCoeff[denomIndex]);
      //printf(" + m_%d * b_%lu", j, denomIndex);
      --denomIndex;
    }
    coeff[order-1-i] = a;
    //printf("\n");
  }
  if (Debug::enabled()) {
    printf("Numerator coeffcients in decreasing order:\n");
    for (double c : coeff) printf("%.6f ", c);
    printf("\n");
  }
  return true;
}

typedef PoleZeroAnalysis::Complex Complex;
typedef std::function<Complex(Complex)> ComplexFunc;

static bool
calcPolynomialRoots(const std::vector<double>& coeff, 
                    std::vector<Complex>& roots)
{
  double rootsr[100];
  double rootsi[100];
  RPoly<double> rpoly;
  int status = rpoly.findRoots(coeff.data(), coeff.size()-1, rootsr, rootsi);
  assert(status != -1 && "rpoly failed");
  for (int i=0; i<status; ++i) {
    roots.push_back({rootsr[i], rootsi[i]});
  }
  return true;
}

bool
PoleZeroAnalysis::calcPoles(const std::vector<double>& denomCoeff, 
                            std::vector<Complex>& poles) const
{
  return calcPolynomialRoots(denomCoeff, poles);  
}

bool
PoleZeroAnalysis::calcZeros(const std::vector<double>& numCoeff, 
                            std::vector<Complex>& zeros) const
{
  return calcPolynomialRoots(numCoeff, zeros);  
}

Complex
minusPower(const Complex& n, int p)
{
  Complex result;
  if (n.imag() == 0) {
    result.real(std::pow(n.real(), p));
    result.imag(0);
  } else {
    if (p < 0) {
      Complex r(1, 0);
      result = r / std::pow(n, -p);
    } else {
      result = std::pow(n, p);
    }
  }
  return result;
}

bool 
PoleZeroAnalysis::calcResidues(const std::vector<Complex>& poles, 
                               const std::vector<double>& moments, 
                               double k,
                               std::vector<Complex>& residues) const
{
  size_t dim = poles.size();
  Eigen::MatrixXcd P(dim, dim);
  Eigen::VectorXcd M(dim);
  std::complex<double> r(1, 0);
  for (size_t i=0; i<dim; ++i) {
    for (size_t j=0; j<dim; ++j) {
      //P(i, j) = minusPower(poles[j], -i);
      P(i, j) = minusPower(poles[j], -(i+1));
      //printf("DEBUG: P(%lu, %lu) = p_%lu ^ %ld\n", i, j, j, -(i+1));
    }
    M(i) = moments[i];
    /*
    if (i == 0) {
      /// moments[-1] is defined as the sum of all outputs of t=0+, 
      /// which will be 0 in our case
      M(i) = 0;
    } else {
      M(i) = -moments[i-1];
    }
    */
  }
  Eigen::FullPivLU<Eigen::MatrixXcd> LU = P.fullPivLu();
  Eigen::VectorXcd R = LU.solve(M);
  if (Debug::enabled()) {
    Debug::printEquation(P, M);
    Debug::printSolution("R", R);
  }
  residues.clear();
  residues.reserve(dim);
  for (size_t i=0; i<dim; ++i) {
    residues.push_back(R(i)*k);
  }
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
  calcMoments(G, C, E, inputMoments, outputMoments);
  std::vector<double> denomCoeff;
  calcTFDenominatorCoeff(outputMoments, denomCoeff);
  std::vector<double> numCoeff;
  calcTFNumeratorCoeff(outputMoments, denomCoeff, numCoeff);
  calcPoles(denomCoeff, _poles);
  calcZeros(numCoeff, _zeros);
  calcResidues(_poles, outputMoments, 1.0 / denomCoeff[0], _residues);
  _moments.assign(outputMoments.begin(), outputMoments.end());
  printf("Moments:\n");
  for (double m : _moments) printf("%.6G ", m);
  printf("\n");
  
  printf("Poles:\n");
  for (const Complex& c : _poles) printf("%.6f+%.6fi ", c.real(), c.imag());
  printf("\n");
  
  printf("Zeros:\n");
  for (const Complex& c : _zeros) printf("%.6f+%.6fi ", c.real(), c.imag());
  printf("\n");
  
  printf("Residues:\n");
  for (const Complex& c : _residues) printf("%.6f+%.6fi ", c.real(), c.imag());
  printf("\n");

  /* a small unit test
  // Below moment values in debugM will give a result of two poles, 1 and 2, 
  // and corresponding residuals 1 and 2
  std::vector<double> debugM = {1, 0.75, 0.625, 0.5625};
  std::vector<double> debugPCoeff;
  calcTFDenominatorCoeff(debugM, debugPCoeff);
  std::vector<Complex> debugP;
  calcPolynomialRoots(debugPCoeff, debugP);
  for (Complex r : debugP) printf("P: %f + %f i\n", r.real(), r.imag());
  std::vector<double> debugQCoeff;
  calcTFNumeratorCoeff(debugM, debugPCoeff, debugQCoeff);
  std::vector<Complex> debugZ;
  calcZeros(debugQCoeff, debugZ);
  for (Complex r : debugZ) printf("Z: %f + %f i\n", r.real(), r.imag());
  std::vector<Complex> debugR;
  calcResidues(debugP, debugM, 1.0 / debugPCoeff[0], debugR);
  for (Complex r : debugR) printf("R: %f + %f i\n", r.real(), r.imag());
  */
}




}
