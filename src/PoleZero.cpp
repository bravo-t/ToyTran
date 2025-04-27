#include <cstdio>
#include <algorithm>
#include <Eigen/Eigenvalues>
#include "PoleZero.h"
#include "MNAStamper.h"
#include "Debug.h"
#include "rpoly.h"

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
PoleZeroAnalysis::calcMoments(const MatrixXd& G, 
                              const MatrixXd& C, 
                              const VectorXd& E,
                              std::vector<double>& inputMoments,
                              std::vector<double>& outputMoments) const
{
  /// Number of moments should be twice the number of poles to be calculated
  size_t order = _param._order * 2;
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
  for (size_t i=0; i<order; ++i) {
    for (size_t j=0; j<order; ++j) {
      size_t index = i + j;
      M(i,j) = moments[index];
    }
    V(i) = moments[i+order];
  }
  Eigen::FullPivLU<Eigen::MatrixXd> LU = M.fullPivLu();
  Eigen::VectorXd B = LU.solve(V);
  if (Debug::enabled()) {
    Debug::printEquation(M, V);
    Debug::printSolution(0, "B", B, _result.indexMap(), _circuit);
  }
  coeff.clear();
  coeff.reserve(order);
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
  coeff.reserve(order);
  for (size_t i=0; i<order; ++i) {
    double b = moments[i];
    size_t denomIndex = 0;
    for (int j=i-1; j>0; --j) {
      b += (moments[j]*denomCoeff[denomIndex]);
      ++denomIndex;
    }
    coeff.push_back(b);
  }
  if (Debug::enabled()) {
    printf("Numerator coeffcients:\n");
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
  int status = rpoly(coeff.data(), coeff.size(), rootsr, rootsi);
  assert(status != -1 && "rpoly failed");
  for (int i=0; i<status; ++i) {
    roots.push_back({rootsr[i], rootsi[i]});
  }
  /*
  /// First let's turn the polynomial to monic polynomial form:
  /// c[0] + c[1]*x + ... + c[n-1]*x^(n-1) + x^n
  double leadCoeff = coeff.back();
  std::vector<double> monicCoeff;
  monicCoeff.reserve(coeff.size());
  monicCoeff.push_back(1/leadCoeff);
  for (size_t i=0; i<coeff.size()-1; ++i) {
    monicCoeff.push_back(coeff[i]/leadCoeff);
  }
  /// Then build companion matrix for this monic polynomial
  size_t dim = monicCoeff.size();
  Eigen::MatrixXd C;
  C.setZero(dim, dim);
  C(0, dim-1) = -monicCoeff[0];
  for (size_t i=1; i<dim; ++i) {
    C(i, i-1) = 1;
    C(i, dim-1) = -monicCoeff[i]; 
  }
  /// Lastly calculate the eigenvalues of the companion matrix
  Eigen::EigenSolver<MatrixXd> es(C);
  const Eigen::VectorXcd& ev = es.eigenvalues();
  roots.clear();
  roots.reserve(dim);
  for (size_t i=0; i<dim; ++i) {
    roots.push_back(ev(i));
  }
  */
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

bool 
PoleZeroAnalysis::calcResidues(const std::vector<Complex>& poles, 
                               const std::vector<double>& moments, 
                               std::vector<Complex>& residues) const
{
  size_t dim = poles.size();
  Eigen::MatrixXcd P(dim, dim);
  Eigen::VectorXcd M(dim);
  for (size_t i=0; i<dim; ++i) {
    for (size_t j=0; j<dim; ++j) {
      P(i, j) = std::pow(poles[i], -(j+1));
    }
    M(i) = -moments[i];
  }
  Eigen::FullPivLU<Eigen::MatrixXcd> LU = P.fullPivLu();
  Eigen::VectorXcd R = LU.solve(M);
  residues.clear();
  residues.reserve(dim);
  for (size_t i=0; i<dim; ++i) {
    residues.push_back(R(i));
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
  calcResidues(_poles, outputMoments, _residues);
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

  std::vector<Complex> test;
  calcPolynomialRoots({1, -2, 1}, test);
  for (Complex t : test) printf("DEBUG: %f + %fi\n", t.real(), t.imag());
}




}
