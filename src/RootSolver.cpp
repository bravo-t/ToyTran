#include "RootSolver.h"

namespace NA {

typedef RootSolver::Function Function;

void
RootSolver::addFunction(const Function& func) 
{
  _functions.push_back(func);
}

void
RootSolver::addDerivativeFunction(const Function& devFunc)
{
  _derivatives.push_back(devFunc);
}

void
RootSolver::setInitX(const std::vector<double>& x) 
{
  if (_x.rows() == 0) {
    _x.resize(x.size());
  }
  for (size_t i=0; i<x.size(); ++i) {
    _x(i) = x[i];
  }
}

std::vector<double> 
RootSolver::solution() const
{
  std::vector<double> sol;
  for (Eigen::Index i=0; i<_x.rows(); ++i) {
    sol.push_back(_x(i));
  }
  return sol;
}

typedef std::vector<Function> Functions;

static double
calcNumericalDerivative(const Function& func, 
                        const Eigen::VectorXd& x, 
                        size_t varIndex)
{
  double h = 1e-6;
  Eigen::VectorXd xCP = x;
  xCP(varIndex) += h;
  double result = (func(xCP) - func(x)) / h;
  return result;
}

static double 
calcDerivative(const Functions& devFuncs, 
               size_t funcIndex, size_t varIndex, 
               const Eigen::VectorXd& x)
{
  size_t index = funcIndex * x.rows() + varIndex;
  const Function& devFunc = devFuncs[index];
  return devFunc(x);
}

Eigen::MatrixXd
Jacobian(const Functions& funcs, 
         const Functions& derivatives, 
         const Eigen::VectorXd& x)
{
  Eigen::MatrixXd Jac(funcs.size(), funcs.size());
  Jac.setZero();
  bool useNumericalDerivative = false;
  if (derivatives.empty()) {
    useNumericalDerivative = true;
  }
  for (size_t i=0; i<funcs.size(); ++i) {
    const Function& func = funcs[i];
    for (size_t j=0; j<funcs.size(); ++j) {
      if (useNumericalDerivative) {
        Jac(i, j) = calcNumericalDerivative(func, x, j);
      } else {
        Jac(i, j) = calcDerivative(derivatives, i, j, x);
      }
    }
  }
  return Jac;
}

bool
RootSolver::check() const
{
  if (_derivatives.empty() == false) {
    if (_derivatives.size() != _functions.size() * _functions.size()) {
      printf("ERROR: Incorrect number of derivative functions, functions have %lu, derivatives have %lu\n", 
        _functions.size(), _derivatives.size());
      return false;
    }
  }
  if (_functions.size() != (size_t) _x.rows()) {
    printf("ERROR: Incorrect number of functions and variables, functions have %lu, variables have %lu\n", 
      _functions.size(), _x.rows());
    return false;
  }
  return true;
}

bool
RootSolver::run() 
{
  if (check() == false) {
    return false;
  }
  size_t iterCount = 0;
  bool converged = false;
  while (!converged) {
    Eigen::Matrix Jac = Jacobian(_functions, _derivatives, _x);
    Eigen::VectorXd f(_x.rows());
    for (Eigen::Index i=0; i<_x.rows(); ++i) {
      f(i) = _functions[i](_x);
    }
    Eigen::VectorXd d = Jac.partialPivLu().solve(f);
    _x -= d;
    converged = true;
    for (Eigen::Index i=0; i<_x.rows(); ++i) {
      if (std::abs(d(i)) > std::abs(_x(i)) * _xTol) {
        converged = false;
        break;
      }
    }
    ++iterCount;
    if (iterCount > _maxIter) {
      return false;
    }
  }
  return true;
}

}

typedef ::NA::RootSolver::Function Function;
void
testRootSolver()
{
  /*
  Test above code by finding the root of
  f(x1, x2) = [x1^2*X2^3-x1*x2^3-1, x1^3-x1*x2^3-4]
  with initX = (1, 1)
  */
  Function f1 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return std::pow(x1, 2) * std::pow(x2, 3) - x1 * std::pow(x2, 3) - 1;
  };
  Function f2 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return std::pow(x1, 3) - x1 * std::pow(x2, 3) - 4;
  };
  
  Function devF1 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return 2 * x1 * std::pow(x2, 3) - std::pow(x2, 3);
  };
  Function devF2 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return 3 * (std::pow(x1, 2) * std::pow(x2, 2) - x1 * std::pow(x2, 2));
  };
  Function devF3 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return 3 * std::pow(x1, 2) - std::pow(x2, 3);
  };
  Function devF4 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return -3 * x1 * std::pow(x2, 2);
  };

  ::NA::RootSolver solver;
  solver.addFunction(f1);
  solver.addFunction(f2);
  solver.setInitX({1, 1});
  solver.run();
  const std::vector<double>& sol = solver.solution();
  printf("Without derivatives: x1 = %f, x2 = %f\n", sol[0], sol[1]);

  ::NA::RootSolver solver2;
  solver2.addFunction(f1);
  solver2.addFunction(f2);
  solver2.addDerivativeFunction(devF1);
  solver2.addDerivativeFunction(devF2);
  solver2.addDerivativeFunction(devF3);
  solver2.addDerivativeFunction(devF4);
  solver2.setInitX({1, 1});
  solver2.run();
  const std::vector<double>& sol2 = solver2.solution();
  printf("With derivatives: x1 = %f, x2 = %f\n", sol[0], sol[1]);
}