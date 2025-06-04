#include "RootSolver.h"
#include "Debug.h"

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
  _iterCount = 0;
  bool converged = false;
  while (!converged) {
    Eigen::Matrix Jac = Jacobian(_functions, _derivatives, _x);
    Eigen::VectorXd f(_x.rows());
    for (Eigen::Index i=0; i<_x.rows(); ++i) {
      f(i) = _functions[i](_x);
    }
    Eigen::VectorXd d = Jac.partialPivLu().solve(f);
    _x -= d;
    if (Debug::enabled()) {
      Debug::printEquation(Jac, f);
      Debug::printSolution("d", d);
      Debug::printSolution("x-d", _x);
    }
    converged = true;
    for (Eigen::Index i=0; i<_x.rows(); ++i) {
      if (std::abs(d(i)) > std::abs(_x(i)) * _xTol) {
        converged = false;
        break;
      }
    }
    ++_iterCount;
    if (_iterCount > _maxIter) {
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
  
  Function df1dx1 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return 2 * x1 * std::pow(x2, 3) - std::pow(x2, 3);
  };
  Function df1dx2 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return 3 * (std::pow(x1, 2) * std::pow(x2, 2) - x1 * std::pow(x2, 2));
  };
  Function df2dx1 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return 3 * std::pow(x1, 2) - std::pow(x2, 3);
  };
  Function df2dx2 = [](const Eigen::VectorXd& x)->double {
    double x1 = x(0);
    double x2 = x(1);
    return -3 * x1 * std::pow(x2, 2);
  };

  double answer[2] = {1.74762, 0.91472};
  ::NA::RootSolver solver;
  solver.addFunction(f1);
  solver.addFunction(f2);
  solver.setInitX({1, 1});
  solver.setXTol(1e-4);
  solver.run();
  const std::vector<double>& sol = solver.solution();
  printf("Without derivatives %lu iter: x1 = %f, x2 = %f\n", solver.iterCount(), sol[0], sol[1]);
  assert(std::abs(answer[0]-sol[0]) < 1e-5 && 
         std::abs(answer[1]-sol[1]) < 1e-5);

  ::NA::RootSolver solver2;
  solver2.addFunction(f1);
  solver2.addFunction(f2);
  solver2.addDerivativeFunction(df1dx1);
  solver2.addDerivativeFunction(df1dx2);
  solver2.addDerivativeFunction(df2dx1);
  solver2.addDerivativeFunction(df2dx2);
  solver2.setInitX({1, 1});
  solver2.setXTol(1e-4);
  solver2.run();
  const std::vector<double>& sol2 = solver2.solution();
  printf("With derivatives %lu iter: x1 = %f, x2 = %f\n", solver2.iterCount(), sol[0], sol[1]);
  assert(std::abs(answer[0]-sol2[0]) < 1e-5 && 
         std::abs(answer[1]-sol2[1]) < 1e-5);
}