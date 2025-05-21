#ifndef _NA_RTSVR_H_
#define _NA_RTSVR_H_

#include <vector>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace NA {

class RootSolver {
  public:
    typedef std::function<double(const Eigen::VectorXd& x)> Function;

    RootSolver() = default;
    
    void addFunction(const Function& func);
    void addDerivativeFunction(const Function& devFunc);

    void setInitX(const std::vector<double>& x);
    void setXTol(double value) { _xTol = value; }
    void setMaxIteration(size_t value) { _maxIter = value; }

    bool run();
    std::vector<double> solution() const;
    size_t iterCount() const { return _iterCount; }

  private:
    bool check() const;

  private:
    std::vector<Function>   _functions;
    std::vector<Function>   _derivatives;
    Eigen::VectorXd         _x;
    double                  _xTol = 0.01;
    size_t                  _maxIter = 20;
    size_t                  _iterCount = 0;
};

}

void testRootSolver();

#endif