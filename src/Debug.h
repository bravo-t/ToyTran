#ifndef _TRAN_DEBUG_H_
#define _TRAN_DEBUG_H_

#include <Eigen/Core>

namespace NA {

class Circuit;
class SimResultMap;

class Debug {
  public:
    static bool enabled(size_t l = 0) { return _level > l; };
    static void setLevel(size_t l) { _level = l; }
    static void printEquation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    static void printVector(double time, const char* name, const Eigen::VectorXd& x);
    static void printEquation(const Eigen::MatrixXcd& A, const Eigen::VectorXcd& b);
    static void printVector(const char* name, const Eigen::VectorXcd& x);
    static void printSolution(double time, const char* name, const Eigen::VectorXd& x,
                              const SimResultMap& resultMap, const Circuit& circuit);
    static void printSolution(const char* name, const Eigen::VectorXd& x);
    static void printSolution(const char* name, const Eigen::VectorXcd& x);

  private:
    static size_t _level;

};

}

#endif
