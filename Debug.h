#ifndef _TRAN_DEBUG_H_
#define _TRAN_DEBUG_H_

#include <Eigen/Core>

namespace Tran {

class Debug {
  public:
    static bool enabled(size_t l = 0) { return _level > l; };
    static void setLevel(size_t l) { _level = l; }
    static void printEquation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    static void printVector(double time, const char* name, const Eigen::VectorXd& x);

  private:
    static size_t _level;

};

}

#endif
