#ifndef _TRAN_MNASTM_H_
#define _TRAN_MNASTM_H_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace Tran {

class Simulator;

class MNAStamper {
  public:
    MNAStamper() = default;
    void setIsSDomain(bool val) { _isSDomain = val; }
    void stamp(Eigen::MatrixXd& G, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Simulator* simulator);
    void updateb(Eigen::VectorXd& b, const Simulator* simulator);

  private:
    bool _isSDomain = false;
};

}

#endif