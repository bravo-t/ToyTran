#ifndef _TRAN_MNASTM_H_
#define _TRAN_MNASTM_H_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace Tran {

class Simulator;

class MNAStamper {
  public:
    static void stamp(Eigen::MatrixXd& A, Eigen::VectorXd& b, const Simulator* simulator);
    static void updateA(Eigen::FullPivLU<Eigen::MatrixXd>& ALU, const Simulator* simulator);
    static void updateb(Eigen::VectorXd& b, const Simulator* simulator);
};

}

#endif