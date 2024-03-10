#include "Debug.h"

namespace Tran {

size_t Debug::_level = 1;

void 
Debug::printEquation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
  for (Eigen::Index i=0; i<A.rows(); ++i) {
    printf("  | ");
    for (Eigen::Index j=0; j<A.cols(); ++j) {
      printf("%7.4g ", A(i, j));
    }
    printf("|  ");
    if (i == A.rows()/2) {
      printf("* X = ");
    } else {
      printf("      ");
    }
    printf(" | %7.4g | \n", b(i));
  }
}

}
