#include "Debug.h"

namespace Tran {

void 
Debug::printEquation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
  for (size_t i=0; i<A.rows(); ++i) {
    printf("  | ");
    for (size_t j=0; j<A.cols(); ++j) {
      printf("%8.5g ", A(i, j));
    }
    printf("|  ");
    if (i == A.rows()/2) {
      printf("* X = ");
    } else {
      printf("      ");
    }
    printf(" | %8.5g | \n", b(i));
  }
}

}