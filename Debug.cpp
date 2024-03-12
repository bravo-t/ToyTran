#include "Debug.h"

namespace Tran {

size_t Debug::_level = 1;

void 
Debug::printEquation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
  printf("  --");
  for (Eigen::Index j=0; j<A.cols(); ++j) {
    printf("       ");
    if (j == A.cols()-1) {
      printf("-");
    } else {
      printf(" ");
    }
  }
  printf("-   ");
  printf("      ");
  printf("--       --\n");
  
  for (Eigen::Index i=0; i<A.rows(); ++i) {
    printf("  | ");
    for (Eigen::Index j=0; j<A.cols(); ++j) {
      printf("% 7.4g ", A(i, j));
    }
    printf("|  ");
    if (i == A.rows()/2) {
      printf("* X = ");
    } else {
      printf("      ");
    }
    printf(" | % 7.4g | \n", b(i));
  }
    
  printf("  --");
  for (Eigen::Index j=0; j<A.cols(); ++j) {
    printf("        ");
  }
  printf("--  ");
  printf("      ");
  printf("--       --\n");
}

void 
Debug::printMatrix(double time, const char* name, const Eigen::VectorXd& x)
{
  printf("              ");
  printf(" --       --\n");
  for (Eigen::Index i=0; i<x.rows(); ++i) {
    if (i == x.rows()/2) {
      printf("%s @ % 7.4g = ", name, time);
    } else {
      printf("              ");
    }
    printf(" | % 7.4g | \n", x(i));
  }
  printf("              ");
  printf(" --       --\n");
}

}
