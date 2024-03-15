#include "Debug.h"
#include "Circuit.h"
#include "Simulator.h"

namespace Tran {

size_t Debug::_level = 0;

const int debugDigits = 5;
const int debugDigitLength = 8;

int 
maxFloatLength(const Eigen::MatrixXd& m)
{
  char temp[100];
  int maxLength = debugDigitLength;
  for (Eigen::Index i=0; i<m.rows(); ++i) {
    for (Eigen::Index j=0; j<m.cols(); ++j) {
      int length = sprintf(temp, "%.*g", debugDigits, m(i,j));
      if (length > 0) {
        maxLength = maxLength < length ? length : maxLength;
      }
    }
  }
  return maxLength;
}

int 
maxFloatLength(const Eigen::VectorXd& v)
{
  char temp[100];
  int maxLength = debugDigitLength;
  for (Eigen::Index i=0; i<v.rows(); ++i) {
    int length = sprintf(temp, "%.*g", debugDigits, v(i));
    if (length > 0) {
      maxLength = maxLength < length ? length : maxLength;
    }
  }
  return maxLength;
}

void 
Debug::printEquation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
  int matrixElementLength = maxFloatLength(A);
  printf("  --");
  for (Eigen::Index j=0; j<A.cols(); ++j) {
    for (int c=0; c<matrixElementLength; ++c) printf(" ");
    if (j == A.cols()-1) {
      printf("-");
    } else {
      printf(" ");
    }
  }
  printf("-   ");
  printf("      ");
  int vectorElementLength = maxFloatLength(b);
  printf("--");
  for (int c=0; c<vectorElementLength; ++c) printf(" ");
  printf("--\n");
  
  for (Eigen::Index i=0; i<A.rows(); ++i) {
    printf("  | ");
    for (Eigen::Index j=0; j<A.cols(); ++j) {
      printf("% *.*g ", matrixElementLength, debugDigits, A(i, j));
    }
    printf("|  ");
    if (i == A.rows()/2) {
      printf("* X = ");
    } else {
      printf("      ");
    }
    printf(" | % *.*g | \n", vectorElementLength, debugDigits, b(i));
  }
    
  printf("  --");
  for (Eigen::Index j=0; j<A.cols(); ++j) {
    for (int c=0; c<matrixElementLength; ++c) printf(" ");
    if (j == A.cols()-1) {
      printf("-");
    } else {
      printf(" ");
    }
  }
  printf("-   ");
  printf("      ");
  printf("--");
  for (int c=0; c<vectorElementLength; ++c) printf(" ");
  printf("--\n");
}

void 
Debug::printVector(double time, const char* name, const Eigen::VectorXd& x)
{
  int vectorElementLength = maxFloatLength(x);
  int nameLength = strlen(name);
  int spaceLength = vectorElementLength + nameLength + 6;
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength; ++c) printf(" ");
  printf("--\n");

  for (Eigen::Index i=0; i<x.rows(); ++i) {
    if (i == x.rows()/2) {
      printf("%s @ % *.*g = ", name, vectorElementLength, debugDigits, time);
    } else {
      for (int c=0; c<spaceLength; ++c) printf(" ");
    }
    printf(" | % *.*g | \n", vectorElementLength, debugDigits, x(i));
  }
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength; ++c) printf(" ");
  printf("--\n");
}

std::vector<std::string>
rowName(const Simulator* sim)
{
  const SimResultMap& map = sim->simulationResult()._map;
  const Circuit& ckt = sim->circuit();
  std::vector<std::string> names(map.size()+1, "");
  for (const auto& kv : map._nodeVoltageMap) {
    size_t nodeId = kv.first;
    size_t index = kv.second;
    names[index] = "V(" + ckt.node(nodeId)._name + ")";
  }
  for (const auto& kv : map._deviceCurrentMap) {
    size_t devId = kv.first;
    size_t index = kv.second;
    names[index] = "I(" + ckt.device(devId)._name + ")";
  }
  return names;
}

void 
Debug::printSolution(double time, const char* name, const Eigen::VectorXd& x, const Simulator* sim)
{
  const std::vector<std::string>& names = rowName(sim);
  int vectorElementLength = maxFloatLength(x);
  int nameLength = strlen(name);
  int spaceLength = vectorElementLength + nameLength + 6;
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength; ++c) printf(" ");
  printf("--\n");

  for (Eigen::Index i=0; i<x.rows(); ++i) {
    if (i == x.rows()/2) {
      printf("%s @ % *.*g = ", name, vectorElementLength, debugDigits, time);
    } else {
      for (int c=0; c<spaceLength; ++c) printf(" ");
    }
    printf(" | % *.*g | -> %s\n", vectorElementLength, debugDigits, x(i), names[i].data());
  }
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength; ++c) printf(" ");
  printf("--\n");
}

}
