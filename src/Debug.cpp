#include "Debug.h"
#include "Circuit.h"
#include "Simulator.h"

namespace NA {

size_t Debug::_level = 0;

const int debugDigits = 5;
const int debugDigitLength = 8;
const int debugComplexDigits = 3;
const int debugComplexDigitLength = 10;

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

int 
maxFloatLength(const Eigen::MatrixXcd& m)
{
  char temp[100];
  int maxLength = debugComplexDigitLength;
  for (Eigen::Index i=0; i<m.rows(); ++i) {
    for (Eigen::Index j=0; j<m.cols(); ++j) {
      int length = sprintf(temp, "%.*g+%.*gi", debugComplexDigits, m(i,j).real(), debugComplexDigits, m(i,j).imag());
      if (length > 0) {
        maxLength = maxLength < length ? length : maxLength;
      }
    }
  }
  return maxLength;
}

int 
maxFloatLength(const Eigen::VectorXcd& v)
{
  char temp[100];
  int maxLength = debugComplexDigitLength;
  for (Eigen::Index i=0; i<v.rows(); ++i) {
    int length = sprintf(temp, "%.*g+%.*gi", debugComplexDigits, v(i).real(), debugComplexDigits, v(i).imag());
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
Debug::printEquation(const Eigen::MatrixXcd& A, const Eigen::VectorXcd& b)
{
  int matrixElementLength = maxFloatLength(A);
  printf("  --");
  for (Eigen::Index j=0; j<A.cols(); ++j) {
    for (int c=0; c<matrixElementLength*2; ++c) printf(" ");
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
  for (int c=0; c<vectorElementLength*2; ++c) printf(" ");
  printf("--\n");
  
  for (Eigen::Index i=0; i<A.rows(); ++i) {
    printf("  | ");
    for (Eigen::Index j=0; j<A.cols(); ++j) {
      printf("% *.*g+% *.*gi ", matrixElementLength, debugComplexDigits, A(i, j).real(), matrixElementLength, debugComplexDigits, A(i, j).imag());
    }
    printf("|  ");
    if (i == A.rows()/2) {
      printf("* X = ");
    } else {
      printf("      ");
    }
    printf(" | % *.*g+% *.*gi | \n", vectorElementLength, debugComplexDigits, b(i).real(), vectorElementLength, debugComplexDigits, b(i).imag());
  }
    
  printf("  --");
  for (Eigen::Index j=0; j<A.cols(); ++j) {
    for (int c=0; c<matrixElementLength*2; ++c) printf(" ");
    if (j == A.cols()-1) {
      printf("-");
    } else {
      printf(" ");
    }
  }
  printf("-   ");
  printf("      ");
  printf("--");
  for (int c=0; c<vectorElementLength*2; ++c) printf(" ");
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

void 
Debug::printVector(const char* name, const Eigen::VectorXcd& x)
{
  int vectorElementLength = maxFloatLength(x);
  int nameLength = strlen(name);
  int spaceLength = nameLength + 3;
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength*2; ++c) printf(" ");
  printf("--\n");

  for (Eigen::Index i=0; i<x.rows(); ++i) {
    if (i == x.rows()/2) {
      printf("%s = ", name);
    } else {
      for (int c=0; c<spaceLength; ++c) printf(" ");
    }
    printf(" | % *.*g+% *.*gi | \n", vectorElementLength, debugComplexDigits, x(i).real(), vectorElementLength, debugComplexDigits, x(i).imag());
  }
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength*2; ++c) printf(" ");
  printf("--\n");
}

std::vector<std::string>
rowName(const SimResultMap& map, const Circuit& ckt)
{
  std::vector<std::string> names(map.size()+1, "");
  for (size_t nodeId=0; nodeId<map._nodeVoltageMap.size(); ++nodeId) {
    size_t index = map._nodeVoltageMap[nodeId];
    if (index == SimResultMap::invalidValue()) {
      continue;
    }
    names[index] = "V(" + ckt.node(nodeId)._name + ")";
  }
  for (size_t devId=0; devId<map._deviceCurrentMap.size(); ++devId) {
    size_t index = map._deviceCurrentMap[devId];
    if (index == SimResultMap::invalidValue()) {
      continue;
    }
    names[index] = "I(" + ckt.device(devId)._name + ")";
  }
  return names;
}

void 
Debug::printSolution(double time, const char* name, const Eigen::VectorXd& x,
                     const SimResultMap& resultMap, const Circuit& ckt)
{
  const std::vector<std::string>& names = rowName(resultMap, ckt);
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

void 
Debug::printSolution(const char* name, const Eigen::VectorXd& x)
{
  int vectorElementLength = maxFloatLength(x);
  int nameLength = strlen(name);
  int spaceLength = nameLength + 3;
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength; ++c) printf(" ");
  printf("--\n");

  for (Eigen::Index i=0; i<x.rows(); ++i) {
    if (i == x.rows()/2) {
      printf("%s = ", name);
    } else {
      for (int c=0; c<spaceLength; ++c) printf(" ");
    }
    printf(" | % *.*g |\n", vectorElementLength, debugDigits, x(i));
  }
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength; ++c) printf(" ");
  printf("--\n");
}

void 
Debug::printSolution(const char* name, const Eigen::VectorXcd& x)
{
  int vectorElementLength = maxFloatLength(x);
  int nameLength = strlen(name);
  int spaceLength = nameLength + 3;
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength*2+1; ++c) printf(" ");
  printf("--\n");

  for (Eigen::Index i=0; i<x.rows(); ++i) {
    if (i == x.rows()/2) {
      printf("%s = ", name);
    } else {
      for (int c=0; c<spaceLength; ++c) printf(" ");
    }
    printf(" | % *.*g+% *.*gi |\n", vectorElementLength, debugComplexDigits, x(i).real(), vectorElementLength, debugComplexDigits, x(i).imag());
  }
  for (int c=0; c<spaceLength; ++c) printf(" ");
  printf(" --");
  for (int c=0; c<vectorElementLength*2+1; ++c) printf(" ");
  printf("--\n");
}

}
