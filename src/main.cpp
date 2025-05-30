#include <cstdio>
#include "DelayCalculator.h"
#include "NetworkAnalyzer.h"

int main(int argc, char** argv) 
{
  if (argc < 2) {
    printf("Input file missing, please provide a circuit netlist\n");
    return 1;
  }

  NA::NetworkAnalyzer::run(argv[1]);
  /// TODO: separate delay calculator to another main file
  NA::DelayCalculator::run(argv[1]);

  return 0;
}
