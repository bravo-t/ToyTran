#include <cstdio>
#include "NetlistParser.h"
#include "Circuit.h"
#include "Simulator.h"

int main(int argc, char** argv) 
{
  Tran::NetlistParser parser(argv[1]);
  Tran::Circuit circuit(parser);
  Tran::Simulator simulator(circuit);
  simulator.run();
  return 0;
}
