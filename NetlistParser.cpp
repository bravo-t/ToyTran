#include <fstream>
#include "NetlistParser.h"

namespace Tran {

NetlistParser::NetlistParser(const char* fileName) 
{
  std::ifstream infile(fileName);
  if (!infile) {
    printf("ERROR: Cannot open %s\n", worldFile);
    return false;
  }
  std::string line;
  while (std::getline(infile, line)) {
    parseLine(line);
  } 
}

void
NetlistParser::parseLine(const std::string& line)
{
  
}


}
