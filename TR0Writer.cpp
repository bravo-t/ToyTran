#include <fstream>
#include <cmath>
#include <cstring>
#include <ctime>
#include <tuple>
#include <iomanip>
#include "TR0Writer.h"
#include "Simulator.h"
#include "Circuit.h"

namespace Tran {

void
formatNumber(double n, std::string& string, 
             int significandWidth, int exponentWidth)
{
  string.clear();
  if (n == 0) {
    string = "0.0000000E+00";
    return;
  }
  int exponent = (int)log10(fabs(n)) + 1;
  double mantissa = n / pow(10, exponent);
  if (mantissa < 0.1) {
    mantissa *= 10;
    exponent -= 1;
  }
  char expn[15];
  sprintf(expn, "%+0*d", exponentWidth, exponent);
  char mts[20];
  sprintf(mts, "%*f", significandWidth, mantissa);
  int formatedMtsLength = strlen(mts);
  size_t startOffset = 0;
  for (int i=0; i<formatedMtsLength; ++i) {
    if (mts[i] != ' ') {
      startOffset = i;
      break;
    }
  }
  string = (mts + startOffset);
  int tailingZeros = significandWidth + exponentWidth - string.size() - strlen(expn);
  if (tailingZeros > 0) {
    std::string zeros(tailingZeros, '0');
    string += zeros;
  }
  string += "E";
  string += expn;
}

std::vector<std::pair<int, std::string>>
columnHeader(const SimResultMap& map, const Circuit& ckt)
{
  std::pair<int, std::string> initValue(0, "");
  std::vector<std::pair<int, std::string>> header(map.size()+1, initValue);
  header[0] = {1, "TIME"};
  for (const auto& kv : map._nodeVoltageMap) {
    size_t nodeId = kv.first;
    size_t index = kv.second;
    std::pair<int, std::string> value(1, ckt.node(nodeId)._name);
    header[index+1] = value;
  }
  for (const auto& kv : map._deviceCurrentMap) {
    size_t devId = kv.first;
    size_t index = kv.second;
    std::pair<int, std::string> value(8, ckt.device(devId)._name);
    header[index+1] = value;
  }
  return header;
}

static void
writeHeader(std::ofstream& out, const Circuit& ckt, const SimResult& result) 
{
  int n = result._map.size();
  char buf[5];
  sprintf(buf, "%04d", n);
  out << buf << "000000000000000" << std::endl;
  std::time_t timeResult = std::time(nullptr);
  out << std::put_time(std::localtime(&timeResult), "%c") << " "
      << "Data generated by ToyTran, written Bin Tang" << std::endl;
  out << 0 << std::endl;
  out << 1 << std::endl;
  const SimResultMap& map = result._map;
  const std::vector<std::pair<int, std::string>>& headerCol = columnHeader(map, ckt);
  for (size_t i=0; i<headerCol.size(); ++i) {
    const auto& data = headerCol[i];
    out << data.first << " ";
    if (i != headerCol.size() - 1) {
      out << " ";
    } else {
      out << std::endl;
    }
  } 
  for (size_t i=0; i<headerCol.size(); ++i) {
    const auto& data = headerCol[i];
    if (data.first == 1) {
      if (i != 0 || data.second.compare("TIME") != 0) {
        out << "V(";
      }
    } else if (data.first == 8) {
      out << "I(";
    } else {
      assert(false && "Unrecognized header type value");
    }
    out << data.second;
    if ((i + 1) % 3 == 0) {
      out << std::endl;
    } else {
      out << " ";
    }
  } 
  out << " $&%#" << std::endl;
}

static void
writeData(std::ofstream& out, const SimResult& result, 
          int significandWidth, int exponentWidth)
{
  std::string outStr;
  size_t cols = result._map.size();
  for (size_t t=0; t<result._ticks.size(); ++t) {
    formatNumber(result._ticks[t], outStr, significandWidth, exponentWidth);
    out << outStr << " ";
    for (size_t i=0; i<cols; ++i) {
      size_t index = t*cols+i;
      formatNumber(result._values[index], outStr, significandWidth, exponentWidth);
      out << outStr;
      if (i == cols-1) {
        out << std::endl;
      } else {
        out << " ";
      }
    }
  }
  out << "0.1000000E+31" << std::endl;
}

void 
TR0Writer::adjustNumberWidth(double simTick, double simTime)
{
  int n = ((int) log10(fabs(simTime/simTick)) + 1);
  if (n > _significandWidth) {
    _significandWidth = n;
    printf("Significand width of tr0 has been adjusted to %d digits due to wide range in simulation time\n", _significandWidth);
  }
}

void 
TR0Writer::writeData(const SimResult& result) const
{
  std::ofstream out (_outFile, std::ofstream::out);
  writeHeader(out, _ckt, result);
  ::Tran::writeData(out, result, _significandWidth, _exponentWidth);
}

}