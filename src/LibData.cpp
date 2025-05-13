#include <fstream>
#include "LibData.h"
#include "StringUtil.h"

namespace NA {
    
typedef std::unordered_map<std::string, std::vector<NLDMArc>> NLDMMap;
typedef std::unordered_map<std::string, std::vector<CCSArc>>  CCSMap;


struct SortCCSLUT {
  bool operator()(const CCSLUT& a, const CCSLUT& b) const 
  {
    if (a.inputTransition() == b.inputTransition()) {
      return a.outputLoad() < b.outputLoad();
    } 
    return a.inputTransition() < b.inputTransition();
  }
};

void 
CCSGroup::sortTable()
{
  std::sort(_ccsluts.begin(), _ccsluts.end(), SortCCSLUT());
}

class LibReader {
  public:
    LibReader(LibData* owner)
    : _owner(owner) {}
    void readFile(const char* datFile);
  private:
    LibData* _owner;
};

size_t 
numLeadSpaces(const std::string& line) 
{
  size_t count = 0;
  while (line[count] == ' ') {
    ++count;
  }
  return count;
}

void 
getArcInfo(const std::string& line, 
           std::string& fromPin, std::string& toPin, bool& isInverted)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  fromPin = strs[0];
  toPin = strs[1];
  if (strs[2] == "negative_unate") {
    isInverted = true;
  } else if (strs[2] == "positive_unate") {
    isInverted = false;
  } else {
    printf("ERROR: Unsupported arc type \"%s\"\n", strs[2].data());
  }
}

std::vector<double>
parseLineNumbers(const std::string& line)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ,", strs);
  std::vector<double> n;
  n.reserve(strs.size());
  for (const std::string& substr : strs) {
    n.push_back(std::stod(substr));
  }
  return n;
}

double
parseLineNumber(const std::string& line)
{
  return std::stod(line);
}

void
readNLDMLUT(std::ifstream& infile, NLDMLUT& data)
{
  std::string line;
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> index1 = parseLineNumbers(line);
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> index2 = parseLineNumbers(line);
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> values = parseLineNumbers(line);
  data.setIndex1(index1);
  data.setIndex2(index2);
  data.setValues(values);
}

void
readCCSLUT(std::ifstream& infile, CCSLUT& data)
{
  std::string line;
  std::getline(infile, line);
  line = trim(line);
  double refTime = parseLineNumber(line);
  std::getline(infile, line);
  line = trim(line);
  double index1 = parseLineNumber(line);
  std::getline(infile, line);
  line = trim(line);
  double index2 = parseLineNumber(line);
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> index3 = parseLineNumbers(line);
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> values = parseLineNumbers(line);
  data.init(refTime, index1, index2, index3, values);
}

struct SortArcDataByPin {
  bool operator()(const NLDMArc& a, const NLDMArc& b) const {
    if (a.fromPin() == b.fromPin()) {
      return a.toPin() < b.toPin();
    }
    return a.fromPin() < b.fromPin();
  }
  bool operator()(const CCSArc& a, const CCSArc& b) const {
    if (a.fromPin() == b.fromPin()) {
      return a.toPin() < b.toPin();
    }
    return a.fromPin() < b.fromPin();
  }
};

void
LibReader::readFile(const char* datFile)
{
  std::ifstream infile(datFile);
  if (!infile) {
    printf("ERROR: Cannot open %s\n", datFile);
    return;
  }
  std::string cellName;
  std::string fromPin;
  std::string toPin;
  bool isInverted;
  NLDMArc nldmArc;
  CCSArc ccsArc;
  std::vector<NLDMArc> nldmData;
  std::vector<CCSArc> ccsData;
  std::string line;
  while (std::getline(infile, line)) {
    size_t numSpace = numLeadSpaces(line);
    line = trim(line);
    if (numSpace == 0) {
      if (cellName.size() > 0) {
        std::sort(nldmData.begin(), nldmData.end(), SortArcDataByPin());
        std::sort(ccsData.begin(), ccsData.end(), SortArcDataByPin());
        _owner->nldmData.insert({cellName, nldmData});
        _owner->ccsData.insert({cellName, ccsData});
        nldmData.clear();
        ccsData.clear();
      }
      cellName = line;
    } else if (numSpace == 2) {
      if (fromPin.size() > 0 || toPin.size() > 0) {
        nldmData.push_back(nldmArc);
        ccsData.push_back(ccsArc);
        nldmArc.reset();
        ccsArc.reset();
      }
      getArcInfo(line, fromPin, toPin, isInverted);
    } else if (numSpace == 4) {
      if (line == "Rise Delay") {
        readNLDMLUT(infile, nldmArc.getLUT(DataType::RiseDelay));
      } else if (line == "Fall Delay") {
        readNLDMLUT(infile, nldmArc.getLUT(DataType::FallDelay));
      } else if (line == "Rise Transition") {
        readNLDMLUT(infile, nldmArc.getLUT(DataType::RiseTransition));
      } else if (line == "Fall Transition") {
        readNLDMLUT(infile, nldmArc.getLUT(DataType::FallTransition));
      } else if (line == "DC Current") {
        readNLDMLUT(infile, ccsArc.getDCCurrent());
      } else if (line == "Current Rise") {
        std::string line;
        std::getline(infile, line);
        line = trim(line);
        size_t tableCount = std::stoi(line);
        CCSGroup& riseCurrents = ccsArc.getCurrent(DataType::RiseCurrent);
        for (size_t i=0; i<tableCount; ++i) {
          CCSLUT lut;
          readCCSLUT(infile, lut);
          riseCurrents.addLUT(lut);
        }
        riseCurrents.sortTable();
      } else if (line == "Current Fall") {
        std::string line;
        std::getline(infile, line);
        line = trim(line);
        size_t tableCount = std::stoi(line);
        CCSGroup& fallCurrents = ccsArc.getCurrent(DataType::FallCurrent);
        for (size_t i=0; i<tableCount; ++i) {
          CCSLUT lut;
          readCCSLUT(infile, lut);
          fallCurrents.addLUT(lut);
        }
        fallCurrents.sortTable();
      } else if (line == "Receiver Cap Rise") {
        readNLDMLUT(infile, ccsArc.getRecvCap(DataType::RiseRecvCap));
      } else if (line == "Receiver Cap Fall") {
        readNLDMLUT(infile, ccsArc.getRecvCap(DataType::FallRecvCap));
      }
    }
  }
}

LibData::LibData(const char* datFile)
{
  LibReader reader(this);
  reader.readFile(datFile);
}

const NLDMArc*
LibData::findNLDMArc(const char* cell, const char* fromPin, const char* toPin) const
{
  std::string cellStr(cell);
  std::string fromPinStr(fromPin);
  std::string toPinStr(toPin);
  const auto& it = _nldmData.find(cellStr);
  if (it == _nldmData.end()) {
    return nullptr;
  } 
  const auto& arcs = it.second;
  NLDMArc tempArc;
  tempArc.setFromToPin(fromPinStr, toPinStr, true);
  const auto& it2 = std::lower_bound(arcs.begin(), arcs.end(), tempArc, SortArcDataByPin());
  if (it2 != arcs.end() && 
      strcmp(it2->fromPin(), fromPin) == 0 && 
      strcmp(it2->toPin(), toPin) == 0) {
    return it2;
  } 
  return nullptr;
}

const CCSArc*
LibData::findCCSArc(const char* cell, const char* fromPin, const char* toPin) const
{
  std::string cellStr(cell);
  std::string fromPinStr(fromPin);
  std::string toPinStr(toPin);
  const auto& it = _ccsData.find(cellStr);
  if (it == _ccsData.end()) {
    return nullptr;
  } 
  const auto& arcs = it.second;
  CCSArc tempArc;
  tempArc.setFromToPin(fromPinStr, toPinStr, true);
  const auto& it2 = std::lower_bound(arcs.begin(), arcs.end(), tempArc, SortArcDataByPin());
  if (it2 != arcs.end() && 
      strcmp(it2->fromPin(), fromPin) == 0 && 
      strcmp(it2->toPin(), toPin) == 0) {
    return it2;
  } 
  return nullptr;
}

}