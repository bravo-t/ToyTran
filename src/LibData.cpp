#include <fstream>
#include <algorithm>
#include <cassert>
#include "LibData.h"
#include "StringUtil.h"

namespace NA {
    
typedef std::unordered_map<std::string, std::vector<NLDMArc>> NLDMMap;
typedef std::unordered_map<std::string, std::vector<CCSArc>>  CCSMap;

NLDMLUT&
NLDMArc::getLUT(DataType dataType) 
{
  switch (dataType) {
    case DataType::RiseDelay: {
      return _riseDelay;
    }
    case DataType::FallDelay: {
      return _fallDelay;
    } 
    case DataType::RiseTransition: {
      return _riseTransition;
    } 
    case DataType::FallTransition: {
      return _fallTransition;
    } 
    default:
      assert(false);
  }
}

NLDMLUT&
CCSArc::getRecvCap(DataType dataType)
{
  switch (dataType) {
    case DataType::RiseRecvCap: {
      return _riseRecvCap;
    } 
    case DataType::FallRecvCap: {
      return _fallRecvCap;
    } 
    default:
      assert(false);
  }
}

CCSGroup&
CCSArc::getCurrent(DataType dataType)
{
  switch (dataType) {
    case DataType::RiseCurrent: {
      return _riseCurrent;
    } 
    case DataType::FallCurrent: {
      return _fallCurrent;
    }
    default:
      assert(false);
  }
}

NLDMLUT&
CCSArc::getDCCurrent()
{
  return _dcCurrent;
}

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
parseLineNumbers(const std::string& line, double unit)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ,", strs);
  std::vector<double> n;
  n.reserve(strs.size());
  for (const std::string& substr : strs) {
    n.push_back(std::stod(substr) * unit);
  }
  return n;
}

double
parseLineNumber(const std::string& line, double unit)
{
  return std::stod(line) * unit;
}

void
readNLDMLUT(std::ifstream& infile, NLDMLUT& data, 
            double index1Unit, double index2Unit, double valueUnit)
{
  std::string line;
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> index1 = parseLineNumbers(line, index1Unit);
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> index2 = parseLineNumbers(line, index2Unit);
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> values = parseLineNumbers(line, valueUnit);
  data.setIndex1(index1);
  data.setIndex2(index2);
  data.setValues(values);
}

void
readCCSLUT(std::ifstream& infile, CCSLUT& data, 
           double timeUnit, double index1Unit, double index2Unit, 
           double index3Unit, double valueUnit)
{
  std::string line;
  std::getline(infile, line);
  line = trim(line);
  double refTime = parseLineNumber(line, timeUnit);
  std::getline(infile, line);
  line = trim(line);
  double index1 = parseLineNumber(line, index1Unit);
  std::getline(infile, line);
  line = trim(line);
  double index2 = parseLineNumber(line, index2Unit);
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> index3 = parseLineNumbers(line, index3Unit);
  std::getline(infile, line);
  line = trim(line);
  std::vector<double> values = parseLineNumbers(line, valueUnit);
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
  double timeUnit = 1;
  double voltageUnit = 1;
  double currentUnit = 1;
  double capUnit = 1;
  //double resUnit = 1;
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
        _owner->_nldmData.insert({cellName, nldmData});
        _owner->_ccsData.insert({cellName, ccsData});
        nldmData.clear();
        ccsData.clear();
      }
      std::vector<std::string> strs;
      splitWithAny(line, " ", strs);
      if (strs[0] == ".UNIT") {
        for (size_t i=1; i<strs.size(); ++i) {
          if (strs[i] == "T") {
            ++i;
            timeUnit = std::stod(strs[i]);
          } else if (strs[i] == "V") {
            ++i;
            voltageUnit = std::stod(strs[i]);
          } else if (strs[i] == "I") {
            ++i;
            currentUnit = std::stod(strs[i]);
          } else if (strs[i] == "C") {
            ++i;
            capUnit = std::stod(strs[i]);
          } else if (strs[i] == "R") {
            ++i;
            //resUnit = std::stod(strs[i]);
          }
        }
      } else if (strs[0] == ".THRES") {
        for (size_t i=1; i<strs.size(); ++i) {
          if (strs[i] == "R") {
            ++i;
            _owner->_transitionRiseLowThres = std::stod(strs[i]);
            ++i;
            _owner->_transitionRiseHighThres = std::stod(strs[i]);
          } else if (strs[i] == "F") {
            ++i;
            _owner->_transitionFallHighThres = std::stod(strs[i]);
            ++i;
            _owner->_transitionFallLowThres = std::stod(strs[i]);
          } else if (strs[i] == "D") {
            ++i;
            _owner->_delayRiseThres = std::stod(strs[i]);
            ++i;
            _owner->_delayFallThres = std::stod(strs[i]);
          } else if (strs[i] == "Vol") {
            ++i;
            _owner->_voltage = std::stod(strs[i]);
          }
        }
      } else {
        cellName = line;
        std::vector<FixedLoadCap> pinCaps;
        for (size_t i=1; i<strs.size(); ++i) {
          FixedLoadCap cap;
          cap.setPinName(strs[i]);
          cap.setCaps(std::stod(strs[i+1]), std::stod(strs[i+2]));
          pinCaps.push_back(cap);
          i += 2;
        }
        std::sort(pinCaps.begin(), pinCaps.end(), 
          [](const FixedLoadCap& a, const FixedLoadCap& b) {
            return a.pinName() < b.pinName();
          });
        _owner->_loadCaps.insert({cellName, pinCaps});
      }
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
        readNLDMLUT(infile, nldmArc.getLUT(DataType::RiseDelay), timeUnit, capUnit, timeUnit);
      } else if (line == "Fall Delay") {
        readNLDMLUT(infile, nldmArc.getLUT(DataType::FallDelay), timeUnit, capUnit, timeUnit);
      } else if (line == "Rise Transition") {
        readNLDMLUT(infile, nldmArc.getLUT(DataType::RiseTransition), timeUnit, capUnit, timeUnit);
      } else if (line == "Fall Transition") {
        readNLDMLUT(infile, nldmArc.getLUT(DataType::FallTransition), timeUnit, capUnit, timeUnit);
      } else if (line == "DC Current") {
        readNLDMLUT(infile, ccsArc.getDCCurrent(), voltageUnit, voltageUnit, currentUnit);
      } else if (line == "Current Rise") {
        std::string line;
        std::getline(infile, line);
        line = trim(line);
        size_t tableCount = std::stoi(line);
        CCSGroup& riseCurrents = ccsArc.getCurrent(DataType::RiseCurrent);
        for (size_t i=0; i<tableCount; ++i) {
          CCSLUT lut;
          readCCSLUT(infile, lut, timeUnit, timeUnit, capUnit, timeUnit, currentUnit);
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
          readCCSLUT(infile, lut, timeUnit, timeUnit, capUnit, timeUnit, currentUnit);
          fallCurrents.addLUT(lut);
        }
        fallCurrents.sortTable();
      } else if (line == "Receiver Cap Rise") {
        readNLDMLUT(infile, ccsArc.getRecvCap(DataType::RiseRecvCap), timeUnit, capUnit, timeUnit);
      } else if (line == "Receiver Cap Fall") {
        readNLDMLUT(infile, ccsArc.getRecvCap(DataType::FallRecvCap), timeUnit, capUnit, timeUnit);
      }
    }
  }
}

LibData::LibData(const std::vector<const char*>& datFiles)
{
  LibReader reader(this);
  for (const char* datFile : datFiles) {
    printf("Reading Lib data file %s\n", datFile);
    reader.readFile(datFile);
  }
}

void 
LibData::read(const std::vector<std::string>& datFiles)
{
  LibReader reader(this);
  for (const std::string& datFile : datFiles) {
    printf("Reading Lib data file %s\n", datFile.data());
    reader.readFile(datFile.data());
  }
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
  const auto& arcs = it->second;
  NLDMArc tempArc;
  tempArc.setFromToPin(fromPinStr, toPinStr, true);
  const auto& it2 = std::lower_bound(arcs.begin(), arcs.end(), tempArc, SortArcDataByPin());
  if (it2 != arcs.end() && 
      strcmp(it2->fromPin(), fromPin) == 0 && 
      strcmp(it2->toPin(), toPin) == 0) {
    return &(*it2);
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
  const auto& arcs = it->second;
  CCSArc tempArc;
  tempArc.setFromToPin(fromPinStr, toPinStr, true);
  const auto& it2 = std::lower_bound(arcs.begin(), arcs.end(), tempArc, SortArcDataByPin());
  if (it2 != arcs.end() && 
      strcmp(it2->fromPin(), fromPin) == 0 && 
      strcmp(it2->toPin(), toPin) == 0) {
    /// TODO: Find out why this strange syntax is needed
    return &(*it2);
  } 
  return nullptr;
}

bool
LibData::isOutputPin(const char* cell, const char* pin) const
{
  std::string cellStr(cell);
  std::string pinStr(pin);
  const auto& it = _loadCaps.find(cellStr);
  if (it == _loadCaps.end()) {
    return false;
  }
  const auto& pinCaps = it->second;
  const auto& it2 = std::lower_bound(pinCaps.begin(), pinCaps.end(), pinStr, 
                      [](const FixedLoadCap& a, const std::string& b) {
                        return a.pinName() < b;
                      });

  if (it2 != pinCaps.end() && it2->pinName() == pinStr) {
    return true;
  }
  return false;
}

const NLDMArc*
LibData::findNLDMArc(const std::string& cell, const std::string& fromPin, const std::string& toPin) const
{
  const auto& it = _nldmData.find(cell);
  if (it == _nldmData.end()) {
    return nullptr;
  } 
  const auto& arcs = it->second;
  NLDMArc tempArc;
  tempArc.setFromToPin(fromPin, toPin, true);
  const auto& it2 = std::lower_bound(arcs.begin(), arcs.end(), tempArc, SortArcDataByPin());
  if (it2 != arcs.end() && 
      strcmp(it2->fromPin(), fromPin.data()) == 0 && 
      strcmp(it2->toPin(), toPin.data()) == 0) {
    return &(*it2);
  } 
  return nullptr;
}

const CCSArc*
LibData::findCCSArc(const std::string& cell, const std::string& fromPin, const std::string& toPin) const
{
  const auto& it = _ccsData.find(cell);
  if (it == _ccsData.end()) {
    return nullptr;
  } 
  const auto& arcs = it->second;
  CCSArc tempArc;
  tempArc.setFromToPin(fromPin, toPin, true);
  const auto& it2 = std::lower_bound(arcs.begin(), arcs.end(), tempArc, SortArcDataByPin());
  if (it2 != arcs.end() && 
      strcmp(it2->fromPin(), fromPin.data()) == 0 && 
      strcmp(it2->toPin(), toPin.data()) == 0) {
    /// TODO: Find out why this strange syntax is needed
    return &(*it2);
  } 
  return nullptr;
}

bool
LibData::isOutputPin(const std::string& cell, const std::string& pin) const
{
  const auto& it = _loadCaps.find(cell);
  if (it == _loadCaps.end()) {
    return false;
  }
  const auto& pinCaps = it->second;
  const auto& it2 = std::lower_bound(pinCaps.begin(), pinCaps.end(), pin, 
                      [](const FixedLoadCap& a, const std::string& b) {
                        return a.pinName() < b;
                      });

  if (it2 != pinCaps.end() && it2->pinName() == pin) {
    return true;
  }
  return false;
}


}