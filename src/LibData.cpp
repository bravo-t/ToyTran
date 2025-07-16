#include <cstdio>
#include <cstddef>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "LibData.h"
#include "StringUtil.h"
#include "Debug.h"

namespace NA {
    
typedef std::unordered_map<std::string, std::vector<NLDMArc>> NLDMMap;
typedef std::unordered_map<std::string, std::vector<CCSArc>>  CCSMap;

static inline size_t 
binaryIndex(const std::vector<double>& values, float v)
{
  size_t lower = 1;
  size_t upper = values.size()-2;
  if (v <= values[lower]) {
    return 0;
  }
  if (v >= values[upper]) {
    return upper;
  }
  size_t idx = 0;
  while (upper - lower > 1) {
    idx = (lower + upper) >> 1;
    if (values[idx] > v) {
      upper = idx;
    } else {
      lower = idx;
    }
  }
  if (values[idx] > v) {
    --idx;
  }
  return idx;
}

static inline size_t
axisIndex(const std::vector<double>& values, double val)
{
  if (val <= values[1]) {
    return 0;
  }
  size_t lastIndex = values.size() - 1;
  if (val >= values[lastIndex]) {
    return lastIndex - 1;
  }
  for (size_t index = 1; index < lastIndex; ++index) {
    if (val < values[index]) {
      return index - 1;
    }
  }
  /// Suppress compiler warnings
  return static_cast<size_t>(-1);
}

static inline void
indexValues(const std::vector<double>& values, size_t X, size_t Y, size_t YDim, 
            double& Z1, double& Z2, double& Z3, double& Z4) 
{
  size_t i = 0;
  i = X * YDim + Y;
  Z1 = values[i];
  Z3 = values[i+1];
  i += YDim;
  Z2 = values[i];
  Z4 = values[i+1];
}

double 
NLDMLUT::value(double inputTran, double outputLoad) const
{
  size_t index1 = binaryIndex(_index1, inputTran);
  size_t index2 = binaryIndex(_index2, outputLoad);
  //size_t index1 = axisIndex(_index1, inputTran);
  //size_t index2 = axisIndex(_index2, outputLoad);
  size_t index2Dim = _index2.size();

  double x1 = _index1[index1];
  double x2 = _index1[index1+1];
  double y1 = _index2[index2];
  double y2 = _index2[index2+1];
  double z1, z2, z3, z4;
  indexValues(_values, index1, index2, index2Dim, z1, z2, z3, z4);
  if (false) {
    printf("DEBUG: inputTran: %G ([%G, %G]), outputLoad: %G ([%G, %G])\n", 
           inputTran, x1, x2, outputLoad, y1, y2);
    printf("DEBUG:                    (X1) %.6G      (X2) %.6G\n", x1, x2);
    printf("DEBUG: (Y1) %.6G         (Z1) %.6G      (Z2) %.6G\n", y1, z1, z2);
    printf("DEBUG: (Y2) %.6G         (Z3) %.6G      (Z4) %.6G\n", x2, z3, z4);
  }
  /*
               (X)  x1      (X)  x2
  (Y)  y1      (Z)  z1      (Z)  z2
  (Y)  y2      (Z)  z3      (Z)  z4

  z1 = A + B*x1 + C*y1 + D*x1*y1
  z2 = A + B*x2 + C*y1 + D*x2*y1
  z3 = A + B*x1 + C*y2 + D*x1*y2
  z4 = A + B*x2 + C*y2 + D*x2*y2
  */
  Eigen::MatrixXd A(4, 4);
  A(0, 0) = 1; A(0, 1) = x1; A(0, 2) = y1; A(0, 3) = x1 * y1;
  A(1, 0) = 1; A(1, 1) = x2; A(1, 2) = y1; A(1, 3) = x2 * y1;
  A(2, 0) = 1; A(2, 1) = x1; A(2, 2) = y2; A(2, 3) = x1 * y2;
  A(3, 0) = 1; A(3, 1) = x2; A(3, 2) = y2; A(3, 3) = x2 * y2;
  Eigen::Vector4d b;
  b(0) = z1; b(1) = z2; b(2) = z3; b(3) = z4;
  Eigen::Vector4d x = A.partialPivLu().solve(b);
  if (false) {
    Debug::printEquation(A, b);
    printf("DEBUG: x = [%.6G, %.6G, %.6G, %.6G]\n", x(0), x(1), x(2), x(3));
    printf("DEBUG: %G + %G*%G + %G*%G + %G*%G*%G = %G\n", x(0), x(1), inputTran, x(2), outputLoad, x(3), inputTran, outputLoad, 
    x(0) + x(1) * inputTran + x(2) * outputLoad + x(3) * inputTran * outputLoad);
  }
  return x(0) + x(1) * inputTran + x(2) * outputLoad + x(3) * inputTran * outputLoad;
}

const NLDMLUT&
NLDMArc::getLUT(LUTType dataType) const
{
  switch (dataType) {
    case LUTType::RiseDelay: {
      return _riseDelay;
    }
    case LUTType::FallDelay: {
      return _fallDelay;
    } 
    case LUTType::RiseTransition: {
      return _riseTransition;
    } 
    case LUTType::FallTransition: {
      return _fallTransition;
    } 
    default:
      assert(false);
  }
}

const NLDMLUT&
CCSArc::getRecvCap(LUTType dataType) const
{
  switch (dataType) {
    case LUTType::RiseRecvCap: {
      return _riseRecvCap;
    } 
    case LUTType::FallRecvCap: {
      return _fallRecvCap;
    } 
    default:
      assert(false);
  }
}

const CCSGroup&
CCSArc::getCurrent(LUTType dataType) const
{
  switch (dataType) {
    case LUTType::RiseCurrent: {
      return _riseCurrent;
    } 
    case LUTType::FallCurrent: {
      return _fallCurrent;
    }
    default:
      assert(false);
  }
}

const NLDMLUT&
CCSArc::getDCCurrent() const
{
  return _dcCurrent;
}

NLDMLUT&
NLDMArc::getLUT(LUTType dataType) 
{
  switch (dataType) {
    case LUTType::RiseDelay: {
      return _riseDelay;
    }
    case LUTType::FallDelay: {
      return _fallDelay;
    } 
    case LUTType::RiseTransition: {
      return _riseTransition;
    } 
    case LUTType::FallTransition: {
      return _fallTransition;
    } 
    default:
      assert(false);
  }
}

NLDMLUT&
CCSArc::getRecvCap(LUTType dataType)
{
  switch (dataType) {
    case LUTType::RiseRecvCap: {
      return _riseRecvCap;
    } 
    case LUTType::FallRecvCap: {
      return _fallRecvCap;
    } 
    default:
      assert(false);
  }
}

CCSGroup&
CCSArc::getCurrent(LUTType dataType)
{
  switch (dataType) {
    case LUTType::RiseCurrent: {
      return _riseCurrent;
    } 
    case LUTType::FallCurrent: {
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
  if (_ccsluts.empty()) {
    return;
  }
  std::sort(_ccsluts.begin(), _ccsluts.end(), SortCCSLUT());
  double prevTrans = _ccsluts[0].inputTransition();
  _transDiv.push_back(0);
  for (size_t i=1; i<_ccsluts.size(); ++i) {
    if (_ccsluts[i].inputTransition() != prevTrans) {
      _transDiv.push_back(i);
      prevTrans = _ccsluts[i].inputTransition();
    }
  }
  /// something works like the end() iterator of std::vector
  _transDiv.push_back(_ccsluts.size());
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
    if (a.toPin() == b.toPin()) {
      return strcmp(a.fromPin(), b.fromPin()) < 0;
    }
    return strcmp(a.toPin(), b.toPin()) < 0;
  }
  bool operator()(const CCSArc& a, const CCSArc& b) const {
    if (a.toPin() == b.toPin()) {
      return strcmp(a.fromPin(), b.fromPin()) < 0;
      return a.fromPin() < b.fromPin();
    }
    return strcmp(a.toPin(), b.toPin()) < 0;
  }
  /// Used to find input pins from output pin
  bool operator()(const NLDMArc& a, const std::string& toPinToFind) const
  {
    return strcmp(a.toPin(), toPinToFind.data()) < 0;
  }
  bool operator()(const std::string& toPinToFind, const NLDMArc& a) const
  {
    return strcmp(toPinToFind.data(), a.toPin()) < 0;
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
  NLDMArc nldmArc(_owner);
  CCSArc ccsArc(_owner);
  std::vector<NLDMArc> nldmData;
  std::vector<CCSArc> ccsData;
  std::string line;
  while (std::getline(infile, line)) {
    size_t numSpace = numLeadSpaces(line);
    line = trim(line);
    if (numSpace == 0) {
      if (cellName.size() > 0) {
        if (nldmData.empty() == false) {
          std::sort(nldmData.begin(), nldmData.end(), SortArcDataByPin());
          _owner->_nldmData.insert({cellName, nldmData});
        }
        if (ccsData.empty() == false) {
          std::sort(ccsData.begin(), ccsData.end(), SortArcDataByPin());
          _owner->_ccsData.insert({cellName, ccsData});
        }
        nldmData.clear();
        ccsData.clear();
        cellName.clear();
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
      } else if (strs[0] == ".DRIVWAVE") {
        for (size_t i=0; i<2; ++i) {
          std::getline(infile, line);
          line = trim(line);
          if (line == "Rise") {
            readNLDMLUT(infile, _owner->_riseDriverWaveform, timeUnit, voltageUnit, voltageUnit);
          } else if (line == "Fall") {
            readNLDMLUT(infile, _owner->_fallDriverWaveform, timeUnit, voltageUnit, voltageUnit);
          }
        }
      } else {
        cellName = strs[0];
        std::vector<FixedLoadCap> pinCaps;
        for (size_t i=1; i<strs.size(); ++i) {
          FixedLoadCap cap;
          cap.setPinName(strs[i]);
          cap.setCaps(std::stod(strs[i+1])*capUnit, std::stod(strs[i+2])*capUnit);
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
        if (nldmArc.empty() == false) nldmData.push_back(nldmArc);
        if (ccsArc.empty() == false) ccsData.push_back(ccsArc);
        nldmArc.reset();
        ccsArc.reset();
        fromPin.clear();
        toPin.clear();
      }
      getArcInfo(line, fromPin, toPin, isInverted);
      nldmArc.setFromToPin(fromPin, toPin, isInverted);
      ccsArc.setFromToPin(fromPin, toPin, isInverted);
    } else if (numSpace == 4) {
      if (line == "Rise Delay") {
        readNLDMLUT(infile, nldmArc.getLUT(LUTType::RiseDelay), timeUnit, capUnit, timeUnit);
      } else if (line == "Fall Delay") {
        readNLDMLUT(infile, nldmArc.getLUT(LUTType::FallDelay), timeUnit, capUnit, timeUnit);
      } else if (line == "Rise Transition") {
        readNLDMLUT(infile, nldmArc.getLUT(LUTType::RiseTransition), timeUnit, capUnit, timeUnit);
      } else if (line == "Fall Transition") {
        readNLDMLUT(infile, nldmArc.getLUT(LUTType::FallTransition), timeUnit, capUnit, timeUnit);
      } else if (line == "DC Current") {
        std::string capLine;
        std::getline(infile, capLine);
        capLine = trim(capLine);
        const std::vector<double> caps = parseLineNumbers(capLine, capUnit);
        ccsArc.setMillerCaps(caps[0], caps[1]);
        readNLDMLUT(infile, ccsArc.getDCCurrent(), voltageUnit, voltageUnit, currentUnit);
      } else if (line == "Current Rise") {
        std::string numLine;
        std::getline(infile, numLine);
        numLine = trim(numLine);
        size_t tableCount = std::stoi(numLine);
        CCSGroup& riseCurrents = ccsArc.getCurrent(LUTType::RiseCurrent);
        for (size_t i=0; i<tableCount; ++i) {
          CCSLUT lut;
          readCCSLUT(infile, lut, timeUnit, timeUnit, capUnit, timeUnit, currentUnit);
          riseCurrents.addLUT(lut);
        }
        riseCurrents.sortTable();
      } else if (line == "Current Fall") {
        std::string numLine;
        std::getline(infile, numLine);
        numLine = trim(numLine);
        size_t tableCount = std::stoi(numLine);
        CCSGroup& fallCurrents = ccsArc.getCurrent(LUTType::FallCurrent);
        for (size_t i=0; i<tableCount; ++i) {
          CCSLUT lut;
          readCCSLUT(infile, lut, timeUnit, timeUnit, capUnit, timeUnit, currentUnit);
          fallCurrents.addLUT(lut);
        }
        fallCurrents.sortTable();
      } else if (line == "Receiver Cap Rise") {
        readNLDMLUT(infile, ccsArc.getRecvCap(LUTType::RiseRecvCap), timeUnit, capUnit, timeUnit);
      } else if (line == "Receiver Cap Fall") {
        readNLDMLUT(infile, ccsArc.getRecvCap(LUTType::FallRecvCap), timeUnit, capUnit, timeUnit);
      }
    }
  }
  if (fromPin.size() > 0 || toPin.size() > 0) {
    if (nldmArc.empty() == false) nldmData.push_back(nldmArc);
    if (ccsArc.empty() == false) ccsData.push_back(ccsArc);
  }
  if (cellName.size() > 0) {
    if (nldmData.empty() == false) {
      std::sort(nldmData.begin(), nldmData.end(), SortArcDataByPin());
      _owner->_nldmData.insert({cellName, nldmData});
    }
    if (ccsData.empty() == false) {
      std::sort(ccsData.begin(), ccsData.end(), SortArcDataByPin());
      _owner->_ccsData.insert({cellName, ccsData});
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
  NLDMArc tempArc(nullptr);
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
  CCSArc tempArc(nullptr);
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
  /// We don't allow missing lib data
  assert(it != _loadCaps.end());
  const auto& pinCaps = it->second;
  const auto& it2 = std::lower_bound(pinCaps.begin(), pinCaps.end(), pinStr, 
                      [](const FixedLoadCap& a, const std::string& b) {
                        return a.pinName() < b;
                      });

  if (it2 != pinCaps.end() && it2->pinName() == pinStr) {
    return false;
  }
  return true;
}

const NLDMArc*
LibData::findNLDMArc(const std::string& cell, const std::string& fromPin, const std::string& toPin) const
{
  const auto& it = _nldmData.find(cell);
  if (it == _nldmData.end()) {
    return nullptr;
  } 
  const auto& arcs = it->second;
  NLDMArc tempArc(nullptr);
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
  CCSArc tempArc(nullptr);
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
  /// We don't allow missing lib data
  assert(it != _loadCaps.end());
  const auto& pinCaps = it->second;
  const auto& it2 = std::lower_bound(pinCaps.begin(), pinCaps.end(), pin, 
                      [](const FixedLoadCap& a, const std::string& b) {
                        return a.pinName() < b;
                      });

  if (it2 != pinCaps.end() && it2->pinName() == pin) {
    return false;
  }
  return true;
}

double
LibData::fixedLoadCap(const std::string& cell, const std::string& pin, bool isRise) const
{
  const auto& it = _loadCaps.find(cell);
  /// We don't allow missing lib data
  assert(it != _loadCaps.end());
  const auto& pinCaps = it->second;
  const auto& it2 = std::lower_bound(pinCaps.begin(), pinCaps.end(), pin, 
                      [](const FixedLoadCap& a, const std::string& b) {
                        return a.pinName() < b;
                      });

  if (it2 != pinCaps.end() && it2->pinName() == pin) {
    const FixedLoadCap& cap = *it2;
    return cap.value(isRise);
  }
  return 0;
}

std::vector<std::string>
LibData::cellArcInputPins(const std::string& cell, const std::string& outPin) const
{
  std::vector<std::string> inputPins;
  const auto& it = _nldmData.find(cell);
  if (it == _nldmData.end()) {
    return inputPins;
  }
  const std::vector<NLDMArc>& nldmArcs = it->second;
  const auto& iters = std::equal_range(nldmArcs.begin(), nldmArcs.end(), outPin, SortArcDataByPin{});
  for (auto it2 = iters.first; it2 != iters.second; ++it2) {
    inputPins.push_back(it2->fromPin());
  }
  return inputPins;
}

std::vector<std::string>
LibData::cellArcOutputPins(const std::string& cell, const std::string& inPin) const
{
  std::vector<std::string> outputPins;
  const auto& it = _nldmData.find(cell);
  if (it == _nldmData.end()) {
    return outputPins;
  }
  const std::vector<NLDMArc>& nldmArcs = it->second;
  for (const NLDMArc& arc : nldmArcs) {
    if (arc.fromPin() == inPin) {
      outputPins.push_back(arc.toPin());
    }
  }
  return outputPins;
}

}
