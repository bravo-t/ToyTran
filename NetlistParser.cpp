#include <fstream>
#include <unordered_map>
#include <cstring>
#include <cctype>  
#include <algorithm> 
#include "NetlistParser.h"
#include "Debug.h"

namespace Tran {

void
nodeMapToList(const std::unordered_map<std::string, size_t>& nodeMap,
              std::vector<std::string>& nodes)
{
  nodes.resize(nodeMap.size(), "");
  for (const auto& kv : nodeMap) {
    nodes[kv.second] = kv.first;
  }
}

char 
firstChar(const std::string& line)
{
  for (size_t i=0; i<line.size(); ++i) {
    if (std::isspace(line[i]) == false) {
      return line[i];
    }
  }
  return '\0';
}

static inline bool
isLineClosed(std::string& line, size_t& parenCounter, std::string& str)
{
  bool tailingPlus = false;
  for (size_t i=0; i<line.size(); ++i) {
    char& c = line[i];
    if (c == '(') {
      ++parenCounter;
    } else if (c == ')') {
      --parenCounter;
    }
    if (iscntrl(c)) {
      c = ' ';
    }
    if (c == '+') {
      tailingPlus = true;
    }
    if (tailingPlus && std::isalnum(c) && c != '+') {
      tailingPlus = false;
    }
  }
  if (tailingPlus) {
    while (line.back() != '+') {
      line.pop_back();
    }
    line.pop_back();
  }
  size_t startOffset = static_cast<size_t>(-1);
  if (firstChar(line) == '+') {
    for (size_t i=0; i<line.size(); ++i) {
      if (line[i] == '+') {
        startOffset = i;
        break;
      }
    }
  }
  str.append(" ");
  str.insert(str.end(), line.begin()+startOffset+1, line.end());
  return parenCounter == 0 && tailingPlus == false;
}

static bool
getClosedLine(std::ifstream& infile, std::string& content)
{
  std::string line;
  size_t parenCounter = 0;
  while (std::getline(infile, line)) {
    if (isLineClosed(line, parenCounter, content)) {
      break;
    }
  }
  while (std::getline(infile, line)) {
    if (firstChar(line) == '+') {
      size_t offset = static_cast<size_t>(-1);
      for (size_t i=0; i<line.size(); ++i) {
        if (line[i] == '+') {
          offset = i+1;
          break;
        }
      }
      content.insert(content.end(), line.begin()+offset, line.end());
    } else {
      infile.seekg(-line.size()-1, std::ios_base::cur);
      break;
    }
  }
  return infile.peek() != EOF;
}

NetlistParser::NetlistParser(const char* fileName) 
{
  std::ifstream infile(fileName);
  if (!infile) {
    printf("ERROR: Cannot open %s\n", fileName);
    return;
  }
  std::string line;
  std::string content;
  while (getClosedLine(infile, content)) {
    parseLine(content);
    content.clear();
  }

  std::vector<size_t> devCounter(static_cast<unsigned char>(DeviceType::Total), 0);
  for (const ParserDevice& dev : _devices) {
    ++devCounter[static_cast<unsigned char>(dev._type)];
  }
  printf("Netlist file %s loaded, devices created:\n"
         "  %lu resistors\n"
         "  %lu capacitors\n"
         "  %lu inductors\n"
         "  %lu independent voltage sources\n"
         "  %lu independent current sources\n"
         "  %lu VCCS\n"
         "  %lu VCVS\n"
         "  %lu CCCS\n"
         "  %lu CCVS\n", 
    fileName, 
    devCounter[static_cast<unsigned char>(DeviceType::Resistor)],
    devCounter[static_cast<unsigned char>(DeviceType::Capacitor)], 
    devCounter[static_cast<unsigned char>(DeviceType::Inductor)], 
    devCounter[static_cast<unsigned char>(DeviceType::VoltageSource)], 
    devCounter[static_cast<unsigned char>(DeviceType::CurrentSource)], 
    devCounter[static_cast<unsigned char>(DeviceType::VCCS)], 
    devCounter[static_cast<unsigned char>(DeviceType::VCVS)],
    devCounter[static_cast<unsigned char>(DeviceType::CCCS)], 
    devCounter[static_cast<unsigned char>(DeviceType::CCVS)]);

}

static inline void
splitWithAny(const std::string& src, const char *delim,
             std::vector<std::string>& strs)
{
  char *data = strdup(src.data());
  strs.clear();
  char *saveptr(nullptr);
  if (char* res1 = strtok_r(data, delim, &saveptr)) {
    strs.push_back(res1);
    while(char *res2 = strtok_r(nullptr, delim, &saveptr)) {
      std::string tempStr(res2);
      size_t eraseLength = 0;
      for (size_t i=0; i<tempStr.size(); ++i) {
        if (isspace(tempStr[i]) || iscntrl(tempStr[i])) {
          ++eraseLength;
        } else {
          break;
        }
      }
      if (eraseLength > 0) {
        tempStr.erase(0, eraseLength);
      }
      if (tempStr.empty()) {
        continue;
      }
      char c = tempStr.back();
      while (isspace(c) || iscntrl(c)) {
        tempStr.pop_back();
        if (tempStr.empty()) {
          break;
        }
        c = tempStr.back();
      }
      if (tempStr.empty() == false) {
        strs.push_back(std::string(tempStr));
      } 
    }
  }
  ::free(data);
}

static inline double 
findUnit(std::string& str, const char* ignoreChars)
{
  size_t lastIndex = str.size() - 1;
  size_t actualLastIndex = lastIndex;
  char lastChar = str[lastIndex];
  size_t ignoreLength = strlen(ignoreChars);
  for (size_t i=0; i<ignoreLength; ++i) {
    if (lastChar == ignoreChars[i]) {
      actualLastIndex -= 1;
      lastChar = str[actualLastIndex];
      break;
    }
  }
  char unitStartIndex = actualLastIndex;
  double scale = 1;
  switch (lastChar) {
    case 'F':
    case 'f':
      scale = 1e-15;
      break;
    case 'P':
    case 'p':
      scale = 1e-12;
      break;
    case 'N':
    case 'n':
      scale = 1e-9;
      break;
    case 'U':
    case 'u':
      scale = 1e-6;
      break;
    case 'M':
    case 'm':
      scale = 1e-3;
      break;
    case 'K':
    case 'k':
      scale = 1e3;
      break;
    case 'X':
    case 'x':
      scale = 1e6;
      break;
    case 'T':
    case 't':
      scale = 1e12;
      break;
    case 'G':
    case 'g':
      if (str.size() > 3 && 
         (str[actualLastIndex-2] == 'M' || str[actualLastIndex-2] == 'm') && 
         (str[actualLastIndex-1] == 'E' || str[actualLastIndex-1] == 'e')) {
        scale = 1e6;
        unitStartIndex = actualLastIndex - 2;   
      } else {
        scale = 1e9;
      }
      break;
    default:
      scale = 1;
      unitStartIndex = lastIndex + 1;
  }
  for (size_t i=unitStartIndex; i<=lastIndex; ++i) {
    str[i] = ' ';
  }
  return scale;
}

static inline double 
numericalValue(std::string& str, const char* ignoreChars)
{
  double scale = findUnit(str, ignoreChars);
  return strtod(str.data(), nullptr) * scale;
}

static PWLValue
parsePWLData(std::vector<std::string>& strs, size_t startIndex)
{
  PWLValue pwlData;
  for (size_t i=startIndex; i<strs.size(); ++i) {
    std::string& str = strs[i];
    if (i == startIndex) {
      if (strncmp(str.data(), "PWL(", 3) == 0 || 
          strncmp(str.data(), "pwl", 3) == 0) {
        str.erase(0, 3);
      }
    }
    if (str[0] == '(') {
      str.erase(0, 1);
    } 
    if (str.back() == ')') {
      str.pop_back();
    }
    if (str.size() > 0) {
      if (((i - startIndex) & 0x1) == 0) {
        double value = numericalValue(str, "Ss");
        pwlData._time.push_back(value);
      } else {
        double value = numericalValue(str, "VvAa");
        pwlData._value.push_back(value);
      }
    }
  }
  assert(pwlData._time.size() == pwlData._value.size() &&
        "Imbalanced PWL data\n");
  return pwlData;
}

/*
struct PWLValue
pulseToPWLValue(double initValue, double puleValue, double delay, 
                double riseTime, double fallTime, double width, double period)
{
  
}
*/

static inline size_t 
findOrCreateNode(std::unordered_map<std::string, size_t>& nodeMap, 
                 const std::string& nodeName)
{
  auto found = nodeMap.find(nodeName);
  if (found == nodeMap.end()) {
    size_t id = nodeMap.size();
    nodeMap.insert({nodeName, id});
    return id;
  } else {
    return found->second;
  }
}

static inline size_t 
findNode(const std::unordered_map<std::string, size_t>& nodeMap, 
         const std::string& nodeName)
{
  auto found = nodeMap.find(nodeName);
  if (found == nodeMap.end()) {
    return static_cast<size_t>(-1);
  } else {
    return found->second;
  }
}

static void
addTwoTermDevice(DeviceType type, 
                 std::vector<std::string>& strs,  
                 std::vector<ParserDevice>& devices, 
                 const char* units)
{
  ParserDevice dev;
  dev._type = type;
  dev._name.assign(strs[0].begin() + 1, strs[0].end());
  dev._posNode = strs[1]; 
  dev._negNode = strs[2]; 
  dev._value = numericalValue(strs[3], units);
  devices.push_back(dev);
}

static void 
addResistor(const std::string& line, std::vector<ParserDevice>& devices)
{
  std::vector<std::string> strs;
  splitWithAny(line, " \t\r", strs);
  if (strs.size() < 4) {
    printf("Unsupported syntax %s\n", line.data());
    return;
  }
  addTwoTermDevice(DeviceType::Resistor, strs, devices, "");
}

static void 
addCapacitor(const std::string& line, std::vector<ParserDevice>& devices)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() < 4) {
    printf("Unsupported syntax %s\n", line.data());
    return;
  }
  addTwoTermDevice(DeviceType::Capacitor, strs, devices, "");
}

static void 
addInductor(const std::string& line, std::vector<ParserDevice>& devices)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() < 4) {
    printf("Unsupported syntax %s\n", line.data());
    return;
  }
  addTwoTermDevice(DeviceType::Inductor, strs, devices, "hH");
}

void 
addIndependentSource(DeviceType type, const std::string& line, 
                     std::vector<ParserDevice>& devices, 
                     std::vector<PWLValue>& PWLData)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() == 4) {
    addTwoTermDevice(type, strs, devices, "VvAa");
  } else if (strs.size() > 4 && 
            (strncmp(strs[3].data(), "PWL", 3) == 0 || 
             strncmp(strs[3].data(), "pwl", 3) == 0)) {
    
    const PWLValue& pwlData = parsePWLData(strs, 3);
    ParserDevice dev;
    dev._type = type;
    dev._name.assign(strs[0].begin() + 1, strs[0].end());
    dev._posNode = strs[1]; 
    dev._negNode = strs[2]; 
    dev._isPWLValue = true;
    dev._PWLData = PWLData.size();
    devices.push_back(dev);
    PWLData.push_back(pwlData);
  } else {
    printf("Unsupported syntax %s\n", line.data());
  }
}

static void 
addVoltageSource(const std::string& line, 
                 std::vector<ParserDevice>& devices, 
                 std::vector<PWLValue>& PWLData)
{
  addIndependentSource(DeviceType::VoltageSource, line, devices, PWLData);
}

static void 
addCurrentSource(const std::string& line, 
                 std::vector<ParserDevice>& devices, 
                 std::vector<PWLValue>& PWLData)
{
  addIndependentSource(DeviceType::CurrentSource, line, devices, PWLData);
}

static void
addDependentDevice(DeviceType type,
                   std::vector<std::string>& strs,
                   std::vector<ParserDevice>& devices)
{
  ParserDevice dev;
  dev._type = type;
  dev._name.assign(strs[0].begin() + 1, strs[0].end());
  dev._posNode = strs[1]; 
  dev._negNode = strs[2];
  dev._posSampleNode = strs[3];
  dev._negSampleNode = strs[4]; 
  dev._value = numericalValue(strs[5], "");
  devices.push_back(dev);
}

static void 
addVCVS(const std::string& line, 
        std::vector<ParserDevice>& devices, 
        std::vector<PWLValue>& /*PWLData*/)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() != 6) {
    printf("Unsupported syntax line\"%s\"\n", line.data());
    return;
  }
  addDependentDevice(DeviceType::VCVS, strs, devices);
}

static void 
addVCCS(const std::string& line, 
        std::vector<ParserDevice>& devices, 
        std::vector<PWLValue>& /*PWLData*/)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() != 6) {
    printf("Unsupported syntax line\"%s\"\n", line.data());
    return;
  }
  addDependentDevice(DeviceType::VCCS, strs, devices);
}

static void 
addCCVS(const std::string& line, 
        std::vector<ParserDevice>& devices, 
        std::vector<PWLValue>& /*PWLData*/) 
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() != 6) {
    printf("Unsupported syntax line\"%s\"\n", line.data());
    return;
  }
  addDependentDevice(DeviceType::CCVS, strs, devices);
}

static void 
addCCCS(const std::string& line, 
        std::vector<ParserDevice>& devices, 
        std::vector<PWLValue>& /*PWLData*/)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() != 6) {
    printf("Unsupported syntax line\"%s\"\n", line.data());
    return;
  }
  addDependentDevice(DeviceType::CCCS, strs, devices);
}

void
NetlistParser::processOption(const std::string& line) 
{
  std::vector<std::string> strs;
  splitWithAny(line, " =", strs);
  /// strs[0] = ".option", discard
  for (size_t i=1; i<strs.size(); ++i) {
    if (strs[i].compare("method") == 0) {
      ++i;
      if (strs[i].compare("gear2") == 0) {
        _intMethod = IntegrateMethod::Gear2;
      } else if (strs[i].compare("euler") == 0) {
        _intMethod = IntegrateMethod::BackwardEuler;
      } else if (strs[i].compare("trap") == 0) {
        _intMethod = IntegrateMethod::Trapezoidal;
      } else {
        printf("Integrate method \"%s\" is not supported, using default gear2\n", strs[i].data());
      }
    } else if (strs[i].compare("post") == 0) {
      ++i;
      if (strs[i].compare("2") == 0) {
        _saveData = true;
      } else {
        printf("Value provided to post is not supported and ignored\n");
      }
    } else {
      printf("option \"%s\" is not supported and ignored\n", strs[i].data());
    }
  }
}

char 
asciitolower(char in) 
{
  if (in <= 'Z' && in >= 'A')
    return in - ('Z' - 'z');
  return in;
}

void 
toLower(std::string& line) 
{
  std::transform(line.begin(), line.end(), line.begin(), asciitolower);
}

/// returns true if given string is like "(XXX)" or "xxx"
/// returns false if given string is like "(XXX" or "XXX)"
bool
findNameInParenthesis(std::string& str, size_t& startIndex, size_t& endIndex)
{
  bool foundOpen = false;
  bool foundClose = false;
  endIndex = 0;
  startIndex = str.size();
  for (size_t i=0; i<str.size(); ++i) {
    if (str[i] == '(') {
      startIndex = i;
      foundOpen = true;
      continue;
    }
    if (startIndex != str.size() && str[i] == ')') {
      endIndex = i;
      foundClose = true;
      break;
    }
  }
  return foundOpen == foundClose;
}

PlotData*
findPlotData(const std::string& canvas, std::vector<PlotData>& plotData)
{
  for (PlotData& data : plotData) {
    if (data._canvasName == canvas) {
      return &data;
    }
  }
  PlotData data;
  data._canvasName = canvas;
  plotData.push_back(data);
  return &(plotData.back());
}

void 
processPlot(const std::string& line, 
            std::vector<PlotData>& plotCmds,
            size_t& plotWidth, size_t& plotHeight)
{
  std::vector<std::string> strs;
  splitWithAny(line, " =", strs);
  /// strs[0] == .plot, discard
  toLower(strs[1]);
  if (strs[1].compare("tran") != 0) {
    printf("Only tran mode is supported in .plot command\n");
    return;
  }
  for (size_t i=2; i<strs.size(); ++i) {
    std::vector<std::string> substr;
    splitWithAny(strs[i], ".", substr);
    std::string canvas = "";
    std::string restStr;
    if (substr.size() == 1) {
      restStr = substr[0];
    } else {
      canvas = substr[0];
      restStr = substr[1];
    }
    PlotData* plotData = findPlotData(canvas, plotCmds);
    if (plotData->_canvasName != "" && 
        plotData->_nodeToPlot.size() + plotData->_deviceToPlot.size() > 3) {
      printf("ERROR: At most 4 plots can be put into 1 canvas. %s already has 4\n", canvas.data());
      continue;
    }
    char c = firstChar(restStr);
    std::vector<std::string>* destVec = nullptr;
    if (c == 'V' || c == 'v') {
      destVec = &(plotData->_nodeToPlot);
    } else if (c == 'I' || c == 'i') {
      destVec = &(plotData->_deviceToPlot);
    } else {
      toLower(restStr);
      if (restStr.compare("width") == 0) {
        ++i;
        plotWidth = numericalValue(strs[i], "");
        continue;
      } else if (restStr.compare("height") == 0) {
        ++i;
        plotHeight = numericalValue(strs[i], "");
        continue;
      } else {
        printf("Unsupported type of metric %c\n", c);
        continue;
      }
    }
    size_t startIndex, endIndex;
    if (findNameInParenthesis(restStr, startIndex, endIndex) == false || 
        startIndex == restStr.size() || endIndex == 0) {
      printf("Unsupported syntax in line \"%s\"", line.data());
      return;
    }
    std::string str = restStr.substr(startIndex + 1, endIndex - startIndex - 1);
    destVec->push_back(str);
  }
}

bool ichar_equals(char a, char b)
{
  return std::tolower(static_cast<unsigned char>(a)) ==
         std::tolower(static_cast<unsigned char>(b));
}

bool iequals(const std::string& a, const std::string& b)
{
  return a.size() == b.size() &&
         std::equal(a.begin(), a.end(), b.begin(), ichar_equals);
}

void
processMeasureCmds(const std::string& line, 
                   std::vector<MeasurePoint>& meas)
{
  std::vector<std::string> strs;
  splitWithAny(line, " =", strs);
  /// line = 
  /// .measure tran rise_delay trig V(POS)=2.5 targ V(ResOUT)=2.5
  /// .measure tran rise_tran_10_90 trig V(ResOUT)=0.5 targ V(ResOUT)=4.5
  /// strs[0] == .measure, discard
  MeasurePoint mp;
  bool inTriggerSection = false;
  bool inTargetSection = false;
  for (size_t i=1; i<strs.size(); ++i) {
    if (iequals(strs[i], "tran")) {
      continue;
    }
    if (iequals(strs[i], "trig")) {
      inTriggerSection = 1;
      inTargetSection = 0;
      continue;
    }
    if (iequals(strs[i], "targ")) {
      inTriggerSection = 0;
      inTargetSection = 1;
      continue;
    }
    if (!inTriggerSection && !inTargetSection) {
      mp._variableName = strs[i];
      continue;
    }
    if (inTriggerSection || inTargetSection) {
      if (iequals(strs[i], "td")) {
        if (inTargetSection) {
          printf("Unsupported TD statement in targ of line \"%s\"\n", line.data());
          return;
        }
        ++i;
        mp._timeDelay = numericalValue(strs[i], "Ss");
        continue;
      } else {
        char c = firstChar(strs[i]);
        SimResultType type = SimResultType::Voltage;
        if (c == 'V' || c == 'v') {
          type = SimResultType::Voltage;
        } else if (c == 'I' || c == 'i') {
          type = SimResultType::Current;
        } else {
          printf("Unsupported type of metric %c\n", c);
          return;
        }
        size_t startIndex, endIndex;
        if (findNameInParenthesis(strs[i], startIndex, endIndex) == false || 
            startIndex == strs[i].size() || endIndex == 0) {
          printf("Unsupported syntax in line \"%s\"", line.data());
          return;
        }
        if (inTargetSection) {
          mp._target = strs[i].substr(startIndex + 1, endIndex - startIndex - 1);
          mp._targetType = type;
          ++i;
          mp._targetValue = numericalValue(strs[i], "");
        } else if (inTriggerSection) {
          mp._trigger = strs[i].substr(startIndex + 1, endIndex - startIndex - 1);
          mp._triggerType = type;
          ++i;
          mp._triggerValue = numericalValue(strs[i], "");
        }
        continue;
      }
    }
  }
  meas.push_back(mp);
}

void
NetlistParser::processCommands(const std::string& line) 
{
  std::vector<std::string> strs;
  splitWithAny(line, " \t\r", strs);
  if (strs[0] == ".tran") {
    _simTick = numericalValue(strs[1], "sS");
    _simTime = numericalValue(strs[2], "sS");
  } else if (strs[0] == ".debug") {
    Debug::setLevel(numericalValue(strs[1], ""));
  } else if (strs[0] == ".option") {
    processOption(line);
  } else if (strs[0] == ".plot") {
    processPlot(line, _plotData, _plotWidth, _plotHeight);
  } else if (strs[0] == ".measure") {
    processMeasureCmds(line, _measurePoints);
  } else if (strs[0] == ".end") {
  } else {
    printf("command line %s is ignored\n", line.data());
  }
}

void
NetlistParser::parseLine(const std::string& line)
{
  char c = firstChar(line);
  switch (c) {
    case 'R':
    case 'r':
      addResistor(line, _devices);
      break;
    case 'C':
    case 'c':
      addCapacitor(line, _devices);
      break;
    case 'L':
    case 'l':
      addInductor(line, _devices);
      break;
    case 'V':
    case 'v':
      addVoltageSource(line, _devices, _PWLData);
      break;
    case 'I':
    case 'i':
      addCurrentSource(line, _devices, _PWLData);
      break;
    case 'E':
    case 'e':
      addVCVS(line, _devices, _PWLData);
      break;
    case 'F':
    case 'f':
      addCCCS(line, _devices, _PWLData);
      break;
    case 'G':
    case 'g':
      addVCCS(line, _devices, _PWLData);
      break;
    case 'H':
    case 'h':
      addCCVS(line, _devices, _PWLData);
      break;
    case '*':
      break;
    case '.':
      processCommands(line);
      break;
    case '\0':
      break;
    default:
      printf("Ignoring line %s\n", line.data());
  }
}


}
