#include <fstream>
#include <unordered_map>
#include <cstring>
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

static inline bool
isLineClosed(std::string& line, size_t& parenCounter, std::string& str)
{
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
  }
  str.append(" ");
  str.append(line);
  return parenCounter == 0;
}


NetlistParser::NetlistParser(const char* fileName) 
{
  std::ifstream infile(fileName);
  if (!infile) {
    printf("ERROR: Cannot open %s\n", fileName);
    return;
  }
  std::unordered_map<std::string, size_t> nodeMap;
  std::string line;
  std::string content;
  size_t parenCounter = 0;
  while (std::getline(infile, line)) {
    while (isLineClosed(line, parenCounter, content) == false) {
      if (!std::getline(infile, line)) {
        break;
      }
    }
    parseLine(content, nodeMap);
    content.clear();
  }
  nodeMapToList(nodeMap, _nodes);

  std::vector<size_t> devCounter(static_cast<unsigned char>(DeviceType::Total), 0);
  for (const Device& dev : _devices) {
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
      if (((i - startIndex - 1) & 0x1) == 0) {
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
                 std::vector<Device>& devices, 
                 const char* units,
                 std::unordered_map<std::string, size_t>& nodeMap)
{
  Device dev;
  dev._type = type;
  dev._name.assign(strs[0].begin() + 1, strs[0].end());
  dev._posNode = findOrCreateNode(nodeMap, strs[1]); 
  dev._negNode = findOrCreateNode(nodeMap, strs[2]); 
  dev._value = numericalValue(strs[3], units);
  devices.push_back(dev);
}

static void 
addResistor(const std::string& line, std::vector<Device>& devices, 
            std::unordered_map<std::string, size_t>& nodeMap)
{
  std::vector<std::string> strs;
  splitWithAny(line, " \t\r", strs);
  if (strs.size() < 4) {
    printf("Unsupported syntax %s\n", line.data());
    return;
  }
  addTwoTermDevice(DeviceType::Resistor, strs, devices, "", nodeMap);
}

static void 
addCapacitor(const std::string& line, std::vector<Device>& devices, 
             std::unordered_map<std::string, size_t>& nodeMap)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() < 4) {
    printf("Unsupported syntax %s\n", line.data());
    return;
  }
  addTwoTermDevice(DeviceType::Capacitor, strs, devices, "", nodeMap);
}

static void 
addInductor(const std::string& line, std::vector<Device>& devices, 
            std::unordered_map<std::string, size_t>& nodeMap)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() < 4) {
    printf("Unsupported syntax %s\n", line.data());
    return;
  }
  addTwoTermDevice(DeviceType::Inductor, strs, devices, "hH", nodeMap);
}

void 
addIndependentSource(DeviceType type, const std::string& line, 
                     std::vector<Device>& devices, 
                     std::vector<PWLValue>& PWLData, 
                     std::unordered_map<std::string, size_t>& nodeMap)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() == 4) {
    addTwoTermDevice(type, strs, devices, "VvAa", nodeMap);
  } else if (strs.size() > 4 && 
            (strncmp(strs[3].data(), "PWL", 3) == 0 || 
             strncmp(strs[3].data(), "pwl", 3) == 0)) {
    
    const PWLValue& pwlData = parsePWLData(strs, 3);
    Device dev;
    dev._type = type;
    dev._name.assign(strs[0].begin() + 1, strs[0].end());
    dev._posNode = findOrCreateNode(nodeMap, strs[1]); 
    dev._negNode = findOrCreateNode(nodeMap, strs[2]); 
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
                 std::vector<Device>& devices, 
                 std::vector<PWLValue>& PWLData, 
                 std::unordered_map<std::string, size_t>& nodeMap)
{
  addIndependentSource(DeviceType::VoltageSource, line, devices, PWLData, nodeMap);
}

static void 
addCurrentSource(const std::string& line, 
                 std::vector<Device>& devices, 
                 std::vector<PWLValue>& PWLData, 
                 std::unordered_map<std::string, size_t>& nodeMap)
{
  addIndependentSource(DeviceType::CurrentSource, line, devices, PWLData, nodeMap);
}

static void
addDependentDevice(DeviceType type,
                   std::vector<std::string>& strs,
                   std::vector<Device>& devices, 
                   std::unordered_map<std::string, size_t>& nodeMap)
{
  Device dev;
  dev._name.assign(strs[0].begin() + 1, strs[0].end());
  dev._posNode = findOrCreateNode(nodeMap, strs[1]); 
  dev._negNode = findOrCreateNode(nodeMap, strs[2]); 
  size_t posSampleNode = findNode(nodeMap, strs[3]);
  size_t negSampleNode = findNode(nodeMap, strs[4]);
  if (posSampleNode == static_cast<size_t>(-1)) {
    printf("Referenced sampling node %s in device %s does not exist\n", strs[3].data(), strs[0].data());
    return;
  }
  if (negSampleNode == static_cast<size_t>(-1)) {
    printf("Referenced sampling node %s in device %s does not exist\n", strs[4].data(), strs[0].data());
    return;
  }
  dev._value = numericalValue(strs[5], "");
  devices.push_back(dev);
}

static void 
addVCVS(const std::string& line, 
        std::vector<Device>& devices, 
        std::vector<PWLValue>& PWLData, 
        std::unordered_map<std::string, size_t>& nodeMap)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() != 6) {
    printf("Unsupported syntax line\"%s\"\n", line.data());
    return;
  }
  addDependentDevice(DeviceType::VCVS, strs, devices, nodeMap);
}

static void 
addVCCS(const std::string& line, 
        std::vector<Device>& devices, 
        std::vector<PWLValue>& PWLData, 
        std::unordered_map<std::string, size_t>& nodeMap)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() != 6) {
    printf("Unsupported syntax line\"%s\"\n", line.data());
    return;
  }
  addDependentDevice(DeviceType::VCCS, strs, devices, nodeMap);
}

static void 
addCCVS(const std::string& line, 
        std::vector<Device>& devices, 
        std::vector<PWLValue>& PWLData, 
        std::unordered_map<std::string, size_t>& nodeMap)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() != 6) {
    printf("Unsupported syntax line\"%s\"\n", line.data());
    return;
  }
  addDependentDevice(DeviceType::CCVS, strs, devices, nodeMap);
}

static void 
addCCCS(const std::string& line, 
        std::vector<Device>& devices, 
        std::vector<PWLValue>& PWLData, 
        std::unordered_map<std::string, size_t>& nodeMap)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() != 6) {
    printf("Unsupported syntax line\"%s\"\n", line.data());
    return;
  }
  addDependentDevice(DeviceType::CCCS, strs, devices, nodeMap);
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
      } else {
        printf("Integrate method \"%s\" is not supported, using default gear2\n", strs[i].data());
      }
    } else {
      printf("option \"%s\" is not supported and ignored\n", strs[i].data());
    }
  }
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
    Debug::setLevel(numericalValue(strs[1], ""));
  } else if (strs[0] == ".end") {
  } else {
    printf("command line %s is ignored\n", line.data());
  }
}

void
NetlistParser::parseLine(const std::string& line, 
                         std::unordered_map<std::string, size_t>& nodeMap)
{
  char c = firstChar(line);
  switch (c) {
    case 'R':
    case 'r':
      addResistor(line, _devices, nodeMap);
      break;
    case 'C':
    case 'c':
      addCapacitor(line, _devices, nodeMap);
      break;
    case 'L':
    case 'l':
      addInductor(line, _devices, nodeMap);
      break;
    case 'V':
    case 'v':
      addVoltageSource(line, _devices, _PWLData, nodeMap);
      break;
    case 'I':
    case 'i':
      addCurrentSource(line, _devices, _PWLData, nodeMap);
      break;
    case 'E':
    case 'e':
      addVCVS(line, _devices, _PWLData, nodeMap);
      break;
    case 'F':
    case 'f':
      addCCCS(line, _devices, _PWLData, nodeMap);
      break;
    case 'G':
    case 'g':
      addVCCS(line, _devices, _PWLData, nodeMap);
      break;
    case 'H':
    case 'h':
      addCCVS(line, _devices, _PWLData, nodeMap);
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
