#include <fstream>
#include <unordered_map>
#include <cstring>
#include "NetlistParser.h"

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


NetlistParser::NetlistParser(const char* fileName) 
{
  std::ifstream infile(fileName);
  if (!infile) {
    printf("ERROR: Cannot open %s\n", fileName);
    return;
  }
  std::unordered_map<std::string, size_t> nodeMap;
  std::string line;
  while (std::getline(infile, line)) {
    parseLine(line, nodeMap);
  }
  nodeMapToList(nodeMap, _nodes);
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
      strs.push_back(std::string(res2));
    }
  }
  ::free(data);
}

static inline double 
findUnit(std::string& str)
{
  size_t lastIndex = str.size() - 1;
  char lastChar = str[lastIndex];
  char unitStartIndex = lastIndex;
  double scale = 1;
  switch (lastChar) {
    case 'F':
    case 'f':
      scale = 1e-15;
      break;
    case 'P':
    case 'p':
      scale = 1e-12;
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
         (str[lastIndex-2] == 'M' || str[lastIndex-2] == 'm') && 
         (str[lastIndex-1] == 'E' || str[lastIndex-1] == 'e')) {
        scale = 1e6;
        unitStartIndex = lastIndex - 2;   
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
numericalValue(std::string& str)
{
  double scale = findUnit(str);
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
      double value = numericalValue(str);
      if (((i - startIndex) & 0x1) == 0) {
        pwlData._time.push_back(value);
      } else {
        pwlData._value.push_back(value);
      }
    }
  }
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

static void
addTwoTermDevice(DeviceType type, 
                 std::vector<std::string>& strs,  
                 std::vector<Device>& devices, 
                 std::unordered_map<std::string, size_t>& nodeMap)
{
  Device dev;
  dev._type = type;
  dev._name.assign(strs[0].begin() + 1, strs[0].end());
  dev._posNode = findOrCreateNode(nodeMap, strs[1]); 
  dev._negNode = findOrCreateNode(nodeMap, strs[2]); 
  dev._value = numericalValue(strs[3]);
  devices.push_back(dev);
}

static void 
addResistor(const std::string& line, std::vector<Device>& devices, 
            std::unordered_map<std::string, size_t>& nodeMap)
{
  std::vector<std::string> strs;
  splitWithAny(line, " ", strs);
  if (strs.size() < 4) {
    printf("Unsupported syntax %s\n", line.data());
    return;
  }
  addTwoTermDevice(DeviceType::Resistor, strs, devices, nodeMap);
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
  addTwoTermDevice(DeviceType::Capacitor, strs, devices, nodeMap);
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
  addTwoTermDevice(DeviceType::Inductor, strs, devices, nodeMap);
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
    addTwoTermDevice(type, strs, devices, nodeMap);
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
addVCVS(const std::string& /*line*/, 
        std::vector<Device>& /*devices*/, 
        std::vector<PWLValue>& /*PWLData*/, 
        std::unordered_map<std::string, size_t>& /*nodeMap*/)
{
  printf("VCVS is not supported yet\n");
}

static void 
addVCCS(const std::string& /*line*/, 
        std::vector<Device>& /*devices*/, 
        std::vector<PWLValue>& /*PWLData*/, 
        std::unordered_map<std::string, size_t>& /*nodeMap*/)
{
  printf("VCCS is not supported yet\n");
}

static void 
addCCVS(const std::string& /*line*/, 
        std::vector<Device>& /*devices*/, 
        std::vector<PWLValue>& /*PWLData*/, 
        std::unordered_map<std::string, size_t>& /*nodeMap*/)
{
  printf("CCVS is not supported yet\n");
}

static void 
addCCCS(const std::string& /*line*/, 
        std::vector<Device>& /*devices*/, 
        std::vector<PWLValue>& /*PWLData*/, 
        std::unordered_map<std::string, size_t>& /*nodeMap*/)
{
  printf("CCCS is not supported yet\n");
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
    default:
      printf("Ignoring line %s\n", line.data());
  }
}


}
