#include <algorithm>
#include <limits>
#include <cmath>
#include "Debug.h"
#include "Circuit.h"
#include "Base.h"
#include "NetlistParser.h"

namespace NA {

static size_t 
findDeviceId(size_t nodeId1, size_t nodeId2, 
             const std::vector<Node>& nodes)
{
  const Node& node1 = nodes[nodeId1];
  const Node& node2 = nodes[nodeId2];

  std::vector<size_t> devs1 = node1._connection;
  std::sort(devs1.begin(), devs1.end());
  std::vector<size_t> devs2 = node2._connection;
  std::sort(devs2.begin(), devs2.end());
  for (size_t dev1 : devs1) {
    for (size_t dev2 : devs2) {
      if (dev1 == dev2) {
        return dev1;
      }
    }
  }
  return static_cast<size_t>(-1);
}

typedef std::unordered_map<std::string, size_t> StringIdMap;

static void 
incrCountMap(StringIdMap& countMap, const std::string& key)
{
  auto it = countMap.find(key);
  if (it == countMap.end()) {
    countMap.insert({key, 1});
  } else {
    it->second += 1;
  }
}

static void
addInternalPosNodeForGate(StringIdMap& countMap, const ParserDevice& dev, const LibData& libData)
{
  const std::string& libCell = dev._libCellName;
  const auto& pinMap = dev._pinMap;
  for (const auto& kv : pinMap) {
    const std::string& pinName = kv.first;
    const std::string& nodeName = kv.second;
    incrCountMap(countMap, nodeName);
    if (libData.isOutputPin(libCell, pinName)) {
      std::string internalNode = dev._name + "/" + pinName + "/VPOS";
      /// this internal node connects to voltage source and the resistor, 
      /// so increment it twice
      incrCountMap(countMap, internalNode);
      incrCountMap(countMap, internalNode);
      if (Debug::enabled()) {
        printf("Created internal node %s\n", internalNode.data());
      }
    }
  }
}

std::string
Circuit::allNodes(const std::vector<ParserDevice>& devs, 
                  std::vector<std::string>& allNodeNames)
{
  bool addInternalVPosNode = (_param._type == AnalysisType::FD && 
                              _param._driverModel == DriverModel::RampVoltage);
  StringIdMap nodeConnectionCount;
  for (const ParserDevice& dev : devs) {
    if (dev._type == DeviceType::Cell) {
      if (addInternalVPosNode) {
        addInternalPosNodeForGate(nodeConnectionCount, dev, _libData);
      }
    } else {
      incrCountMap(nodeConnectionCount, dev._posNode);
      incrCountMap(nodeConnectionCount, dev._negNode);
    }
  }
  size_t maxCount = 0;
  std::string maxNode;
  for (const auto& kv : nodeConnectionCount) {
    if (kv.second > maxCount || 
       (kv.second == maxCount && kv.first.compare(maxNode) < 0)) {
      maxCount = kv.second;
      maxNode = kv.first;
    }
    allNodeNames.push_back(kv.first);
  }
  return maxNode;
}

static size_t
findNodeByName(const StringIdMap& nodeIdMap, 
               const std::string& name)
{
  const auto& it = nodeIdMap.find(name);
  if (it == nodeIdMap.end()) {
    return static_cast<size_t>(-1);
  }
  return it->second;
}

bool
createDevice(Device& dev, const ParserDevice& pDev, const StringIdMap& nodeIdMap)
{
  size_t posNode = findNodeByName(nodeIdMap, pDev._posNode);
  size_t negNode = findNodeByName(nodeIdMap, pDev._negNode);
  if (posNode == static_cast<size_t>(-1)) {
    printf("Cannot find node \"%s\" referenced by device %s\n", pDev._posNode.data(), pDev._name.data());
  }
  if (negNode == static_cast<size_t>(-1)) {
    printf("Cannot find node \"%s\" referenced by device %s\n", pDev._negNode.data(), pDev._name.data());
  }
  if (posNode == static_cast<size_t>(-1) || negNode == static_cast<size_t>(-1)) {
    return false;
  }

  dev._posNode = posNode;
  dev._negNode = negNode;
  
  if (pDev._type == DeviceType::CCCS || pDev._type == DeviceType::CCVS || 
      pDev._type == DeviceType::VCCS || pDev._type == DeviceType::VCVS) {
    size_t posSampleNode = findNodeByName(nodeIdMap, pDev._posSampleNode);
    size_t negSampleNode = findNodeByName(nodeIdMap, pDev._negSampleNode);
    if (posSampleNode == static_cast<size_t>(-1)) {
      printf("Cannot find node \"%s\" referenced by device %s\n", pDev._posNode.data(), pDev._name.data());
    }
    if (negSampleNode == static_cast<size_t>(-1)) {
      printf("Cannot find node \"%s\" referenced by device %s\n", pDev._posNode.data(), pDev._name.data());
    }
    if (posSampleNode == static_cast<size_t>(-1) || negSampleNode == static_cast<size_t>(-1)) {
      return false;
    }
    dev._posSampleNode = posSampleNode;
    dev._negSampleNode = negSampleNode;
  }
  dev._name = pDev._name;
  dev._type = pDev._type;
  dev._isPWLValue = pDev._isPWLValue;
  if (dev._isPWLValue) {
    dev._PWLData = pDev._PWLData;
  } else {
    dev._value = pDev._value;
  }
  return true;
}

std::vector<Device>
Circuit::elaborateDevice(const ParserDevice& dev, const StringIdMap& nodeIdMap)
{
  std::vector<Device> devs;
  const std::string& libCell = dev._libCellName;
  const auto& pinMap = dev._pinMap;
  for (const auto& kv : pinMap) {
    const std::string& pinName = kv.first;
    const std::string& nodeName = kv.second;
    if (_libData.isOutputPin(libCell, pinName)) {
      if (_param._driverModel == DriverModel::RampVoltage) {

      } else if (_param._driverModel == DriverModel::PWLCurrent) {

      }
    } else {

    }
  }
  return devs;
}

static inline bool
isDynamicDevice(const Device& dev)
{
  return dev._type == DeviceType::Capacitor ||
         dev._type == DeviceType::Inductor;
}

Circuit::Circuit(const NetlistParser& parser)
: _PWLData(parser.PWLData())
{
  if (parser.libDataFiles().empty() == false) {
    _libData.read(parser.libDataFiles());
  }
  const std::vector<ParserDevice>& parserDevs = parser.devices();
  std::vector<std::string> allNodeNames;
  std::string groundNodeName = allNodes(parserDevs, allNodeNames);
  if (parser.userGroundNet().size() > 0) {
    groundNodeName = parser.userGroundNet();
  }
  std::unordered_map<std::string, size_t> nodeIdMap;
  printf("Ground node identified as node \"%s\"\n", groundNodeName.data());
  _nodes.reserve(allNodeNames.size());
  Node ground;
  ground._name = groundNodeName;
  ground._nodeId = 0;
  ground._isGround = true;
  _nodes.push_back(ground);
  nodeIdMap.insert({ground._name, ground._nodeId});
  _groundNodeId = ground._nodeId;
  for (const std::string& nodeName : allNodeNames) {
    if (nodeName == groundNodeName) {
      continue;
    }
    Node n;
    n._name = nodeName;
    n._nodeId = _nodes.size();
    n._isGround = false;
    _nodes.push_back(n);
    nodeIdMap.insert({n._name, n._nodeId});
  }

  _devices.reserve(parserDevs.size());
  for (const ParserDevice& pDev : parserDevs) {
    Device dev;
    size_t devId = _devices.size();
    dev._devId = devId;
    if (createDevice(dev, pDev, nodeIdMap) == false) {
      continue;
    }
    _devices.push_back(dev);
    size_t posNode = dev._posNode;
    _nodes[posNode]._connection.push_back(devId);
    size_t negNode = dev._negNode;
    _nodes[negNode]._connection.push_back(devId);
  }
  _order = 0;
  double smallestValueOfDynamicDevice = std::numeric_limits<double>::max();
  double largestValueOfStaticDevice = -1;
  for (size_t i=0; i<_devices.size(); ++i) {
    Device& dev = _devices[i];
    if (dev._type == DeviceType::CCCS || 
        dev._type == DeviceType::CCVS) {
      size_t sampleDev = findDeviceId(dev._posSampleNode, dev._negSampleNode, _nodes);
      if (sampleDev == static_cast<size_t>(-1)) {
        printf("ERROR: Cannot find sampling branch with %s and %s of current controlled device %s\n", 
          _nodes[dev._posSampleNode]._name.data(), _nodes[dev._negSampleNode]._name.data(), dev._name.data());
      } else {
        dev._sampleDevice = sampleDev;
      }
    }
    if (isDynamicDevice((dev))) {
      if (dev._value < smallestValueOfDynamicDevice) {
        smallestValueOfDynamicDevice = dev._value;
      }
      ++_order;
    } else if (dev._type == DeviceType::Resistor) {
      if (dev._value > largestValueOfStaticDevice) {
        largestValueOfStaticDevice = dev._value;
      }
    }
  }
  //printf("DEBUG: largest: %E, smallest: %E\n", largestValueOfStaticDevice, smallestValueOfDynamicDevice);
  _scalingFactor = std::pow(10, (0 - int(std::log10(smallestValueOfDynamicDevice)) - 3));
  if (_scalingFactor < 1) {
    _scalingFactor = 1;
  }
}

const PWLValue&
Circuit::PWLData(const Device& dev) const
{
  static PWLValue empty;
  if (dev._isPWLValue == false) {
    return empty;
  }
  return _PWLData[dev._PWLData];
}

const Device& 
Circuit::findDeviceByName(const std::string& name) const
{
  for (const Device& dev : _devices) {
    if (dev._name == name) {
      return dev;
    }
  }
  static Device empty;
  empty._type = DeviceType::Total;
  return empty;
}

const Node&
Circuit::findNodeByName(const std::string& name) const
{
  for (const Node& node : _nodes) {
    if (node._name == name) {
      return node;
    }
  }
  static Node empty;
  return empty;
}

}