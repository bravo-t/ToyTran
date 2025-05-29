#include <algorithm>
#include <cstddef>
#include <limits>
#include <cmath>
#include <unordered_set>
#include "Debug.h"
#include "Circuit.h"
#include "Base.h"
#include "NetlistParser.h"
#include "Timer.h"

namespace NA {

static const size_t invalidId = static_cast<size_t>(-1);

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

static inline std::string 
internalVPosNodeName(const std::string& inst, const std::string& pin) 
{
  std::string internalNode = inst + "/" + pin + "/VPOS";
  return internalNode;
}

static inline std::string
internalRampVoltageSourceName(const std::string& inst, const std::string& pin) 
{
  std::string devName = inst + "/" + pin + "/Vd";
  return devName;
}

static inline std::string
internalDriverResistorName(const std::string& inst, const std::string& pin)
{
  std::string devName = inst + "/" + pin + "/Rd";
  return devName;
}

static inline std::string
internalCurrentSourceName(const std::string& inst, const std::string& pin)
{
  std::string devName = inst + "/" + pin + "/Id";
  return devName;
}

static inline std::string
internalLoaderCapName(const std::string& inst, const std::string& pin)
{
  std::string devName = inst + "/" + pin + "/Cl";
  return devName;
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
      const std::string& internalNode = internalVPosNodeName(dev._name, pinName);
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

ParserDevice 
createDriverVoltageSourceParserDevice(const std::string& inst, const std::string& pin, const std::string& gnd)
{
  ParserDevice dev;
  dev._name = internalRampVoltageSourceName(inst, pin);
  dev._posNode = internalVPosNodeName(inst, pin);
  dev._negNode = gnd;
  dev._type = DeviceType::VoltageSource;
  dev._isPWLValue = false;
  dev._isInternal = true;
  dev._value = 0;
  return dev;
}

ParserDevice 
createDriverResistorParserDevice(const std::string& inst, const std::string& pin, const std::string& pinNode)
{
  ParserDevice dev;
  dev._name = internalDriverResistorName(inst, pin);
  dev._posNode = internalVPosNodeName(inst, pin);
  dev._negNode = pinNode;
  dev._type = DeviceType::Resistor;
  dev._isPWLValue = false;
  dev._isInternal = true;
  dev._value = 0;
  return dev;
}

ParserDevice 
createDriverCurrentSourceParserDevice(const std::string& inst, const std::string& pin, 
                           const std::string& gnd, const std::string& pinNode)
{
  ParserDevice dev;
  dev._name = internalCurrentSourceName(inst, pin);
  dev._posNode = pinNode;
  dev._negNode = gnd;
  dev._type = DeviceType::CurrentSource;
  dev._isPWLValue = false;
  dev._isInternal = true;
  dev._value = 0;
  return dev;
}

ParserDevice 
createLoaderCapParserDevice(const std::string& inst, const std::string& pin, 
                           const std::string& gnd, const std::string& pinNode)
{
  ParserDevice dev;
  dev._name = internalLoaderCapName(inst, pin);
  dev._posNode = pinNode;
  dev._negNode = gnd;
  dev._type = DeviceType::Capacitor;
  dev._isPWLValue = false;
  dev._isInternal = true;
  dev._value = 0;
  return dev;
}

Device*
Circuit::createDevice(const ParserDevice& pDev, const StringIdMap& nodeIdMap)
{
  Device dev;
  size_t devId = _devices.size();
  dev._devId = devId;
  dev._isInternal = pDev._isInternal;
  if (::NA::createDevice(dev, pDev, nodeIdMap) == true) {
    _devices.push_back(dev);
    updateNodeConnection(dev);
    return &(_devices.back());
  } else {
    return nullptr;
  }
}

void
Circuit::elaborateGateDevice(const ParserDevice& dev, const StringIdMap& nodeIdMap)
{
  std::vector<Device> devs;
  const std::string& libCell = dev._libCellName;
  const auto& pinMap = dev._pinMap;
  const std::string& gndNode = _nodes[_groundNodeId]._name;
  std::vector<std::string> outputPins;
  for (const auto& kv : pinMap) {
    const std::string& pinName = kv.first;
    const std::string& nodeName = kv.second;
    if (_libData.isOutputPin(libCell, pinName)) {
      outputPins.push_back(pinName);
    } else {
      const ParserDevice& Cl = createLoaderCapParserDevice(dev._name, pinName, gndNode, nodeName);
      createDevice(Cl, nodeIdMap);
    }
  }
  for (const std::string& outPin : outputPins) {
    const std::vector<std::string>& inputPins = _libData.cellArcInputPins(libCell, outPin);
    assert(inputPins.size() < 2 && "Multiple fanin cell arcs are beyond the scope of this code");
    if (inputPins.empty()) {
      printf("ERROR: Lib data for cell arc to pin %s of cell %s is missing\n", outPin.data(), libCell.data());
      continue;
    }
    const std::string& inPin = inputPins[0];
    const auto& foundInputNode = pinMap.find(inPin);
    if (foundInputNode != pinMap.end()) {
      const std::string& inputNode = foundInputNode->second;
      size_t inputNodeId = ::NA::findNodeByName(nodeIdMap, inputNode);
      const auto& foundOutputNode = pinMap.find(outPin);
      if (foundOutputNode != pinMap.end()) {
        CellArc cellArcData(&_libData, libCell, inPin, outPin);
        if (cellArcData.empty()) {
          printf("ERROR: Lib data for cell arc %s->%s of cell %s is missing\n", inPin.data(), outPin.data(), libCell.data());
          continue;
        }
        cellArcData.setInputTranNode(inputNodeId);
        const std::string& outputNode = foundOutputNode->second;
        size_t outputNodeId = ::NA::findNodeByName(nodeIdMap, outputNode);
        if (_param._driverModel == DriverModel::RampVoltage) {
          const ParserDevice& VRampPDev = createDriverVoltageSourceParserDevice(dev._name, outPin, gndNode);
          Device* driverSource = createDevice(VRampPDev, nodeIdMap);
          driverSource->_sampleDevice = _cellArcs.size();
          driverSource->_isPWLValue = true;
          driverSource->_PWLData = _PWLData.size();
          PWLValue empty;
          _PWLData.push_back(empty);
          const ParserDevice& Rd = createDriverResistorParserDevice(dev._name, outPin, outputNode);
          Device* driverRes = createDevice(Rd, nodeIdMap);
          driverRes->_sampleDevice = _cellArcs.size();
          cellArcData.setDriverResistorId(driverRes->_devId);
          _cellArcs.push_back(cellArcData);
        } else if (_param._driverModel == DriverModel::PWLCurrent) {
          const ParserDevice& Id = createDriverCurrentSourceParserDevice(dev._name, outPin, gndNode, outputNode);
          Device* driverSource = createDevice(Id, nodeIdMap);
          driverSource->_sampleDevice = _cellArcs.size();
          driverSource->_isPWLValue = true;
          driverSource->_PWLData = _PWLData.size();
          PWLValue empty;
          _PWLData.push_back(empty);
          _cellArcs.push_back(cellArcData);
        }
        _driverOutputNodes.push_back(outputNodeId);
      }
    }
  }
}

static inline bool
isDynamicDevice(const Device& dev)
{
  return dev._type == DeviceType::Capacitor ||
         dev._type == DeviceType::Inductor;
}

static inline void
printInfo(const std::string& simName, const std::vector<Device>& devs, 
          const std::vector<Node>& nodes, const LibData& libData)
{
  std::vector<size_t> devCounter(static_cast<unsigned char>(DeviceType::Total), 0);
  for (const Device& dev : devs) {
    ++devCounter[static_cast<unsigned char>(dev._type)];
  }
  printf("Circuit built for %s, devices created:\n"
         "  %lu resistors\n"
         "  %lu capacitors\n"
         "  %lu inductors\n"
         "  %lu independent voltage sources\n"
         "  %lu independent current sources\n"
         "  %lu VCCS\n"
         "  %lu VCVS\n"
         "  %lu CCCS\n"
         "  %lu CCVS\n"
         "%lu nodes created\n"
         "%lu lib cells loaded\n",
    simName.data(), 
    devCounter[static_cast<unsigned char>(DeviceType::Resistor)],
    devCounter[static_cast<unsigned char>(DeviceType::Capacitor)], 
    devCounter[static_cast<unsigned char>(DeviceType::Inductor)], 
    devCounter[static_cast<unsigned char>(DeviceType::VoltageSource)], 
    devCounter[static_cast<unsigned char>(DeviceType::CurrentSource)], 
    devCounter[static_cast<unsigned char>(DeviceType::VCCS)], 
    devCounter[static_cast<unsigned char>(DeviceType::VCVS)],
    devCounter[static_cast<unsigned char>(DeviceType::CCCS)], 
    devCounter[static_cast<unsigned char>(DeviceType::CCVS)], 
    nodes.size(),
    libData.cellCount());
}

Circuit::Circuit(const NetlistParser& parser, const AnalysisParameter& param)
: _param(param), _PWLData(parser.PWLData())
{
  if (parser.libDataFiles().empty() == false) {
    _libData.read(parser.libDataFiles());
  }

  timespec cktStart;
  clock_gettime(CLOCK_REALTIME, &cktStart);
  
  buildCircuit(parser);
  
  timespec cktEnd;
  clock_gettime(CLOCK_REALTIME, &cktEnd);

  if (_cellArcs.empty() == false) { 
    printInfo(simName(), _devices, _nodes, _libData);
  }
  printf("Time spent in building circuit for %s: %.3f milliseconds\n",
         simName().data(), 1e-6*timeDiffNs(cktEnd, cktStart));

  if (Debug::enabled()) {
    debugPrint();
  }
}

void
Circuit::debugPrint() const 
{
  printf("DEBUG Devices: \n");
  for (const Device& dev : _devices) {
    printf("  Dev %s: ID: %lu, node %lu-> node %lu\n", 
    dev._name.data(), dev._devId, dev._posNode, dev._negNode);
  }

  printf("DEBUG Nodes: \n");
  for (const Node& node : _nodes) {
    printf("Node %s: ID: %lu, conn: ", node._name.data(), node._nodeId);
    for (const size_t& devId : node._connection) {
      printf("%lu ", devId);
    }
    printf("\n");
  }
}

void
Circuit::buildCircuit(const NetlistParser& parser)
{
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
    if (pDev._type == DeviceType::Cell) {
      elaborateGateDevice(pDev, nodeIdMap);
    } else {
      createDevice(pDev, nodeIdMap);
    }
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

void
Circuit::updateNodeConnection(const Device& dev)
{
  size_t devId = dev._devId;
  size_t posNode = dev._posNode;
  _nodes[posNode]._connection.push_back(devId);
  size_t negNode = dev._negNode;
  _nodes[negNode]._connection.push_back(devId);
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

static std::vector<const Device*> 
getConnectedDevices(size_t nodeId, const Circuit* ckt)
{
  std::vector<const Device*> devs;
  const Node& node = ckt->node(nodeId);
  if (node._isGround) {
    return devs;
  }
  for (size_t devId : node._connection) {
    const Device& dev = ckt->device(devId);
    devs.push_back(&dev);
  }
  return devs;
}

size_t
getOtherSideNodeId(const Device* dev, size_t nodeId) 
{
  if (dev->_posNode == nodeId) {
    return dev->_negNode;
  } else {
    return dev->_posNode;
  }
}

std::vector<const Device*> 
Circuit::traceDevice(size_t devId) const
{
  std::unordered_set<size_t> visitedNodes;
  std::vector<const Device*> devs;
  const Device* dev = &device(devId);
  std::unordered_set<size_t> wavefront;
  std::unordered_set<size_t> nextWavefront;
  wavefront.insert(dev->_posNode);
  while (!wavefront.empty()) {
    nextWavefront.clear();
    for (size_t nodeId : wavefront) {
      if (visitedNodes.find(nodeId) == visitedNodes.end()) {
        const std::vector<const Device*>& connDevs = getConnectedDevices(nodeId, this);
        devs.insert(devs.end(), connDevs.begin(), connDevs.end());
        visitedNodes.insert(nodeId);
        for (const Device* connDev : connDevs) {
          nextWavefront.insert(getOtherSideNodeId(connDev, nodeId));
        }
      }
      wavefront.swap(nextWavefront); 
    }
  }
  return devs;
}

CellArc::CellArc(const LibData* libData, const std::string& cell, 
                const std::string& fromPin, const std::string& toPin) 
: _cellName(cell), _fromPin(fromPin), _toPin(toPin)
{
  _nldmArc = libData->findNLDMArc(cell, fromPin, toPin);
  _ccsArc = libData->findCCSArc(cell, fromPin, toPin);
}

static double
transitionTime(const PWLValue& data, double voltage, double thres1, double thres2) 
{
  double t1 = data.measure(voltage * thres1 / 100);
  double t2 = data.measure(voltage * thres2 / 100);
  if (t1 == 1e99 || t2 == 1e99) {
    return 0;
  }
  return t2 - t1;
}

static double
transitionTime(const Device* vSrc, const Circuit* ckt, const LibData* libData)
{
  if (vSrc->_isPWLValue == false) {
    return 0;
  } 
  const PWLValue& data = ckt->PWLData(*vSrc);
  double voltage = libData->voltage();
  if (data.isRiseTransition()) {
    return transitionTime(data, voltage, libData->riseTransitionLowThres(), libData->riseTransitionHighThres());
  } else {
    return transitionTime(data, voltage, libData->fallTransitionHighThres(), libData->fallTransitionLowThres());
  }
  return 0;
}

double
CellArc::inputTransition(const Circuit* ckt) const
{
  size_t inputSourceId = inputSourceDevId(ckt);
  if (inputSourceId == invalidId) {
    return 0;
  }
  const Device& inputSource = ckt->device(inputSourceId);
  const LibData* libData = _nldmArc->owner();
  return transitionTime(&inputSource, ckt, libData);
}

size_t
CellArc::inputSourceDevId(const Circuit* ckt) const
{
  const Node& inputTranNode = ckt->node(_inputTranNode);
  const Device* inputSource = nullptr;
  for (size_t connDevId : inputTranNode._connection) {
    const Device& connDev = ckt->device(connDevId);
    if (connDev._type != DeviceType::Capacitor) {
      return connDev._devId;
    }
  }
  return invalidId;
}

}
