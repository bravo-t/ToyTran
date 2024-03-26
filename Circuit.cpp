#include <algorithm>
#include "Circuit.h"
#include "NetlistParser.h"

namespace Tran {

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


Circuit::Circuit(const NetlistParser& parser)
: _devices(parser.devices()), 
  _PWLData(parser.PWLData())
{
  const std::vector<std::string>& nodeNames = parser.nodes();
  _nodes.reserve(nodeNames.size());
  for (const std::string& nodeName : nodeNames) {
    Node n;
    n._name = nodeName;
    n._nodeId = _nodes.size();
    _nodes.push_back(n);
  }
  for (size_t i=0; i<_devices.size(); ++i) {
    Device& dev = _devices[i];
    dev._devId = i;
    size_t posNode = dev._posNode;
    _nodes[posNode]._connection.push_back(i);
    size_t negNode = dev._negNode;
    _nodes[negNode]._connection.push_back(i);
    if (isAnySource(dev)) {
      _nodes[negNode]._isGround = true;
    }
  }
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
  }

  printf("Following node(s) are recognized as ground node(s):\n");
  for (const Node& n : _nodes) {
    if (n._isGround) {
      printf("  %s\n", n._name.data());
    }
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