#include "Circuit.h"
#include "NetlistParser.h"

namespace Tran {

Circuit::Circuit(const NetlistParser& parser)
: _devices(parser.devices()), _PWLData(parser.PWLData())
{
  const std::vector<std::string>& nodeNames = parser.nodes();
  _nodes.reserve(nodeNames.size());
  for (const std::string& nodeName : nodeNames) {
    Node n;
    n._name = nodeName;
    _nodes.push_back(n);
  }
  for (size_t i=0; i<_devices.size(); +i) {
    const Device& dev = _devices[i];
    size_t posNode = dev._posNode;
    _nodes[posNode]._posConnection.push_back(i);
    size_t negNode = dev._negNode;
    _nodes[negNode]._negConnection.push_back(i);
  }
}



}
