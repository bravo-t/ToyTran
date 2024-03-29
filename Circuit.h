#ifndef _TRAN_CKT_H_
#define _TRAN_CKT_H_

#include "Base.h"

namespace Tran {

class NetlistParser;

class Circuit {
  public:
    Circuit(const NetlistParser& parser);
    
    size_t nodeNumber() const { return _nodes.size(); }
    size_t deviceNumber() const { return _devices.size(); }

    std::vector<Node> nodes() const { return _nodes; }
    std::vector<Device> devices() const { return _devices; }
    std::vector<PWLValue> PWLData() const { return _PWLData; }
    const PWLValue& PWLData(const Device& dev) const;
    const Device& device(size_t id) const { return _devices[id]; }
    const Node& node(size_t id) const { return _nodes[id]; }

    const Device& findDeviceByName(const std::string& name) const;
    const Node& findNodeByName(const std::string& name) const;

  private:
    size_t                         _groundNodeId;
    std::vector<Node>              _nodes;
    std::vector<Device>            _devices;
    std::vector<PWLValue>          _PWLData;
}; 

}


#endif
