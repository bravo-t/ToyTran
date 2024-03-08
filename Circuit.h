#ifndef _TRAN_CKT_H_
#define _TRAN_CKT_H_

#include "Base.h"

namespace Tran {

class NetlistParser;

class Circuit {
  public:
    Circuit(const NetlistParser& parser);

    std::vector<Node> nodes() const { return _nodes; }
    std::vector<Device> devices() const { return _devices; }
    std::vector<PWLValue> PWLData() const { return _PWLData; }
    const PWLValue& PWLData(const Device& dev) const;
    const Device& device(size_t id) const { return _devices[id]; }
    const Node& node(size_t id) const { return _nodes[id]; }

  private:
    std::vector<Node>              _nodes;
    std::vector<Device>            _devices;
    std::vector<PWLValue>          _PWLData;
}; 

}


#endif
