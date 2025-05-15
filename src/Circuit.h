#ifndef _TRAN_CKT_H_
#define _TRAN_CKT_H_

#include "Base.h"
#include <cstddef>
#include <vector>
#include "NetlistParser.h"
#include "LibData.h"

namespace NA {

class Circuit {
  public:
    Circuit(const NetlistParser& parser, const AnalysisParameter& param);

    std::string simName() const { return _param._name; }
    
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

    bool isGroundNode(size_t nodeId) const { return _groundNodeId == nodeId; }

    /// For moment scaling in PZ and TF analysis
    size_t scalingFactor() const { return _scalingFactor; }
    size_t order() const { return _order; }

  private:
    std::string allNodes(const std::vector<ParserDevice>& devs, std::vector<std::string>& allNodes);
    typedef std::unordered_map<std::string, size_t> StringIdMap;
    void elaborateGateDevice(const ParserDevice& dev, const StringIdMap& nodeIdMap);
    Device createDevice(const ParserDevice& pDev, const StringIdMap& nodeIdMap);
    void updateNodeConnection(const Device& dev);
    void buildCircuit(const NetlistParser& parser);

  private:
    size_t                         _groundNodeId;
    size_t                         _order;
    size_t                         _scalingFactor;
    AnalysisParameter              _param;
    std::vector<Node>              _nodes;
    std::vector<Device>            _devices;
    std::vector<PWLValue>          _PWLData;
    LibData                        _libData;
}; 

}

#endif
