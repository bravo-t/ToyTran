#ifndef _TRAN_CKT_H_
#define _TRAN_CKT_H_

#include "Base.h"
#include <cstddef>
#include <vector>
#include "NetlistParser.h"
#include "LibData.h"

namespace NA {

class Circuit;

class CellArc {
  public: 
    CellArc(const LibData* libData, const std::string& instName, 
            const std::string& cellName, 
            const std::string& fromPin, const std::string& toPin);

    void setInputTranNode(size_t node) { _inputTranNode = node; }
    void setDriverResistorId(size_t dev) { _driverResistor = dev; }
    void setDriverSourceId(size_t dev) { _driverSource = dev; }
  
    bool empty() const { return _nldmArc == nullptr; }

    double inputTransition(const Circuit* ckt) const;
    size_t driverResistorId() const { return _driverResistor; }
    size_t driverSourceId() const { return _driverSource; }
    size_t inputSourceDevId(const Circuit* ckt) const;
    bool isInvertedArc() const { return _nldmArc->isInverted(); }
    const NLDMArc* nldmData() const { return _nldmArc; }
    const CCSArc* ccsData() const { return _ccsArc; }

    std::string fromPinFullName() const { return _instName + "/" + _fromPin; }

    std::string instance() const { return _instName; }
    std::string fromPin() const { return _fromPin; }
    std::string toPin() const { return _toPin; }

  private:
    size_t         _inputTranNode = static_cast<size_t>(-1);
    size_t         _driverResistor = static_cast<size_t>(-1);
    size_t         _driverSource = static_cast<size_t>(-1);
    std::string    _instName;
    std::string    _cellName;
    std::string    _fromPin;
    std::string    _toPin;
    const NLDMArc* _nldmArc = nullptr;
    const CCSArc*  _ccsArc = nullptr;
};

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
    PWLValue& PWLData(const Device& dev);
    const Device& device(size_t id) const { return _devices[id]; }
    Device& device(size_t id) { return _devices[id]; }
    const Node& node(size_t id) const { return _nodes[id]; }

    const Device& findDeviceByName(const std::string& name) const;
    const Node& findNodeByName(const std::string& name) const;

    bool isGroundNode(size_t nodeId) const { return _groundNodeId == nodeId; }

    /// For moment scaling in PZ and TF analysis
    double scalingFactor() const { return _scalingFactor; }
    size_t order() const { return _order; }

    /// Trace circuit from specified Device
    std::vector<const Device*> traceDevice(const Device* dev) const;
    std::vector<const Device*> traceDevice(size_t devId) const;

    /// Find CellArc data
    const CellArc* cellArc(const std::string& fromPin) const;

    void debugPrint() const;

  private:
    std::string allNodes(const std::vector<ParserDevice>& devs, std::vector<std::string>& allNodes);
    typedef std::unordered_map<std::string, size_t> StringIdMap;
    void elaborateGateDevice(const ParserDevice& dev, const StringIdMap& nodeIdMap);
    Device* createDevice(const ParserDevice& pDev, const StringIdMap& nodeIdMap);
    void updateNodeConnection(const Device& dev);
    void buildCircuit(const NetlistParser& parser);

  private:
    size_t                         _groundNodeId;
    size_t                         _order;
    double                         _scalingFactor;
    AnalysisParameter              _param;
    std::vector<Node>              _nodes;
    std::vector<Device>            _devices;
    std::vector<PWLValue>          _PWLData;
    LibData                        _libData;
    /// Currently we only allow one driver, but let's keep it more general.
    std::vector<size_t>            _driverOutputNodes;
    std::vector<size_t>            _loaderInputNodes;
    std::vector<CellArc>           _cellArcs;
}; 

}

#endif
