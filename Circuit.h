#ifndef _TRAN_CKT_H_
#define _TRAN_CKT_H_

#include <Eigen/Core>
#include "Base.h"

namespace Tran {

class NetlistParser;

class Circuit {
  public:
    Circuit(const NetlistParser& parser);

    Eigen::MatrixXd A() const;
    Eigen::VectorXd b() const;
    size_t mSize() const;
    size_t nSize() const;
    size_t Asize() const;
  private:
    

  private:
    size_t                         _ASize = 0;
    size_t                         _bSize = 0;
    std::vector<Node>              _nodes;
    std::vector<Device>            _devices;
    std::vector<DependentDevice>   _dependentDevices;
    std::vector<PWLValue>          _PWLData;
}; 

}


#endif
