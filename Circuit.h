#ifndef _TRAN_CKT_H_
#define _TRAN_CKT_H_

#include "Base.h"

namespace Tran {

class NetlistParser;

class Circuit {
  public:
    Circuit(const NetlistParser& parser);

  private:
    buildNode();
    

  private:
    std::vector<Node>     _nodes;
    std::vector<Device>   _devices;
    std::vector<PWLValue> _PWLData;
}; 

}


#endif
