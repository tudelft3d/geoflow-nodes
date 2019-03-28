#include "las_nodes.hpp"

namespace geoflow::nodes::las {

  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create("LAS");
    R->register_node<LASLoaderNode>("LASLoader");
    R->register_node<LASGroundLoaderNode>("LASGroundLoader");
    R->register_node<LASWriterNode>("LASWriter");
    return R;
  }

}