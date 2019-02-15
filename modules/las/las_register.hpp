#include "las_nodes.hpp"

namespace geoflow::nodes::las {

  NodeRegister create_register() {
    NodeRegister R("LAS");
    R.register_node<LASLoaderNode>("LASLoader");
    R.register_node<LASWriterNode>("LASWriter");
    return R;
  }

}