#include "las_nodes.hpp"

using namespace geoflow::nodes::las;

void register_nodes(geoflow::NodeRegister& node_register) {
    node_register.register_node<LASLoaderNode>("LASLoader");
    node_register.register_node<LASGroundLoaderNode>("LASGroundLoader");
    node_register.register_node<LASWriterNode>("LASWriter");
}

namespace geoflow::nodes::las {
  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create(GF_PLUGIN_NAME);
    register_nodes(*R);
    return R;
  }
}