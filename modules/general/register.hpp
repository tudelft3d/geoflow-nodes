#include "general_nodes.hpp"

using namespace geoflow::nodes::general;

void register_nodes(geoflow::NodeRegister& node_register) {
    node_register.register_node<MergeGeometriesNode>("MergeGeometries");
    node_register.register_node<MergeLinestringsNode>("MergeLinestrings");
    node_register.register_node<LineStringFilterNode>("LineStringFilter");
    node_register.register_node<OBJwriterNode>("OBJwriter");
}

namespace geoflow::nodes::las {
  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create(GF_PLUGIN_NAME);
    register_nodes(*R);
    return R;
  }
}