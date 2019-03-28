#include "general_nodes.hpp"

namespace geoflow::nodes::general {
  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create("General");
    R->register_node<MergeGeometriesNode>("MergeGeometries");
    R->register_node<MergeLinestringsNode>("MergeLinestrings");
    R->register_node<LineStringFilterNode>("LineStringFilter");
    return R;
  }
}