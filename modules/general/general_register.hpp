#include "general_nodes.hpp"

namespace geoflow::nodes::general {
  NodeRegister create_register() {
    NodeRegister R("General");
    R.register_node<MergeGeometriesNode>("MergeGeometries");
    R.register_node<MergeLinestringsNode>("MergeLinestrings");
    R.register_node<LineStringFilterNode>("LineStringFilter");
    return R;
  }
}