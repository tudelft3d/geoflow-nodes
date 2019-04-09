#include "general_nodes.hpp"

namespace geoflow::nodes::general {
  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create("General");
    R->register_node<MergeGeometriesNode>("MergeGeometries");
    R->register_node<MergeLinestringsNode>("MergeLinestrings");
    R->register_node<LineStringFilterNode>("LineStringFilter");
    R->register_node<PolygonToLineStringNode>("PolygonToLineString");
    R->register_node<CreateLineEquationsNode>("CreateLineEquations");
    R->register_node<CreateLineBuffersNode>("CreateLineBuffers");
    return R;
  }
}