#include "masb_nodes.hpp"

namespace geoflow::nodes::mat {

  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create("MAT");
    R->register_node<ComputeMedialAxisNode>("ComputeMedialAxisNode");
    R->register_node<ComputeNormalsNode>("ComputeNormalsNode");
    R->register_node<SegmentMakerNode>("SegmentMaker");
    R->register_node<TestPointsNode>("TestPoints");
    R->register_node<RegionGrowMedialAxisNode>("RegionGrowMedialAxis");
    return R;
  }

}