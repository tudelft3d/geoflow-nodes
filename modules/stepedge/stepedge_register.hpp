#include "stepedge_nodes.hpp"

namespace geoflow::nodes::stepedge {

  NodeRegister create_register() {
    auto R = NodeRegister("Step edge");
    R.register_node<AlphaShapeNode>("AlphaShape");
    R.register_node<PolygonExtruderNode>("PolygonExtruder");
    R.register_node<Arr2LinearRingsNode>("Arr2LinearRings");
    R.register_node<ExtruderNode>("Extruder");
    R.register_node<ProcessArrangementNode>("ProcessArrangement");
    R.register_node<BuildArrangementNode>("BuildArrangement");
    R.register_node<DetectLinesNode>("DetectLines");
    R.register_node<ClassifyEdgePointsNode>("ClassifyEdgePoints");
    R.register_node<ComputeMetricsNode>("ComputeMetrics");
    R.register_node<LASInPolygonsNode>("LASInPolygons");
    R.register_node<BuildingSelectorNode>("BuildingSelector");
    R.register_node<RegulariseLinesNode>("RegulariseLines");
    R.register_node<RegulariseRingsNode>("RegulariseRings");
    R.register_node<LOD13GeneratorNode>("LOD13Generator");
    return R;
  }

}