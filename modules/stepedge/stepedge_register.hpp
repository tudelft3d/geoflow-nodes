#include "stepedge_nodes.hpp"

namespace geoflow::nodes::stepedge {

  NodeRegister create_register() {
    auto R = NodeRegister("Step edge");
    R.register_node<AlphaShapeNode>("AlphaShape");
    R.register_node<PolygonExtruderNode>("PolygonExtruder");
    R.register_node<Arr2LinearRingsNode>("Arr2LinearRings");
    R.register_node<ExtruderNode>("Extruder");
    R.register_node<ProcessArrangementNode>("ProcessArrangement");
    R.register_node<LinearRingtoRingsNode>("LinearRingtoRings");
    R.register_node<BuildArrangementNode>("BuildArrangement");
    R.register_node<BuildArrFromRingsNode>("BuildArrFromRings");
    R.register_node<DetectLinesNode>("DetectLines");
    R.register_node<DetectPlanesNode>("DetectPlanes");
    R.register_node<ClassifyEdgePointsNode>("ClassifyEdgePoints");
    R.register_node<ComputeMetricsNode>("ComputeMetrics");
    R.register_node<LASInPolygonsNode>("LASInPolygons");
    R.register_node<BuildingSelectorNode>("BuildingSelector");
    R.register_node<RegulariseLinesNode>("RegulariseLines");
    R.register_node<RegulariseRingsNode>("RegulariseRings");
    R.register_node<SimplifyFootprintNode>("SimplifyFootprint");
    R.register_node<LOD13GeneratorNode>("LOD13Generator");
    R.register_node<Ring2SegmentsNode>("Ring2Segments");
    R.register_node<PrintResultNode>("PrintResult");
    return R;
  }

}