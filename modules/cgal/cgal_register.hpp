#include "cgal_nodes.hpp"

namespace geoflow::nodes::cgal {
  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create("CGAL");
    R->register_node<CDTNode>("CDT");
    R->register_node<DTNode>("DT");
    R->register_node<ComparePointDistanceNode>("ComparePointDistance");
    R->register_node<PointDistanceNode>("PointDistance");
    R->register_node<CDTDistanceNode>("CDTDistance");
    R->register_node<DensifyNode>("Densify");
    R->register_node<TinSimpNode>("TinSimp");
    R->register_node<TinSimpLASReaderNode>("TinSimpLASReader");
    R->register_node<SimplifyLine3DNode>("SimplifyLine3D");
    R->register_node<SimplifyLineNode>("SimplifyLine");
    R->register_node<SimplifyLinesNode>("SimplifyLines");
    R->register_node<SimplifyFootprintsCDTNode>("SimplifyFootprintsCDT");
    R->register_node<PLYWriterNode>("PLYWriter");
    R->register_node<IsoLineNode>("IsoLine");
    R->register_node<IsoLineSlicerNode>("IsoLineSlicer");
    R->register_node<LineHeightNode>("LineHeight");
    R->register_node<LineHeightCDTNode>("LineHeightCDT");
    R->register_node<SimplifyLinesBufferNode>("SimplifyLinesBuffer");
    return R;
  }
}