#include "gdal_nodes.hpp"

namespace geoflow::nodes::gdal {
  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create("OGR");
    R->register_node<OGRLoaderNode>("OGRLoader");
    R->register_node<OGRWriterNode>("OGRWriter");
    R->register_node<OGRWriterNoAttributesNode>("OGRWriterNoAttributes");
    R->register_node<CSVLoaderNode>("CSVLoader");
    R->register_node<CSVWriterNode>("CSVWriter");
    R->register_node<GEOSMergeLinesNode>("GEOSMergeLines");
    return R;
  }
}