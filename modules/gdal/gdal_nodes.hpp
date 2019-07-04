#include <geoflow/geoflow.hpp>

#include <ogrsf_frmts.h>
#include <geos_c.h>

namespace geoflow::nodes::gdal {

  class OGRLoaderNode:public Node {
    int layer_count = 0;
    int layer_id = 0;
    std::string filepath = "";

    std::string geometry_type_name;
    OGRwkbGeometryType geometry_type;
    
    void push_attributes(OGRFeature& poFeature);

  public:
    using Node::Node;
    void init() {
      add_output("line_strings", typeid(LineStringCollection));
      add_output("linear_rings", typeid(LinearRingCollection));

      add_output_group("attributes", {typeid(vec1b), typeid(vec1i), typeid(vec1f), typeid(vec1s)});

      add_param("filepath", ParamPath(filepath));
      add_param("layer_id", ParamInt(layer_id));

      GDALAllRegister();
    }
    void process();
  };

  class OGRWriterNode:public Node {
    int epsg = 7415;
    std::string filepath = "out";
  public:

    using Node::Node;
    void init() {
      add_input("geometries", { typeid(LineStringCollection), typeid(LinearRingCollection) });

      add_input_group("attributes", {typeid(vec1b), typeid(vec1i), typeid(vec1f), typeid(vec1s)});

      add_param("filepath", ParamPath(filepath));
    }
    void process();
  };

  class OGRWriterNoAttributesNode:public Node {
  public:
    int epsg = 7415;
    std::string filepath = "out";
    using Node::Node;
    void init() {
      add_input("geometries", { typeid(LineStringCollection), typeid(LinearRingCollection) });
      
      add_param("filepath", ParamPath(filepath));
    }
    void process();
  };

  class CSVLoaderNode:public Node {
    std::string filepath = "out";
    int thin_nth = 5, thin_nth_min = 1, thin_nth_max = 100;
  public:
    using Node::Node;
    void init() {
      add_output("points", typeid(PointCollection));

      add_param("filepath", ParamPath(filepath));
      add_param("thin_nth", ParamIntRange(thin_nth, thin_nth_min, thin_nth_max));
    }
    void process();
  };

  class CSVWriterNode:public Node {
    std::string filepath = "out";
  public:
    using Node::Node;
    void init() {
      add_input("points", typeid(PointCollection));
      add_input("distances", typeid(vec1f));

      add_param("filepath", ParamPath(filepath));
    }
    void process();
  };

  class GEOSMergeLinesNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("lines", typeid(LineStringCollection));
      add_output("lines", typeid(LineStringCollection));
    }
    void process();
  };
}
