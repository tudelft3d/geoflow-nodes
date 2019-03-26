#include <geoflow/core/geoflow.hpp>

#include <ogrsf_frmts.h>
#include <geos_c.h>

namespace geoflow::nodes::gdal {

  class OGRLoaderNode:public Node {
    int layer_count = 0;

    std::string geometry_type_name;
    OGRwkbGeometryType geometry_type;
    
    void push_attributes(OGRFeature& poFeature);

  public:
    using Node::Node;
    void init() {
      add_output("line_strings", typeid(LineStringCollection));
      add_output("linear_rings", typeid(LinearRingCollection));

      add_output_group("attributes", {typeid(vec1b), typeid(vec1i), typeid(vec1f), typeid(vec1s)});

      add_param("filepath", (std::string) "");
      add_param("layer_id", (int)0);

      GDALAllRegister();
    }
    void gui() {
      ImGui::InputText("File path", &param<std::string>("filepath"));
      ImGui::SliderInt("Layer id", &param<int>("layer_id"), 0, layer_count - 1);
      ImGui::Text("%s", geometry_type_name.c_str());
    }
    void process();
  };

  class OGRWriterNode:public Node {
  public:
    int epsg = 7415;

    using Node::Node;
    void init() {
      add_input("geometries", { typeid(LineStringCollection), typeid(LinearRingCollection) });

      add_input_group("attributes", {typeid(vec1b), typeid(vec1i), typeid(vec1f), typeid(vec1s)});

      add_param("filepath", (std::string) "out");
    }
    void gui() {
      ImGui::InputText("File path", &param<std::string>("filepath"));
    }
    void process();
  };

  class OGRWriterNoAttributesNode:public Node {
  public:
    int epsg = 7415;
    using Node::Node;
    void init() {
      add_input("geometries", { typeid(LineStringCollection), typeid(LinearRingCollection) });
      
      add_param("filepath", (std::string) "out");
    }
    void gui() {
      ImGui::InputText("File path", &param<std::string>("filepath"));
    }
    void process();
  };

  class CSVLoaderNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_output("points", typeid(PointCollection));

      add_param("filepath", (std::string) "");
      add_param("thin_nth", (int)5);
    }
    void gui() {
      ImGui::InputText("CSV file path", &param<std::string>("filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 1, 100);
    }
    void process();
  };

  class CSVWriterNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("points", typeid(PointCollection));
      add_input("distances", typeid(vec1f));

      add_param("filepath", (std::string) "out.csv");
    }
    void gui() {
      ImGui::InputText("CSV file path", &param<std::string>("filepath"));
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
