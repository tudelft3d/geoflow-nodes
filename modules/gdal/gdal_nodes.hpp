#include <geoflow/core/geoflow.hpp>

#include <ogrsf_frmts.h>
#include <geos_c.h>

namespace geoflow::nodes::gdal {

  class OGRLoaderNode:public Node {
    int layer_count = 0;

    std::string geometry_type_name;
    OGRwkbGeometryType geometry_type;

  public:
    using Node::Node;
    void init() {
      add_output("line_strings", TT_line_string_collection);
      add_output("linear_rings", TT_linear_ring_collection);

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
      add_input("geometries", { TT_line_string_collection, TT_linear_ring_collection });
      add_input("attributes", TT_attribute_map_f);

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
      add_input("geometries", { TT_line_string_collection, TT_linear_ring_collection });

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
      add_output("points", TT_point_collection);

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
      add_input("points", TT_point_collection);
      add_input("distances", TT_vec1f);

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
      add_input("lines", TT_line_string_collection);
      add_output("lines", TT_line_string_collection);
    }
    void process();
  };
}