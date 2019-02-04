#include <geoflow/core/geoflow.hpp>

#include <ogrsf_frmts.h>

namespace geoflow::nodes::gdal {

  class OGRLoaderNode:public Node {
    int layer_count = 0;
    
    int current_layer_id = 0;
    std::string geometry_type_name;
    OGRwkbGeometryType geometry_type;

    public:
    // char filepath[256] = "/Users/ravi/surfdrive/Data/step-edge-detector/hoogtelijnen_dgmr_.gpkg";
    char filepath[256] = "";
    // char filepath[256] = "/Users/ravi/surfdrive/Projects/RWS-Basisbestand-3D-geluid/3D-basisbestand-geluid-v0.1/output/hoogtelijnen/hoogtelijnen_v2/hoogtelijnen_out";

    using Node::Node;
    void init() {
      add_output("line_strings", TT_line_string_collection);
      add_output("linear_rings", TT_linear_ring_collection);
      GDALAllRegister();
    }
    void gui(){
      ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
      ImGui::SliderInt("Layer id", &current_layer_id, 0, layer_count-1);
      ImGui::Text("%s", geometry_type_name.c_str());
    }
    void process();
  };
  // class OGRLoaderOldNode:public Node {
    
  class OGRWriterNode:public Node {
    public:
    char filepath[256] = "blabla.gpkg";
    int epsg = 7415;


    using Node::Node;
    void init() {
        add_input("geometries", {TT_line_string_collection, TT_linear_ring_collection});
        add_input("attributes", TT_attribute_map_f);
    }
    void gui() {
      ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    }
    void process();
  };
  class OGRWriterNoAttributesNode:public Node {
    public:
    char filepath[256] = "out";
    int epsg = 7415;
    

      using Node::Node;
    void init() {
        add_input("geometries", {TT_line_string_collection, TT_linear_ring_collection});
    }

    void gui() {
      ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    }
    void process();
  };

  class CSVLoaderNode:public Node {
    public:
    char filepath[256] = "/Users/ravi/git/heightjump-detect/build/ComparePointDistanceNode.out";
    int thin_nth = 5;

    using Node::Node;
    void init() {
      add_output("points", TT_point_collection);
    }
    void gui(){
      ImGui::InputText("CSV file path", filepath, IM_ARRAYSIZE(filepath));
      ImGui::SliderInt("Thin nth", &thin_nth, 1, 100);
    }
    void process();
  };

  class CSVWriterNode:public Node {
    public:
    char filepath[256] = "/Users/ravi/git/heightjump-detect/build/csv.out";

    using Node::Node;
    void init() {
      add_input("points", TT_point_collection);
      add_input("distances", TT_vec1f);
    }
    void gui(){
      ImGui::InputText("CSV file path", filepath, IM_ARRAYSIZE(filepath));
    }
    void process();
  };

}