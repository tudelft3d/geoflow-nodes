#include <geoflow.hpp>
#include <imgui.h>

#include <ogrsf_frmts.h>

using namespace geoflow;

class OGRLoaderNode:public Node {
  int layer_count = 0;
  
  int current_layer_id = 0;
  std::string geometry_type_name;
  OGRwkbGeometryType geometry_type;

  public:
  // char filepath[256] = "/Users/ravi/surfdrive/Data/step-edge-detector/hoogtelijnen_dgmr_.gpkg";
  char filepath[256] = "/Users/ravi/surfdrive/Data/step-edge-detector/hoogtelijnen_v01_simp_dp1m_subset.gpkg";
  // char filepath[256] = "/Users/ravi/surfdrive/Projects/RWS-Basisbestand-3D-geluid/3D-basisbestand-geluid-v0.1/output/hoogtelijnen/hoogtelijnen_v2/hoogtelijnen_out";

  OGRLoaderNode(NodeManager& manager):Node(manager) {
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


  OGRWriterNode(NodeManager& manager):Node(manager) {
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
  char filepath[256] = "blabla.gpkg";
  int epsg = 7415;
  
  
  OGRWriterNoAttributesNode(NodeManager& manager):Node(manager) {
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

  CSVLoaderNode(NodeManager& manager):Node(manager) {
    add_output("points", TT_vec3f);
    add_output("distances1", TT_vec1f);
    add_output("distances2", TT_vec1f);
    add_output("difference", TT_vec1f);
  }

  void gui(){
    ImGui::InputText("CSV file path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 0, 100);
  }
  void process();
};