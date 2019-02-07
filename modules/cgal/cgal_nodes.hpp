#include <geoflow.hpp>
#include <imgui.h>

using namespace geoflow;

class CDTNode:public Node {
  public:
  bool create_triangles = false;
  
  CDTNode(NodeManager& manager):Node(manager) {
    add_input("geometries", { TT_point_collection, TT_line_string_collection });
    add_output("cgal_cdt", TT_any);
    add_output("triangles", TT_triangle_collection);
  }
  void process();
};

class DTNode:public Node {
public:
  DTNode(NodeManager& manager):Node(manager) {
    add_input("points", TT_point_collection);
    add_output("cgal_dt", TT_any);
  }
  void process();
};

class ComparePointDistanceNode:public Node {
  public:
  char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
  char log_filepath[256] = "ComparePointDistanceNode.out";
  int thin_nth = 20;

  ComparePointDistanceNode(NodeManager& manager):Node(manager) {
    add_input("triangles1_vec3f", TT_vec3f);
    add_input("triangles2_vec3f", TT_vec3f);
    add_output("points", TT_vec3f);
    add_output("distances1", TT_vec1f);
    add_output("distances2", TT_vec1f);
    add_output("diff", TT_vec1f);
  }
  void gui(){
    ImGui::InputText("LAS file path", las_filepath, IM_ARRAYSIZE(las_filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 0, 100);
  }
  void process();
};

class PointDistanceNode:public Node{
  public:
  char filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
  int thin_nth = 5;
  bool overwritez = false;

  PointDistanceNode(NodeManager& manager):Node(manager) {
    add_input("triangles", TT_triangle_collection);
    add_output("points", TT_point_collection);
    add_output("distances", TT_vec1f);
  }
  void gui(){
    ImGui::InputText("LAS file path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 0, 100);
  }
  void process();
};

class DensifyNode:public Node {
  public:
  float interval = 2;

  DensifyNode(NodeManager& manager):Node(manager) {
    add_input("geometries", {TT_line_string_collection});
    add_output("dense_linestrings", TT_line_string_collection);
  }
  void gui(){
    ImGui::SliderFloat("Interval", &interval, 0, 100);
  }
  void process();
};

class TinSimpNode:public Node {
  public:
  float thres_error = 2;
  float densify_interval = 2;

  TinSimpNode(NodeManager& manager):Node(manager) {
    add_input("geometries", {TT_point_collection, TT_line_string_collection});
    add_output("triangles", TT_triangle_collection);
    add_output("normals", TT_vec3f);
    add_output("selected_lines", TT_line_string_collection);
    // add_output("count", TT_vec1ui);
    // add_output("error", TT_vec1f);
  }
  void gui(){
    if (ImGui::SliderFloat("Error threshold", &thres_error, 0, 100)) {
      manager.run(*this);
    }
    if (inputs("geometries").connected_type == TT_line_string_collection) 
      ImGui::SliderFloat("Line densify", &densify_interval, 0, 100);
  }
  void process();
};

class SimplifyLine3DNode:public Node {
  public:
  float area_threshold=0.1;

  SimplifyLine3DNode(NodeManager& manager):Node(manager) {
    add_input("lines", TT_line_string_collection);
    add_output("lines", TT_line_string_collection);
  }
  void gui(){
    if(ImGui::DragFloat("stop cost", &area_threshold,0.1)) {
      manager.run(*this);
    }
  }
  void process();
};

class SimplifyLineNode:public Node {
  public:
  float threshold_stop_cost=0.1;
  
  SimplifyLineNode(NodeManager& manager):Node(manager) {
    add_input("lines", TT_geometry);
    add_output("lines", TT_geometry);
    add_output("lines_vec3f", TT_vec3f);
  }
  void gui(){
    if(ImGui::DragFloat("stop cost", &threshold_stop_cost,0.01)) {
      manager.run(*this);
    }
  }
  void process();
};

class SimplifyLinesNode:public Node {
  public:
  float threshold_stop_cost=0.1;

  SimplifyLinesNode(NodeManager& manager):Node(manager) {
    add_input("lines", TT_line_string_collection);
    add_input("lines2", TT_line_string_collection);
    add_output("lines", TT_line_string_collection);
  }
  void gui(){
    if(ImGui::DragFloat("stop cost", &threshold_stop_cost,0.01)) {
      manager.run(*this);
    }
  }
  void process();
};

class SimplifyFootprintNode:public Node {
  public:
  float threshold_stop_cost=0.1;

  SimplifyFootprintNode(NodeManager& manager):Node(manager) {
    add_input("polygons", TT_linear_ring_collection);
    add_output("polygons_simp", TT_linear_ring_collection);
  }
  void gui(){
    if(ImGui::DragFloat("stop cost", &threshold_stop_cost,0.01)) {
      manager.run(*this);
    }
  }
  void process();
};

class SimplifyFootprintCDTNode:public Node {
public:
  float threshold_stop_cost = 0.1;

  SimplifyFootprintCDTNode(NodeManager& manager):Node(manager) {
    add_input("polygons", TT_linear_ring_collection);
    add_output("polygons_simp", TT_linear_ring_collection);
  }
  void gui() {
    if (ImGui::DragFloat("stop cost", &threshold_stop_cost, 0.01)) {
      manager.run(*this);
    }
  }
  void process();
};

class PLWriterNode:public Node {
  public:
  char filepath[256] = "/Users/ravi/out.ply";
  bool multiple_files = true;
  bool write_binary = false;

  PLWriterNode(NodeManager& manager):Node(manager) {
    add_input("points", TT_point_collection); //TT_point_collection_list
    add_input("labels", TT_vec1i);
  }
  void gui(){
    ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    // ImGui::Checkbox("Write multiple files in case of point cloud list", &multiple_files);
    ImGui::Checkbox("Write binary output", &write_binary);
  }
  void process();
};

class IsoLineNode:public Node {
public:
  IsoLineNode(NodeManager& manager):Node(manager) {
    add_input("cgal_cdt", TT_any);
    add_output("lines", TT_line_string_collection);
    add_output("attributes", TT_attribute_map_f);
  }

  void process();
};

class IsoLineSlicerNode:public Node {
public:
  IsoLineSlicerNode(NodeManager& manager):Node(manager) {
    add_input("cgal_cdt", TT_any);
    add_output("lines", TT_line_string_collection);
    add_output("attributes", TT_attribute_map_f);
  }
  void process();
};

class LineHeightNode:public Node {
public:
  char filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
  int thin_nth = 5;

  LineHeightNode(NodeManager& manager):Node(manager) {
    add_input("lines", TT_line_string_collection);
    add_output("lines", TT_line_string_collection);
  }
  void gui() {
    ImGui::InputText("LAS file path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 0, 100);
  }
  void process();
};