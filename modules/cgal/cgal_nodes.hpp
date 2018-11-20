#include <geoflow.hpp>
#include <imgui.h>

using namespace geoflow;

class CDTNode:public Node {
  public:
  CDTNode(NodeManager& manager):Node(manager) {
    // add_input("points", TT_any);
    add_input("lines_vec3f", TT_vec3f);
    add_output("cgal_CDT", TT_any);
    add_output("normals_vec3f", TT_vec3f);
    add_output("triangles_vec3f", TT_vec3f);
  }
  void process();
};

// class ComparePointDistanceNode:public Node{};
// class PointDistanceNode:public Node{};

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
    ImGui::SliderFloat("Error threshold", &thres_error, 0, 100);
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