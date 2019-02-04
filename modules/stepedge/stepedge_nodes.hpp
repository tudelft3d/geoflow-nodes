#include "imgui.h"
#include "geoflow.hpp"

#include "point_edge.h"

using namespace geoflow;

class AlphaShapeNode:public Node {
  public:
  float thres_alpha = 0.7;
  bool extract_alpha_rings = false;
  AlphaShapeNode(NodeManager& manager):Node(manager) {
    // add_input("points", TT_any);
    add_input("points", TT_any);
    add_output("alpha_rings", TT_linear_ring_collection);
    add_output("edge_points", TT_point_collection);
    add_output("alpha_edges", TT_line_string_collection);
  }

  void gui(){
    ImGui::InputFloat("Alpha", &thres_alpha, 0.01, 1);
    ImGui::Checkbox("extract_alpha_rings", &extract_alpha_rings);
  }
  void process();
};

class PolygonExtruderNode:public Node {

  public:
  PolygonExtruderNode(NodeManager& manager):Node(manager) {
    add_input("polygons", TT_any);
    add_input("point_clouds", TT_any);
    add_output("polygons_extruded", TT_any);
  }

  void process();
};

class Arr2LinearRingsNode:public Node {

  public:
  Arr2LinearRingsNode(NodeManager& manager):Node(manager) {
    add_input("arrangement", TT_any);
    add_output("linear_rings", TT_linear_ring_collection);
    add_output("attributes", TT_attribute_map_f);
  }

  void process();
};

class ExtruderNode:public Node {
  bool do_walls=true, do_roofs=true;
  public:
  ExtruderNode(NodeManager& manager):Node(manager) {
    add_input("arrangement", TT_any);
    add_output("cell_id_vec1i", TT_vec1i);
    add_output("triangles", TT_triangle_collection);
    add_output("normals_vec3f", TT_vec3f);
    add_output("labels_vec1i", TT_vec1i); // 0==ground, 1==roof, 2==outerwall, 3==innerwall
  }

  void gui() {
    ImGui::Checkbox("Do walls", &do_walls);
    ImGui::Checkbox("Do roofs", &do_roofs);
  }
  void process();
};

class ProcessArrangementNode:public Node {

  public:
  config c;
  ProcessArrangementNode(NodeManager& manager):Node(manager) {
    add_input("arrangement", TT_any);
    add_input("points", TT_any);
    add_output("arrangement", TT_any);
  }

  void gui() {
    ImGui::DragFloat("Min step height", &c.step_height_threshold, 0.1);
    ImGui::DragFloat("zrange_threshold", &c.zrange_threshold, 0.1);
    ImGui::Checkbox("merge_segid", &c.merge_segid);
    ImGui::Checkbox("merge_zrange", &c.merge_zrange);
    ImGui::Checkbox("merge_step_height", &c.merge_step_height);
    ImGui::Checkbox("merge_unsegmented", &c.merge_unsegmented);
    ImGui::Checkbox("merge_dangling_egdes", &c.merge_dangling_egdes);
  }
  void process();
};

class BuildArrangementNode:public Node {

  public:
  bool remove_unsupported=false;

  BuildArrangementNode(NodeManager& manager):Node(manager) {
    add_input("edge_segments", TT_line_string_collection);
    add_input("footprint", TT_linear_ring);
    add_output("arrangement", TT_any);
    add_output("arr_segments", TT_line_string_collection);
  }
  void gui() {
    ImGui::Checkbox("Remove unsupported edges", &remove_unsupported);
  }
  void process();
};

class DetectLinesNode:public Node {
  config c;

  public:
  bool use_linear_neighboorhood=false;

  DetectLinesNode(NodeManager& manager):Node(manager) {
    add_input("edge_points", {TT_point_collection, TT_linear_ring_collection});
    add_output("edge_segments", TT_line_string_collection);
  }

  void gui(){
    ImGui::InputFloat("Dist thres", &c.linedetect_dist_threshold, 0.01, 1);
    ImGui::InputInt("Segment cnt min", &c.linedetect_min_segment_count);
    ImGui::InputInt("K", &c.linedetect_k);
    ImGui::Checkbox("Use linear neighbourhood for ring input", &use_linear_neighboorhood);
  }
  void process();
};

class ClassifyEdgePointsNode:public Node {
  config c;

  public:
  ClassifyEdgePointsNode(NodeManager& manager):Node(manager) {
    add_output("edge_points", TT_any);
    add_output("edge_points_vec3f", TT_vec3f);
    add_input("points", TT_any);
  }

  void gui(){
    ImGui::InputInt("Jump cnt min", &c.classify_jump_count_min);
    ImGui::InputInt("Jump cnt max", &c.classify_jump_count_max);
    ImGui::InputFloat("Line dist", &c.classify_line_dist, 0.01, 1);
    ImGui::InputFloat("Elevation jump", &c.classify_jump_ele, 0.01, 1);
  }

  void process();
};

class ComputeMetricsNode:public Node {
  config c;

  public:
  ComputeMetricsNode(NodeManager& manager):Node(manager) {
    add_output("points", TT_any);
    add_input("points", TT_point_collection); // change to Feature
    add_output("plane_id", TT_vec1i);
    add_output("is_wall", TT_vec1i);
    add_output("is_horizontal", TT_vec1i);
    add_output("line_dist", TT_vec1f);
    add_output("jump_count", TT_vec1f);
    add_output("jump_ele", TT_vec1f);
    add_output("points_c", TT_point_collection);
  }

  void gui(){
    ImGui::InputInt("K estimate normal ", &c.metrics_normal_k);
    ImGui::InputInt("Plane min points", &c.metrics_plane_min_points);
    ImGui::InputFloat("Plane epsilon", &c.metrics_plane_epsilon, 0.01, 1);
    ImGui::InputFloat("Plane normal thres", &c.metrics_plane_normal_threshold, 0.01, 1);
    ImGui::InputFloat("Wall angle thres", &c.metrics_is_wall_threshold, 0.01, 1);
    ImGui::InputInt("K linefit", &c.metrics_k_linefit);
    ImGui::InputInt("K jumpedge", &c.metrics_k_jumpcnt_elediff);
  }

  void process();
};

class LASInPolygonsNode:public Node {
  std::vector<PointCollection> point_clouds;
  LinearRingCollection polygons;

  public:
  int footprint_id=0;
  // char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
  char las_filepath[256] = "";
  LASInPolygonsNode(NodeManager& manager):Node(manager) {
    add_input("polygons", TT_linear_ring_collection);
    add_output("point_clouds", TT_point_collection_list);
    add_output("points", TT_point_collection);
    add_output("footprint", TT_linear_ring);
  }

  void gui() {
    ImGui::InputText("LAS file path", las_filepath, IM_ARRAYSIZE(las_filepath));
    if (ImGui::SliderInt("#", &footprint_id, 0, polygons.size()-1)) {
      // if(run_on_change) {
      //   manager.run(*this);
      // } else {
      if (footprint_id < polygons.size() && footprint_id >= 0) {
        notify_children();
        outputs("points").set(point_clouds[footprint_id]);
        outputs("point_clouds").set(point_clouds);
        outputs("footprint").set(polygons[footprint_id]);
        propagate_outputs();
      } else { footprint_id = polygons.size()-1; }
    }
  }
  void process();
};

struct LineCluster {
  //                source_id, target_id, is_footprint
  typedef std::tuple<size_t, size_t, bool> SegmentTuple; 
  Vector_2 ref_vec; // direction of reference line for this cluster (exact_exact arrangent traits)
  Point_2 ref_point; // extreme point on reference line (ie most left or most right point)
  vec1f vertices; // stored as distances from ref_point in direction of ref_vec
  std::vector<SegmentTuple> segments;
};

struct ValueCluster {
  double angle;
  double distance;
  std::vector<size_t> idx;
};

class RegulariseLinesNode:public Node {
  static constexpr double pi = 3.14159265358979323846;
  float dist_threshold = 0.5;
  float angle_threshold = 0.1;//5*(pi/180);

  public:
  RegulariseLinesNode(NodeManager& manager):Node(manager) {
    add_input("edge_segments", TT_line_string_collection);
    add_input("footprint", TT_linear_ring);
    add_output("edges_out", TT_line_string_collection);
    add_output("cluster_labels", TT_vec1i);
    add_output("footprint_labels", TT_vec1i);
    add_output("line_clusters", TT_any); // ie a LineCluster
    add_output("tmp_vec3f", TT_vec3f);
  }

  void gui(){
    ImGui::DragFloat("Distance threshold", &dist_threshold, 0.1, 0);
    ImGui::DragFloat("Angle threshold", &angle_threshold, 0.01, 0.01, pi);
  }
  void process();
};

class LOD13GeneratorNode:public Node {
  public:
  float step_height_threshold = 1.0;
  float zrange_threshold = 0.2;
  bool merge_segid = true;
  bool merge_zrange = false;
  bool merge_step_height = true;
  bool merge_unsegmented = false;
  bool merge_dangling_egdes = false;
  LOD13GeneratorNode(NodeManager& manager):Node(manager) {
    add_input("point_clouds", TT_point_collection_list);
    add_input("polygons", TT_linear_ring_collection);
    add_output("decomposed_footprints", TT_linear_ring_collection);
    add_output("attributes", TT_attribute_map_f);
  }

  void gui(){
    ImGui::InputFloat("Step height", &step_height_threshold, 0.1, 1);
    ImGui::DragFloat("zrange_threshold", &zrange_threshold, 0.1);
    ImGui::Checkbox("merge_segid", &merge_segid);
    ImGui::Checkbox("merge_zrange", &merge_zrange);
    ImGui::Checkbox("merge_step_height", &merge_step_height);
    ImGui::Checkbox("merge_unsegmented", &merge_unsegmented);
    ImGui::Checkbox("merge_dangling_egdes", &merge_dangling_egdes);
  }
  void process();
};