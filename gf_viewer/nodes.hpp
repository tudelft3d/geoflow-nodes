#include "imgui.h"
#include "gloo.h"
#include "geoflow.hpp"
#include "point_edge.h"

class ComputeMetricsNode:public Node {
  config c;

  public:
  ComputeMetricsNode(NodeManager& manager):Node(manager, "ComputeMetrics") {
    add_output("pnl_vec", TT_vec_pnl);
    add_input("pnl_vec", TT_vec_pnl);
  }

  void gui(){
    ImGui::InputInt("Plane min points", &c.metrics_plane_min_points);
    ImGui::InputFloat("Plane epsilon", &c.metrics_plane_epsilon, 0.01, 1);
    ImGui::InputFloat("Plane normal thres", &c.metrics_plane_normal_threshold, 0.01, 1);
    ImGui::InputFloat("Wall angle thres", &c.metrics_is_wall_threshold, 0.01, 1);
    ImGui::InputInt("K linefit", &c.metrics_k_linefit);
    ImGui::InputInt("K jumpedge", &c.metrics_k_jumpcnt_elediff);
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto points = std::any_cast<PNL_vector>(get_value("pnl_vec"));
    compute_metrics(points, c);
    set_value("pnl_vec", points);
    // std::cout << "end NumberNode::process()" << "\n";
  }
};

class PointsInFootprintNode:public Node {
  config c;
  int footprint_id=0;
  std::vector<bg::model::polygon<point_type>> footprints;
  std::vector<PNL_vector> points_vec;

  public:
  PointsInFootprintNode(NodeManager& manager):Node(manager, "PointsInFootprint") {
    add_output("pnl_vec", TT_vec_pnl);
    // add_output("footprints", TT_vec_pnl);
  }

  void gui(){
    if (ImGui::SliderInt("#", &footprint_id, 0, footprints.size()-1))
      set_value("pnl_vec", points_vec[footprint_id]);
  }

  void process(){
    std::string las_path = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
    std::string csv_path = "/Users/ravi/surfdrive/data/step-edge-detector/rdam_sample_0.csv";
    
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto csv_footprints = std::ifstream(csv_path);
    // auto csv_footprints = std::ifstream("/Users/ravi/surfdrive/data/step-edge-detector/bag_amersfoort_0.csv");
    std::string column_names, row;
    std::getline(csv_footprints, column_names);
    while (std::getline(csv_footprints, row)) {
        bg::model::polygon<point_type> bag_polygon;
        bg::read_wkt(row, bag_polygon);
        bg::unique(bag_polygon);
        footprints.push_back(bag_polygon);
    } csv_footprints.close();

    pc_in_footprint(las_path, footprints, points_vec);
    set_value("pnl_vec", points_vec[footprint_id]);
    // std::cout << footprints.size() << "\n";
  }
};