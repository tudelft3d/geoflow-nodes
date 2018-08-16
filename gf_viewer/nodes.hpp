#include "imgui.h"
#include "gloo.h"
#include "geoflow.hpp"
#include "point_edge.h"

class BuildArrangementNode:public Node {
  float footprint_simp_thres=0;

  public:
  BuildArrangementNode(NodeManager& manager):Node(manager, "BuildArrangement") {
    add_input("edge_segments", TT_any);
    add_input("footprint", TT_any);
    add_output("arr_segments_vec3f", TT_vec3f);
  }

  void gui(){
    ImGui::InputFloat("Footprint simp", &footprint_simp_thres, 0.01, 1);
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto edge_segments = std::any_cast<std::vector<std::pair<Point,Point>>>(get_value("edge_segments"));
    auto fp = std::any_cast<bg::model::polygon<point_type>>(get_value("footprint"));
    Arrangement_2 arr;
    arr.clear();
    build_arrangement(fp, edge_segments, arr);
    vec3f polygons;
    for (auto face: arr.face_handles()){
        if(face->data()==1){
            auto he = face->outer_ccb();
            auto first = he;

            while(true){
                polygons.push_back({
                  float(CGAL::to_double(he->source()->point().x())),
                  float(CGAL::to_double(he->source()->point().y())),
                  0
                });
                polygons.push_back({
                  float(CGAL::to_double(he->target()->point().x())),
                  float(CGAL::to_double(he->target()->point().y())),
                  0
                });

                he = he->next();
                if (he==first) break;
            }
            polygons.push_back({
              float(CGAL::to_double(he->source()->point().x())),
              float(CGAL::to_double(he->source()->point().y())),
              0
            });
            polygons.push_back({
              float(CGAL::to_double(he->target()->point().x())),
              float(CGAL::to_double(he->target()->point().y())),
              0
            });
        }
    }
    set_value("arr_segments_vec3f", polygons);
  }
};

class DetectLinesNode:public Node {
  config c;

  public:
  DetectLinesNode(NodeManager& manager):Node(manager, "DetectLines") {
    add_input("edge_points", TT_any);
    add_output("edge_segments_vec3f", TT_vec3f);
    add_output("edge_segments", TT_any);
  }

  void gui(){
    ImGui::InputFloat("Dist thres", &c.linedetect_dist_threshold, 0.01, 1);
    ImGui::InputInt("Segment cnt min", &c.linedetect_min_segment_count);
    ImGui::InputInt("K", &c.linedetect_k);
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto edge_points = std::any_cast<std::vector<linedect::Point>>(get_value("edge_points"));
    std::vector<std::pair<Point,Point>> edge_segments;
    detect_lines(edge_segments, edge_points, c);
    set_value("edge_segments", edge_segments);
    vec3f edge_segments_vec3f;
    for (auto s : edge_segments){
      edge_segments_vec3f.push_back({
        float(s.first.x()),
        float(s.first.y()),
        float(s.first.z())
      });
      edge_segments_vec3f.push_back({
        float(s.second.x()),
        float(s.second.y()),
        float(s.second.z())
      });
    }
    set_value("edge_segments_vec3f", edge_segments_vec3f);
  }
};

class ClassifyEdgePointsNode:public Node {
  config c;

  public:
  ClassifyEdgePointsNode(NodeManager& manager):Node(manager, "ClassifyEdgePoints") {
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

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto points = std::any_cast<PNL_vector>(get_value("points"));
    std::vector<linedect::Point> edge_points;
    classify_edgepoints(edge_points, points, c);
    set_value("edge_points", edge_points);
  }
};

class ComputeMetricsNode:public Node {
  config c;

  public:
  ComputeMetricsNode(NodeManager& manager):Node(manager, "ComputeMetrics") {
    add_output("points", TT_any);
    add_input("points", TT_any);
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
    auto points = std::any_cast<PNL_vector>(get_value("points"));
    compute_metrics(points, c);
    set_value("points", points);

  }
};

class PointsInFootprintNode:public Node {
  config c;
  int footprint_id=0;
  std::vector<bg::model::polygon<point_type>> footprints;
  std::vector<PNL_vector> points_vec;
  std::vector<vec3f> points_vec3f;

  public:
  PointsInFootprintNode(NodeManager& manager):Node(manager, "PointsInFootprint") {
    add_output("points", TT_any);
    add_output("points_vec3f", TT_vec3f);
    add_output("footprint", TT_any);
  }

  void gui(){
    if (ImGui::SliderInt("#", &footprint_id, 0, footprints.size()-1))
      set_value("points", points_vec[footprint_id]);
      set_value("points_vec3f", points_vec3f[footprint_id]);
      set_value("footprint", footprints[footprint_id]);
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
    set_value("points", points_vec[footprint_id]);
    
    for(auto& pc : points_vec) {
      vec3f v;
      for(auto& p : pc) {
        std::array<float,3> a = {{
          float(p.get<0>().x()), 
          float(p.get<0>().y()), 
          float(p.get<0>().z())
        }};
        v.push_back(a);
      }
      points_vec3f.push_back(v);
    }
    set_value("points_vec3f", points_vec3f[footprint_id]);
    set_value("footprint", footprints[footprint_id]);
    std::cout << footprints.size() << "\n";
  }
};