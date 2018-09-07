#include "imgui.h"
#include "gloo.h"
#include "geoflow.hpp"
#include "point_edge.h"
#include "earcut.hpp"

class ExtruderNode:public Node {

  public:
  ExtruderNode(NodeManager& manager):Node(manager, "Extruder") {
    add_input("arrangement", TT_any);
    add_output("triangles_vec3f", TT_vec3f);
    add_output("labels_vec1i", TT_vec1i); // 0==ground, 1==roof, 2==outerwall, 3==innerwall
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    // auto polygons = std::any_cast<std::vector<vec2f>>(get_value("polygons"));
    // auto elevations = std::any_cast<std::vector<float>>(get_value("elevations"));
    auto arr = std::any_cast<Arrangement_2>(get_value("arrangement"));

    vec3f triangles;
    vec1i labels;
    using N = uint32_t;

    for (auto face: arr.face_handles()){
      if(face->data().is_finite) {
        vec2f polygon, face_triangles;
        arrangementface_to_polygon(face, polygon);
        std::vector<N> indices = mapbox::earcut<N>(std::vector<vec2f>({polygon}));
        for(auto i : indices) {
          face_triangles.push_back({polygon[i]});
        }
        for (auto& vertex : face_triangles) {
          //add to ground face
          triangles.push_back({vertex[0], vertex[1], 0});
          labels.push_back(0);
        } 
        for (auto& vertex : face_triangles) {
          //add to elevated (roof) face
          triangles.push_back({vertex[0], vertex[1], face->data().elevation_avg});
          labels.push_back(1);
        } 
      }
    }

    for (auto edge : arr.edge_handles()) {
      // skip if faces on both sides of this edge are not finite
      bool left_finite = edge->twin()->face()->data().is_finite;
      bool right_finite = edge->face()->data().is_finite;
      if (left_finite || right_finite) {
        int wall_label = 2;
        if (left_finite && right_finite)
          wall_label = 3;

        auto h1 = edge->face()->data().elevation_avg;
        auto h2 = edge->twin()->face()->data().elevation_avg;
        // push 2 triangles to form the quad between lower and upper edges
        // notice that this is not always topologically correct, but fine for visualisation
        
        // define four points of the quad between upper and lower edge
        std::array<float,3> l1,l2,u1,u2;
        l1 = {
          float(CGAL::to_double(edge->source()->point().x())),
          float(CGAL::to_double(edge->source()->point().y())),
          h1
        };
        l2 = {
          float(CGAL::to_double(edge->target()->point().x())),
          float(CGAL::to_double(edge->target()->point().y())),
          h1
        };
        u1 = {
          float(CGAL::to_double(edge->source()->point().x())),
          float(CGAL::to_double(edge->source()->point().y())),
          h2
        };
        u2 = {
          float(CGAL::to_double(edge->target()->point().x())),
          float(CGAL::to_double(edge->target()->point().y())),
          h2
        };

        // 1st triangle
        triangles.push_back(l1);
        labels.push_back(wall_label);
        triangles.push_back(l2);
        labels.push_back(wall_label);
        triangles.push_back(u1);
        labels.push_back(wall_label);
        // 2nd triangle
        triangles.push_back(u1);
        labels.push_back(wall_label);
        triangles.push_back(u2);
        labels.push_back(wall_label);
        triangles.push_back(l2);
        labels.push_back(wall_label);
      }
    }

    set_value("triangles_vec3f", triangles);
    set_value("labels_vec1i", labels);
  }
};

class ProcessArrangementNode:public Node {

  public:
  ProcessArrangementNode(NodeManager& manager):Node(manager, "ProcessArrangement") {
    add_input("arrangement", TT_any);
    add_input("points", TT_any);
    add_output("arrangement", TT_any);
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto points = std::any_cast<PNL_vector>(get_value("points"));
    auto arr = std::any_cast<Arrangement_2>(get_value("arrangement"));

    process_arrangement(points, arr);
    
    set_value("arrangement", arr);
  }
};

class BuildArrangementNode:public Node {
  float footprint_simp_thres=0;

  public:
  BuildArrangementNode(NodeManager& manager):Node(manager, "BuildArrangement") {
    add_input("edge_segments", TT_any);
    add_input("footprint", TT_any);
    add_output("arrangement", TT_any);
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
        if(face->data().is_finite){
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
    set_value("arrangement", arr);
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

    vec3f edge_points_vec3f;
    for(auto& p : edge_points) {
        std::array<float,3> a = {{
          float(p.x()), 
          float(p.y()), 
          float(p.z())
        }};
        edge_points_vec3f.push_back(a);
      }
    set_value("edge_points_vec3f", edge_points_vec3f);
  }
};

class ComputeMetricsNode:public Node {
  config c;

  public:
  ComputeMetricsNode(NodeManager& manager):Node(manager, "ComputeMetrics") {
    add_output("points", TT_any);
    add_input("points", TT_any);
    add_output("plane_id", TT_vec1i);
    add_output("is_wall", TT_vec1i);
    add_output("line_dist", TT_vec1f);
    add_output("jump_count", TT_vec1f);
    add_output("jump_ele", TT_vec1f);
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
    vec1f line_dist, jump_count, jump_ele;
    vec1i plane_id, is_wall;
    for(auto& p : points){
      plane_id.push_back(p.get<2>());
      is_wall.push_back(p.get<3>());
      line_dist.push_back(p.get<4>());
      jump_count.push_back(p.get<5>());
      jump_ele.push_back(p.get<7>());
    }
    set_value("points", points);
    set_value("plane_id", plane_id);
    set_value("is_wall", is_wall);
    set_value("line_dist", line_dist);
    set_value("jump_count", jump_count);
    set_value("jump_ele", jump_ele);

  }
};

class PointsInFootprintNode:public Node {
  config c;
  int footprint_id=0;
  bool isInitialised = false;
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
    if (ImGui::SliderInt("#", &footprint_id, 0, footprints.size()-1)) {
      // manager.run(*this);
      notify_children();
      set_value("points", points_vec[footprint_id]);
      set_value("points_vec3f", points_vec3f[footprint_id]);
      // outputTerminals["points_vec3f"]->propagate();
      set_value("footprint", footprints[footprint_id]);
      propagate_outputs();
    }
  }

  void process(){
    if(!isInitialised) {
      // std::string las_path = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
      std::string las_path = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
      // std::string csv_path = "/Users/ravi/surfdrive/data/step-edge-detector/rdam_sample_0.csv";
      std::string csv_path = "/Users/ravi/surfdrive/data/step-edge-detector/bag_amersfoort_0.csv";
      
      // Set up vertex data (and buffer(s)) and attribute pointers
      auto csv_footprints = std::ifstream(csv_path);
      
      std::string column_names, row;
      std::getline(csv_footprints, column_names);
      point_type centroid; // we'll set the origin to the centroid of the first footprint
      bool read_first=false;
      while (std::getline(csv_footprints, row)) {
          bg::model::polygon<point_type> bag_polygon;
          bg::read_wkt(row, bag_polygon);
          bg::unique(bag_polygon);

          if(!read_first) {
            bg::centroid(bag_polygon, centroid);
            read_first=true;
          }
          footprints.push_back(bag_polygon);
      } csv_footprints.close();

      pc_in_footprint(las_path, footprints, points_vec);

      for (auto& fp : footprints) {
        for (auto& p : fp.outer())
            bg::subtract_point(p, centroid);
      }

      for(auto& pc : points_vec) {
        for(auto& p : pc) {
          p.get<0>() = Point(
            p.get<0>().x()-bg::get<0>(centroid), 
            p.get<0>().y()-bg::get<1>(centroid),
            p.get<0>().z());
        }
      }

      for(auto& pc : points_vec) {
        vec3f v;
        for(auto& p : pc) {
          std::array<float,3> a = {
            float(p.get<0>().x()), 
            float(p.get<0>().y()), 
            float(p.get<0>().z())
          };
          v.push_back(a);
        }
        points_vec3f.push_back(v);
      }
      std::cout << footprints.size() << "\n";
      isInitialised = true;
    }
    set_value("points", points_vec[footprint_id]);
    set_value("points_vec3f", points_vec3f[footprint_id]);
    set_value("footprint", footprints[footprint_id]);
  }
};