#include "imgui.h"
#include "gloo.h"
#include "geoflow.hpp"
#include "point_edge.h"
#include "earcut.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>

typedef std::array<float,3> vertex;
vertex get_normal(vertex v0, vertex v1, vertex v2) {
    // assuming ccw winding order
    auto a = glm::make_vec3(v0.data());
    auto b = glm::make_vec3(v1.data());
    auto c = glm::make_vec3(v2.data());
    auto n = glm::cross(b-a, c-b);
    return {n.x,n.y,n.z};
}

class ExtruderNode:public Node {

  public:
  ExtruderNode(NodeManager& manager):Node(manager, "Extruder") {
    add_input("arrangement", TT_any);
    add_output("cell_id_vec1i", TT_vec1i);
    add_output("triangles_vec3f", TT_vec3f);
    add_output("normals_vec3f", TT_vec3f);
    add_output("labels_vec1i", TT_vec1i); // 0==ground, 1==roof, 2==outerwall, 3==innerwall
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    // auto polygons = std::any_cast<std::vector<vec2f>>(get_value("polygons"));
    // auto elevations = std::any_cast<std::vector<float>>(get_value("elevations"));
    auto arr = std::any_cast<Arrangement_2>(get_value("arrangement"));

    vec3f triangles, normals;
    vec1i cell_id_vec1i;
    vec1i labels;
    using N = uint32_t;

    size_t cell_id=0;
    for (auto face: arr.face_handles()){
      if(face->data().is_finite) {
        cell_id++;
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
          normals.push_back({0,0,-1});
          cell_id_vec1i.push_back(cell_id);
        } 
        for (auto& vertex : face_triangles) {
          //add to elevated (roof) face
          triangles.push_back({vertex[0], vertex[1], face->data().elevation_avg});
          labels.push_back(1);
          normals.push_back({0,0,1});
          cell_id_vec1i.push_back(cell_id);
        } 
      }
    }

    vertex n;
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
        triangles.push_back(u1);
        labels.push_back(wall_label);
        triangles.push_back(l2);
        labels.push_back(wall_label);
        triangles.push_back(l1);
        labels.push_back(wall_label);

        n = get_normal(u1,l2,l1);
        normals.push_back(n);
        normals.push_back(n);
        normals.push_back(n);

        cell_id_vec1i.push_back(0);
        cell_id_vec1i.push_back(0);
        cell_id_vec1i.push_back(0);

        // 2nd triangle
        triangles.push_back(u1);
        labels.push_back(wall_label);
        triangles.push_back(u2);
        labels.push_back(wall_label);
        triangles.push_back(l2);
        labels.push_back(wall_label);

        n = get_normal(u1,u2,l2);
        normals.push_back(n);
        normals.push_back(n);
        normals.push_back(n);

        cell_id_vec1i.push_back(0);
        cell_id_vec1i.push_back(0);
        cell_id_vec1i.push_back(0);
      }
    }

    set_value("normals_vec3f", normals);
    set_value("cell_id_vec1i", cell_id_vec1i);
    set_value("triangles_vec3f", triangles);
    set_value("labels_vec1i", labels);
  }
};

class SimplifyFootprintNode:public Node {

  float threshold_stop_cost=0.1;

  public:
  SimplifyFootprintNode(NodeManager& manager):Node(manager, "SimplifyFootprint") {
    add_input("footprint", TT_any);
    add_output("footprint", TT_any);
    add_output("footprint_vec3f", TT_vec3f);
  }

  void gui(){
    if(ImGui::DragFloat("stop cost", &threshold_stop_cost,0.01)) {
      manager.run(*this);
    }
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto footprint = std::any_cast<bg::model::polygon<point_type>>(get_value("footprint"));

    namespace PS = CGAL::Polyline_simplification_2;
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_2 Point_2;
    typedef CGAL::Polygon_2<K>                   Polygon_2;
    typedef PS::Stop_below_count_ratio_threshold Stop_count_ratio;
    typedef PS::Stop_above_cost_threshold        Stop_cost;
    typedef PS::Squared_distance_cost            Cost;

    Polygon_2 polygon;
    Cost cost;

    for (auto p : footprint.outer()) {
      polygon.push_back(Point_2(bg::get<0>(p), bg::get<1>(p)));
    }
    polygon.erase(polygon.vertices_end()-1); // remove repeated point from the boost polygon
    
    // polygon = PS::simplify(polygon, cost, Stop_count_ratio(0.5));

    polygon = PS::simplify(polygon, cost, Stop_cost(threshold_stop_cost));
    
    vec3f footprint_vec3f;
    bg::model::polygon<point_type> footprint_out;
    footprint.outer().clear();
    for (auto v = polygon.vertices_begin(); v!=polygon.vertices_end(); v++){
      footprint_vec3f.push_back({float(v->x()),float(v->y()),0});
      footprint_out.outer().push_back(point_type(v->x(),v->y()));
    }
    footprint_out.outer().push_back(point_type(polygon.vertices_begin()->x(),polygon.vertices_begin()->y()));
    set_value("footprint_vec3f", footprint_vec3f);
    set_value("footprint", footprint_out);
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
  bool run_on_change=false;
  bool isInitialised = false;
  std::vector<bg::model::polygon<point_type>> footprints;
  std::vector<PNL_vector> points_vec;
  std::vector<vec3f> points_vec3f;
  std::vector<vec3f> footprints_vec3f;

  public:
  PointsInFootprintNode(NodeManager& manager):Node(manager, "PointsInFootprint") {
    add_output("points", TT_any);
    add_output("points_vec3f", TT_vec3f);
    add_output("footprint", TT_any);
    add_output("footprint_vec3f", TT_vec3f);
  }

  void gui(){
    ImGui::Checkbox("Run on change", &run_on_change);
    if (ImGui::SliderInt("#", &footprint_id, 0, footprints.size()-1)) {
      if(run_on_change) {
        manager.run(*this);
      } else {
        notify_children();
        set_value("points", points_vec[footprint_id]);
        set_value("points_vec3f", points_vec3f[footprint_id]);
        set_value("footprint", footprints[footprint_id]);
        set_value("footprint_vec3f", footprints_vec3f[footprint_id]);
        propagate_outputs();
      }
    }
  }

  void process(){
    if(!isInitialised) {
      std::string las_path = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
      // std::string las_path = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
      std::string csv_path = "/Users/ravi/surfdrive/data/step-edge-detector/rdam_sample_0.csv";
      // std::string csv_path = "/Users/ravi/surfdrive/data/step-edge-detector/bag_amersfoort_0.csv";
      
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
        vec3f fp_vec3f;
        for (auto& p : fp.outer()){
            bg::subtract_point(p, centroid);
            fp_vec3f.push_back({float(bg::get<0>(p)), float(bg::get<1>(p)), 0});
        }
        footprints_vec3f.push_back(fp_vec3f);
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
    set_value("footprint_vec3f", footprints_vec3f[footprint_id]);
  }
};

class RegulariseLinesNode:public Node {
  static constexpr double pi = 3.14159265358979323846;
  float dist_threshold = 2;
  float angle_threshold = 10*(pi/180);

  public:
  RegulariseLinesNode(NodeManager& manager):Node(manager, "RegulariseLines") {
    add_input("edge_segments", TT_any);
    add_input("footprint_vec3f", TT_vec3f);
    add_output("edges_out", TT_any);
    add_output("edges_out_vec3f", TT_vec3f);
  }

  void gui(){
    ImGui::DragFloat("Distance threshold", &dist_threshold, 0.1, 0);
    ImGui::DragFloat("Angle threshold", &angle_threshold, 0.1, 0.1, pi);
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto edges = std::any_cast<std::vector<std::pair<Point,Point>>>(get_value("edge_segments"));
    auto footprint_vec3f = std::any_cast<vec3f>(get_value("footprint_vec3f"));
    typedef CGAL::Exact_predicates_inexact_constructions_kernel::Point_2 Point_2;

    //compute midpoint and direction for each segment
    typedef std::tuple<double, Point_2, double, double, bool> linetype; // angle, midpoint, distance in angle cluster, elevation, is_footprint
    std::vector<linetype> lines;
    // add non-footprint lines
    for(auto edge : edges) {
      auto v = edge.second-edge.first;
      auto p_ = edge.first + v/2;
      auto p = Point_2(p_.x(),p_.y());
      auto angle = std::atan2(v.x(),v.y());
      if (angle < 0) angle += pi;
      lines.push_back(std::make_tuple(angle,p,0,p_.z(), false));
    }
    // add footprint edges
    // footprint_vec3f.push_back(footprint_vec3f[0]); //repeat first point as last
    // for(size_t i=0; i<footprint_vec3f.size()-1; i++) {
    //   auto p_first = Point_2(footprint_vec3f[i+0][0], footprint_vec3f[i+0][1]);
    //   auto p_second = Point_2(footprint_vec3f[i+1][0], footprint_vec3f[i+1][1]);
    //   auto v = p_second - p_first;

    //   auto p_ = p_first + v/2;
    //   auto p = Point_2(p_.x(),p_.y());
    //   auto angle = std::atan2(v.x(),v.y());

    //   lines.push_back(std::make_tuple(angle,p,0,0, true));
    // }

    //sort by angle, smallest on top
    std::sort(lines.begin(), lines.end(), [](linetype a, linetype b) {
        return std::get<0>(a) < std::get<0>(b);   
    });
    //cluster by angle difference
    std::vector<std::vector<linetype>> angle_clusters(1);
    auto last_angle = std::get<0>(lines[0]);
    for(auto line : lines ) {
      if((std::get<0>(line) - last_angle) < angle_threshold)
        angle_clusters.back().push_back(line);
      else {
        angle_clusters.resize(angle_clusters.size()+1);
        angle_clusters.back().push_back(line);
        }
      last_angle=std::get<0>(line);
    }

    // snap to average angle in each cluster
    vec3f directions_before, directions_after;
    vec1i angles;
    int cluster_id=0;
    for(auto& cluster:angle_clusters) {
      double sum=0;
      for(auto& line : cluster) {
        sum+=std::get<0>(line);
      }
      double average_angle = sum/cluster.size();
      for(auto& line : cluster) {
        auto angle = std::get<0>(line);
        std::get<0>(line)=average_angle;
      }
      cluster_id++;
    }

    vec1f distances;
    // snap nearby lines that are close
    std::vector<std::vector<linetype>> dist_clusters;
    for(auto& cluster:angle_clusters) {
      // compute vec orthogonal to lines in this cluster
      auto angle = std::get<0>(cluster[0]);
      Vector_2 n(-1.0, std::tan(angle));
      n = n/std::sqrt(n.squared_length()); // normalize
      // compute distances along n wrt to first line in cluster
      auto p = std::get<1>(cluster[0]);
      for(auto& line : cluster) {
        auto q = std::get<1>(line);
        auto v = p-q;
        std::get<2>(line) = v*n;
        distances.push_back(v*n);
      }
      // sort by distance, ascending
      std::sort(cluster.begin(), cluster.end(), [](linetype a, linetype b){
          return std::get<2>(a) < std::get<2>(b);
      });
      // cluster nearby lines using separation threshold
      double last_dist = std::get<2>(cluster[0]);
      dist_clusters.resize(dist_clusters.size()+1);
      for(auto& line : cluster) {
        double dist_diff = std::get<2>(line) - last_dist;
        if (dist_diff < dist_threshold) {
          dist_clusters.back().push_back(line);
        } else {
          dist_clusters.resize(dist_clusters.size()+1);
          dist_clusters.back().push_back(line);
        }
        last_dist = std::get<2>(line);
      }
    }

    // compute one line per dist cluster => the one with the highest elevation
    vec3f edges_out_vec3f;
    std::vector<std::pair<Point,Point>> edges_out;
    for(auto& cluster : dist_clusters) {
      double max_z=0;
      linetype high_line;
      for(auto& line : cluster) {
        auto z = std::get<3>(line);
        if(z > max_z) {
          max_z = z;
          high_line = line;
        }
      }
      // compute vec orthogonal to lines in this cluster
      double angle = std::get<0>(high_line);
      auto p0 = std::get<1>(high_line);
      // Vector_2 n(-1.0, std::tan(angle));
      // n = n/std::sqrt(n.squared_length()); // normalize
      Vector_2 l(std::tan(angle),1.0);
      l = l/std::sqrt(l.squared_length()); // normalize
      auto p_center = p0;// + average_dist*n;
      auto p_begin = p_center + 3*l;
      auto p_end = p_center - 3*l;
      edges_out_vec3f.push_back({float(p_begin.x()), float(p_begin.y()), 0});
      edges_out_vec3f.push_back({float(p_end.x()), float(p_end.y()), 0});
      edges_out.push_back(std::make_pair(
        Point(float(p_begin.x()), float(p_begin.y()), 0),
        Point(float(p_end.x()), float(p_end.y()), 0)
      ));
    }

    set_value("edges_out", edges_out);
    set_value("edges_out_vec3f", edges_out_vec3f);
  }
};