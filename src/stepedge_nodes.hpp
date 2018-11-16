#include "imgui.h"
#include "gloo.h"
#include "geoflow.hpp"
#include "point_edge.h"
#include "ptinpoly.h"

#include "earcut.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include <boost/tuple/tuple.hpp>

#include <filesystem>
#include <numeric>

namespace fs=std::filesystem;

typedef std::array<float,3> vertex;
vertex get_normal(vertex v0, vertex v1, vertex v2) {
    // assuming ccw winding order
    auto a = glm::make_vec3(v0.data());
    auto b = glm::make_vec3(v1.data());
    auto c = glm::make_vec3(v2.data());
    auto n = glm::cross(b-a, c-b);
    return {n.x,n.y,n.z};
}


// 2D alpha shapes
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Projection_traits_xy_3.h>

class AlphaShapeNode:public Node {

  public:
  float thres_alpha = 0.7;
  AlphaShapeNode(NodeManager& manager):Node(manager) {
    // add_input("points", TT_any);
    add_input("points", TT_any);
    add_output("edge_points", TT_any);
    add_output("edge_points_vec3f", TT_vec3f);
  }

  void gui(){
    ImGui::InputFloat("Alpha", &thres_alpha, 0.01, 1);
  }

  void process(){
    typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
    typedef CGAL::Projection_traits_xy_3<K>								       Gt;
    typedef K::FT                                                FT;
    // typedef K::Point_2                                           Point;
    // typedef K::Segment_2                                         Segment;
    typedef CGAL::Alpha_shape_vertex_base_2<Gt>                  Vb;
    typedef CGAL::Alpha_shape_face_base_2<Gt>                    Fb;
    typedef CGAL::Triangulation_data_structure_2<Vb,Fb>          Tds;
    typedef CGAL::Delaunay_triangulation_2<Gt,Tds>               Triangulation_2;
    typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;
    typedef Triangulation_2::Vertex_handle                       VertexHandle;

    // Set up vertex data (and buffer(s)) and attribute pointers
    auto points = std::any_cast<PNL_vector>(get_value("points"));
   
    std::unordered_map<int, std::vector<Point>> points_per_segment;
    for (auto& p : points) {
      if (boost::get<2>(p)==0) // unsegmented
        continue;
      if (boost::get<3>(p)) // classified as wall
        continue;
      points_per_segment[boost::get<2>(p)].push_back(boost::get<0>(p));
    }
    std::vector<Point> edge_points;
    vec3f edge_points_vec3f;
    for (auto& it : points_per_segment ) {
      auto points = it.second;
      Triangulation_2 T;
      T.insert(points.begin(), points.end());
      Alpha_shape_2 A(T,
                  FT(thres_alpha),
                  Alpha_shape_2::GENERAL);
      // thres_alpha = *A.find_optimal_alpha(1);
      A.set_alpha(FT(thres_alpha));
      std::vector<std::pair<VertexHandle, VertexHandle>> alpha_edges;
      for (auto it = A.alpha_shape_edges_begin(); it!=A.alpha_shape_edges_end(); it++) {
        auto face = (it)->first;
        auto i = (it)->second;
        alpha_edges.push_back(std::make_pair(face->vertex(T.cw(i)), face->vertex(T.ccw(i))));
      }
      // find connecting edges...
      // ...

      for (auto it = A.alpha_shape_vertices_begin(); it!=A.alpha_shape_vertices_end(); it++) {
        auto p = (*it)->point();
        edge_points.push_back(p);
        edge_points_vec3f.push_back({float(p.x()), float(p.y()), float(p.z())});
      }
    }
    
    set_value("edge_points", edge_points);
    set_value("edge_points_vec3f", edge_points_vec3f);
  }
};

class PolygonExtruderNode:public Node {

  public:
  PolygonExtruderNode(NodeManager& manager):Node(manager) {
    add_input("polygons", TT_any);
    add_input("point_clouds", TT_any);
    add_output("polygons_extruded", TT_any);
  }

  void process(){
    auto polygons = std::any_cast<Feature>(get_value("polygons"));
    auto point_clouds = std::any_cast<Feature>(get_value("point_clouds"));

    if(polygons.geom.size()!=point_clouds.geom.size()) return;

    polygons.attr["height"] = vec1f();
    for (auto& point_cloud : point_clouds.geom) {
      double sum_elevation = 0;
      for (auto& p : point_cloud) {
        sum_elevation += p[2];
      }
      polygons.attr["height"].push_back(sum_elevation/point_cloud.size());
    }
    set_value("polygons_extruded", polygons);
  }
};

class Arr2FeatureNode:public Node {

  public:
  Arr2FeatureNode(NodeManager& manager):Node(manager) {
    add_input("arrangement", TT_any);
    add_output("decomposed_footprint", TT_any);
  }

  void process(){
    auto arr = std::any_cast<Arrangement_2>(get_value("arrangement"));

    Feature decomposed_footprint;
    decomposed_footprint.type=geoflow::line_loop;
    for (auto face: arr.face_handles()){
      if(face->data().is_finite) {
        vec2f polygon, face_triangles;
        arrangementface_to_polygon(face, polygon);
        vec3f polygon3d;
        for (auto& p : polygon) {
          polygon3d.push_back({p[0],p[1],0});
        }
        decomposed_footprint.geom.push_back(polygon3d);
        decomposed_footprint.attr["height"].push_back(face->data().elevation_avg);
      }
    }
    set_value("decomposed_footprint", decomposed_footprint);
  }
};

class ExtruderNode:public Node {
  bool do_walls=true, do_roofs=true;
  public:
  ExtruderNode(NodeManager& manager):Node(manager) {
    add_input("arrangement", TT_any);
    add_output("cell_id_vec1i", TT_vec1i);
    add_output("triangles_vec3f", TT_vec3f);
    add_output("normals_vec3f", TT_vec3f);
    add_output("labels_vec1i", TT_vec1i); // 0==ground, 1==roof, 2==outerwall, 3==innerwall
  }

  void gui() {
    ImGui::Checkbox("Do walls", &do_walls);
    ImGui::Checkbox("Do roofs", &do_roofs);
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

    if (do_roofs) {
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
    }
    if (do_walls) {
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
    }

    set_value("normals_vec3f", normals);
    set_value("cell_id_vec1i", cell_id_vec1i);
    set_value("triangles_vec3f", triangles);
    set_value("labels_vec1i", labels);
  }
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

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto points = std::any_cast<PNL_vector>(get_value("points"));
    auto arr = std::any_cast<Arrangement_2>(get_value("arrangement"));

    process_arrangement(points, arr, c);
    
    set_value("arrangement", arr);
  }
};

class BuildArrangementNode:public Node {
  // float footprint_simp_thres=0;

  public:
  BuildArrangementNode(NodeManager& manager):Node(manager) {
    add_input("edge_segments", TT_any);
    add_input("footprint_vec3f", TT_vec3f);
    add_output("arrangement", TT_any);
    add_output("features", TT_any);
    add_output("arr_segments_vec3f", TT_vec3f);
  }

  void gui(){
    // ImGui::InputFloat("Footprint simp", &footprint_simp_thres, 0.01, 1);
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto edge_segments = std::any_cast<std::vector<std::pair<Point,Point>>>(get_value("edge_segments"));
    auto fp_vec3f = std::any_cast<vec3f>(get_value("footprint_vec3f"));

    bg::model::polygon<point_type> fp;
    for(auto& p : fp_vec3f) {
       bg::append(fp.outer(), point_type(p[0], p[1]));
     }

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
  DetectLinesNode(NodeManager& manager):Node(manager) {
    add_input("edge_points", TT_any);
    add_output("edge_segments_c", TT_line_string_collection);
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
    LineStringCollection edge_segments_c;
    for (auto s : edge_segments){
      edge_segments_c.push_back({{
        float(s.first.x()),
        float(s.first.y()),
        float(s.first.z())
      },{
        float(s.second.x()),
        float(s.second.y()),
        float(s.second.z())
      }});
    }
    set_value("edge_segments_c", edge_segments_c);
  }
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
  ComputeMetricsNode(NodeManager& manager):Node(manager) {
    add_output("points", TT_any);
    add_input("points_vec3f", TT_vec3f); // change to Feature
    add_output("plane_id", TT_vec1i);
    add_output("is_wall", TT_vec1i);
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

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto points = std::any_cast<vec3f>(get_value("points_vec3f"));
    PNL_vector pnl_points;
    for (auto& p : points) {
      PNL pv;
      CGAL::cpp11::get<0>(pv) = Point(p[0], p[1], p[2]);
      pnl_points.push_back(pv);
    }
    compute_metrics(pnl_points, c);
    vec1f line_dist, jump_count, jump_ele;
    vec1i plane_id, is_wall;
    PointCollection points_c;
    for(auto& p : pnl_points){
      plane_id.push_back(boost::get<2>(p));
      is_wall.push_back(boost::get<3>(p));
      line_dist.push_back(boost::get<4>(p));
      jump_count.push_back(boost::get<5>(p));
      jump_ele.push_back(boost::get<7>(p));
      std::array<float,3> a = {
              float(boost::get<0>(p).x()),
              float(boost::get<0>(p).y()),
              float(boost::get<0>(p).z())
            };
      points_c.push_back(a);
    }
    set_value("points", pnl_points);
    set_value("points_c", points_c);
    set_value("plane_id", plane_id);
    set_value("is_wall", is_wall);
    set_value("line_dist", line_dist);
    set_value("jump_count", jump_count);
    set_value("jump_ele", jump_ele);

  }
};

pGridSet build_grid(vec3f& ring) {
  int Grid_Resolution = 20;

  int size = ring.size();
  std::vector<pPipoint> pgon;
  for (auto& p : ring) {
    pgon.push_back(new Pipoint{ p[0],p[1] });
  }
  pGridSet grid_set = new GridSet();
  // skip last point in the ring, ie the repetition of the first vertex
  GridSetup(&pgon[0], pgon.size()-1, Grid_Resolution, grid_set);
  for (int i = 0; i < size; i++) {
    delete pgon[i];
  }
  return grid_set;
}

class LASInPolygonsNode:public Node {
  Feature point_clouds;
  LinearRingCollection polygons;

  public:
  int footprint_id=0;
  // char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
  char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
  LASInPolygonsNode(NodeManager& manager):Node(manager) {
    add_input("polygons", TT_linear_ring_collection);
    add_output("point_clouds", TT_any);
    add_output("points_vec3f", TT_vec3f);
    add_output("footprint_vec3f", TT_vec3f);
  }

  void gui() {
    ImGui::InputText("LAS file path", las_filepath, IM_ARRAYSIZE(las_filepath));
    if (ImGui::SliderInt("#", &footprint_id, 0, polygons.size()-1)) {
      // if(run_on_change) {
      //   manager.run(*this);
      // } else {
        notify_children();
        set_value("points_vec3f", point_clouds.geom[footprint_id]);
        set_value("footprint_vec3f", polygons[footprint_id]);
        propagate_outputs();
      // }
    }
  }

  void process() {
    polygons = std::any_cast<LinearRingCollection>(get_value("polygons"));

    // std::vector<bg::model::polygon<point_type>> boost_polygons;
    std::vector<pGridSet> poly_grids;
            
    for (auto& ring : polygons) {
      poly_grids.push_back(build_grid(ring));
      // bg::model::polygon<point_type> boost_poly;
      // for (auto& p : ring) {
      //   bg::append(boost_poly.outer(), point_type(p[0], p[1]));
      // }
      // bg::unique(boost_poly);
      // boost_polygons.push_back(boost_poly);
    }

    //  for(auto& p : ring) {
    //    bg::append(new_fp.outer(), point_type(p[0], p[1]));
    //  }
    //  footprints.push_back(new_fp);

    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(las_filepath);
    LASreader* lasreader = lasreadopener.open();

    point_clouds = Feature();
    point_clouds.type = geoflow::points;
    point_clouds.geom.resize(polygons.size());

    while (lasreader->read_point()) {
      if (lasreader->point.get_classification() == 6) {
        int i=0;
        pPipoint point = new Pipoint{ lasreader->point.get_x()-(*manager.data_offset)[0], lasreader->point.get_y()-(*manager.data_offset)[1] };
        
        for (auto& poly_grid:poly_grids) {
          if (GridTest(poly_grid, point)) {
            point_clouds.geom[i].push_back({
              float(lasreader->point.get_x()-(*manager.data_offset)[0]), 
              float(lasreader->point.get_y()-(*manager.data_offset)[1]),
              float(lasreader->point.get_z()-(*manager.data_offset)[2])
            });
            break;
            // laswriter->write_point(&lasreader->point);
          }
          i++;
        }
        delete point;
      }
    }
    for (int i=0; i<poly_grids.size(); i++) {
      delete poly_grids[i];
    }
    lasreader->close();
    delete lasreader;

    set_value("point_clouds", point_clouds);
    set_value("points_vec3f", point_clouds.geom[footprint_id]);
    set_value("footprint_vec3f", polygons[footprint_id]);
  }
};

class PointsInFootprintNode:public Node {
  private:
  std::vector<bg::model::polygon<point_type>> footprints;
  std::vector<PNL_vector> points_vec;
  std::vector<vec3f> points_vec3f;
  std::vector<vec3f> footprints_vec3f;

  public:
  config c;
  int footprint_id=0;
  bool run_on_change=false;
  bool isInitialised = false;

  // char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
  char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
  // char csv_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/rdam_sample_0.csv";
  char csv_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/bag_amersfoort_0.csv";
  // char las_filepath[256] = "/Users/ravi/data/VOLTA-ICGC-BCN/VOLTA_LAS/LAS_ETOH/116102.LAS";
  // char csv_filepath[256] = "/Users/ravi/data/VOLTA-ICGC-BCN/DGN-SHP/Footprints_MUC/tile_117102/footprints2d.csv";
//   /Users/ravi/data/VOLTA-ICGC-BCN/VOLTA_LAS/LAS_ETOH/117102.LAS
// /Users/ravi/data/VOLTA-ICGC-BCN/DGN-SHP/Footprints_MUC/tile_117102/footprints2d.csv
  // char pc_cache_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/bcn116102_cache.cbor";
  char pc_cache_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/cache.cbor";

  PointsInFootprintNode(NodeManager& manager):Node(manager) {
    add_output("points", TT_any);
    add_output("points_vec3f", TT_vec3f);
    add_output("footprint", TT_any);
    add_output("footprint_vec3f", TT_vec3f);
  }

  void gui(){
    ImGui::InputText("LAS file path", las_filepath, IM_ARRAYSIZE(las_filepath));
    ImGui::InputText("CSV file path", csv_filepath, IM_ARRAYSIZE(csv_filepath));
    ImGui::InputText("cache path", pc_cache_filepath, IM_ARRAYSIZE(pc_cache_filepath));
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
    if(ImGui::Button("deinit")) {
      isInitialised=false;
    }
    if(ImGui::Button("save to cache")) {
      save();
    }
  }

  void save() {
    //json j;
    //j["footprints_vec3f"] = footprints_vec3f;
    //j["points_vec3f"] = points_vec3f;
    //std::ofstream os(pc_cache_filepath);
    //json::to_cbor(j, os);
    //os.close();
  }
  void load() {
    //std::ifstream is(pc_cache_filepath);
    //json j = json::from_cbor(is);
    //is.close();
    //footprints_vec3f = (const std::vector<vec3f>) j["footprints_vec3f"];
    //footprints.clear();
    //for(auto& ring : footprints_vec3f){
    //  bg::model::polygon<point_type> new_fp;
    //  for(auto& p : ring) {
    //    bg::append(new_fp.outer(), point_type(p[0], p[1]));
    //  }
    //  footprints.push_back(new_fp);
    //}
    //
    //points_vec3f = (const std::vector<vec3f>) j["points_vec3f"];
    //points_vec.clear();
    //for(auto& points : points_vec3f){
    //  PNL_vector new_pc;
    //  for (auto& p : points){
    //    PNL pv;
    //    CGAL::cpp11::get<0>(pv) = Point(p[0], p[1], p[2]);
    //    new_pc.push_back(pv);
    //  }
    //  points_vec.push_back(new_pc);
    //}
  }

  void process(){
    if(!isInitialised) {
      fs::path path(pc_cache_filepath);
      if(fs::exists(path)) {
        load();
        isInitialised = true;
      } else {        
        footprints.clear();
        footprints_vec3f.clear();
        points_vec.clear();
        points_vec3f.clear();
        // Set up vertex data (and buffer(s)) and attribute pointers
        auto csv_footprints = std::ifstream(csv_filepath);
        
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

        pc_in_footprint(std::string(las_filepath), footprints, points_vec);

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
            boost::get<0>(p) = Point(
              boost::get<0>(p).x()-bg::get<0>(centroid),
              boost::get<0>(p).y()-bg::get<1>(centroid),
              boost::get<0>(p).z());
          }
        }

        for(auto& pc : points_vec) {
          vec3f v;
          for(auto& p : pc) {
            std::array<float,3> a = {
              float(boost::get<0>(p).x()),
              float(boost::get<0>(p).y()),
              float(boost::get<0>(p).z())
            };
            v.push_back(a);
          }
          points_vec3f.push_back(v);
        }
        std::cout << footprints.size() << "\n";
        isInitialised = true;
      }
    }
    set_value("points", points_vec[footprint_id]);
    set_value("points_vec3f", points_vec3f[footprint_id]);
    set_value("footprint", footprints[footprint_id]);
    set_value("footprint_vec3f", footprints_vec3f[footprint_id]);
  }
};

class RegulariseLinesNode:public Node {
  static constexpr double pi = 3.14159265358979323846;
  float dist_threshold = 0.5;
  float angle_threshold = 0.1;//5*(pi/180);

  public:
  RegulariseLinesNode(NodeManager& manager):Node(manager) {
    add_input("edge_segments", TT_any);
    add_input("footprint_vec3f", TT_vec3f);
    add_output("edges_out", TT_any);
    add_output("edges_out_vec3f", TT_vec3f);
    add_output("tmp_vec3f", TT_vec3f);
  }

  void gui(){
    ImGui::DragFloat("Distance threshold", &dist_threshold, 0.1, 0);
    ImGui::DragFloat("Angle threshold", &angle_threshold, 0.1, 0.0, pi);
  }

  void process(){
    // Set up vertex data (and buffer(s)) and attribute pointers
    auto edges = std::any_cast<std::vector<std::pair<Point,Point>>>(get_value("edge_segments"));
    auto footprint_vec3f = std::any_cast<vec3f>(get_value("footprint_vec3f"));
    typedef CGAL::Exact_predicates_inexact_constructions_kernel::Point_2 Point_2;

    //compute midpoint and direction for each segment
    typedef std::tuple<double, Point_2, double, double, bool, double, double> linetype; // new angle, midpoint, distance in angle cluster, elevation, is_footprint, initial angle, squared distance from midpoint to an end point
    std::vector<linetype> lines;
    // add non-footprint lines
    for(auto edge : edges) {
      auto v = edge.second-edge.first;
      auto p_ = edge.first + v/2;
      auto p = Point_2(p_.x(),p_.y());
      auto l = v.squared_length();
      auto angle = std::atan2(v.x(),v.y());
      if (angle < 0) angle += pi;
      lines.push_back(std::make_tuple(angle,p,0,p_.z(), false, angle, l));
    }
    // add footprint edges
    // footprint_vec3f.push_back(footprint_vec3f[0]); //repeat first point as last
    vec3f tmp_vec3f;
    for(size_t i=0; i<footprint_vec3f.size()-1; i++) {
      auto p_first = Point_2(footprint_vec3f[i+0][0], footprint_vec3f[i+0][1]);
      auto p_second = Point_2(footprint_vec3f[i+1][0], footprint_vec3f[i+1][1]);
      auto v = p_second - p_first;

      tmp_vec3f.push_back({float(p_first.x()), float(p_first.y()), 0});
      tmp_vec3f.push_back({float(p_second.x()), float(p_second.y()), 0});

      auto p_ = p_first + v/2;
      auto p = Point_2(p_.x(),p_.y());
      auto l = v.squared_length();
      auto angle = std::atan2(v.x(),v.y());
      if (angle < 0) angle += pi;
      lines.push_back(std::make_tuple(angle,p,0,0, true, angle, l));
    }
    set_value("tmp_vec3f", tmp_vec3f);

    // for (auto line: lines) {
    //   std::cout << std::get<0>(line) << " " << std::get<4>(line) << "\n";
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

    // for (auto cluster: angle_clusters) {
    //   std::cout << "cluster ..\n";
    //   for (auto line: cluster) {
    //     std::cout << std::get<0>(line) << " " << std::get<4>(line) << "\n";
    //   }
    // }

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

    // std::cout << "\nafter angle snapping...:\n";
    // for (auto cluster: angle_clusters) {
    //   std::cout << "cluster ..\n";
    //   for (auto line: cluster) {
    //     std::cout << std::get<0>(line) << " " << std::get<4>(line) << "\n";
    //   }
    // }

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

    // std::cout << "\nafter distance snapping...:\n";
    // for (auto cluster: dist_clusters) {
    //   std::cout << "cluster ..\n";
    //   for (auto line: cluster) {
    //     std::cout << std::get<2>(line) << " " << std::get<4>(line) << "\n";
    //   }
    // }

    // compute one line per dist cluster => the one with the highest elevation
    vec3f edges_out_vec3f;
    std::vector<std::pair<Point,Point>> edges_out;
    for(auto& cluster : dist_clusters) {
      //try to find a footprint line
      linetype best_line;
      double best_angle;
      bool found_fp=false, found_non_fp=false;
      for(auto& line : cluster) {
        if(std::get<4>(line)){
          found_fp=true;
          best_line = line;
          best_angle = std::get<5>(line); // initial angle of this fp line
        } else {
          found_non_fp = true;
        }
      }
      // if there are no non-footprint lines, skip this cluster
      if(!found_non_fp) continue;
      // if we didn't find any footprint lines, pick the line with the highest elevation
      if(!found_fp){
        double max_z=0;
        linetype high_line;
        for(auto& line : cluster) {
          auto z = std::get<3>(line);
          if(z > max_z) {
            max_z = z;
            best_line = line;
            best_angle = std::get<0>(best_line);
          }
        }
      }
      // compute vec orthogonal to lines in this cluster
      auto p0 = std::get<1>(best_line);
      auto halfdist = std::sqrt(std::get<6>(best_line));
      // Vector_2 n(-1.0, std::tan(angle));
      // n = n/std::sqrt(n.squared_length();()); // normalize
      Vector_2 l(std::tan(best_angle),1.0);
      l = l/std::sqrt(l.squared_length()); // normalize
      auto p_center = p0;// + average_dist*n;
      auto p_begin = p_center + halfdist*l;
      auto p_end = p_center - halfdist*l;
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


class LOD13GeneratorNode:public Node {
  public:
  float step_threshold = 1.0;
  LOD13GeneratorNode(NodeManager& manager):Node(manager) {
    add_input("point_clouds", TT_any);
    add_input("polygons", TT_any);
    add_output("decomposed_polygons", TT_any);
  }

  void gui(){
    ImGui::InputFloat("Step height", &step_threshold, 0.1, 1);
  }

  void process(){
    auto point_clouds = std::any_cast<Feature>(get_value("point_clouds"));
    auto polygons = std::any_cast<Feature>(get_value("polygons"));
    
    // for each pair of polygon and point_cloud
      //create nodes and connections
      //run the thing
    if (point_clouds.geom.size()!=polygons.geom.size()) return;

    Feature decomposed_polygons;
    decomposed_polygons.type=geoflow::line_loop;
    
    for(int i=0; i<point_clouds.geom.size(); i++) {
      auto& points_vec3f = point_clouds.geom[i];
      auto& polygon_vec3f = polygons.geom[i];

      NodeManager N = NodeManager();

      auto ComputeMetrics_node = std::make_shared<ComputeMetricsNode>(N);
      auto AlphaShape_node = std::make_shared<AlphaShapeNode>(N);
      auto DetectLines_node = std::make_shared<DetectLinesNode>(N);
      auto RegulariseLines_node = std::make_shared<RegulariseLinesNode>(N);
      auto BuildArrangement_node = std::make_shared<BuildArrangementNode>(N);
      auto ProcessArrangement_node = std::make_shared<ProcessArrangementNode>(N);
      auto Arr2Feature_node = std::make_shared<Arr2FeatureNode>(N);

      ComputeMetrics_node->inputTerminals["points_vec3f"]->push(points_vec3f);
      BuildArrangement_node->inputTerminals["footprint_vec3f"]->push(polygon_vec3f);
      RegulariseLines_node->inputTerminals["footprint_vec3f"]->push(polygon_vec3f);

      connect(*ComputeMetrics_node, *AlphaShape_node, "points", "points");
      connect(*ComputeMetrics_node, *ProcessArrangement_node, "points", "points");
      connect(*AlphaShape_node, *DetectLines_node, "edge_points", "edge_points");
      connect(*DetectLines_node, *RegulariseLines_node, "edge_segments", "edge_segments");
      connect(*RegulariseLines_node, *BuildArrangement_node, "edges_out", "edge_segments");
      connect(*BuildArrangement_node, *ProcessArrangement_node, "arrangement", "arrangement");
      connect(*ProcessArrangement_node, *Arr2Feature_node, "arrangement", "arrangement");

      // config and run
      ProcessArrangement_node->c.step_height_threshold = step_threshold;
      N.run(*ComputeMetrics_node);

      auto oTerm = Arr2Feature_node->outputTerminals["decomposed_footprint"];
      auto polygons_feature = std::any_cast<Feature>(oTerm->cdata);

      for (int i=0; i<polygons_feature.geom.size(); i++) {
        // if(polygons_feature.attr["height"][i]!=0) { //FIXME this is a hack!!
          decomposed_polygons.geom.push_back(polygons_feature.geom[i]);
          decomposed_polygons.attr["height"].push_back(polygons_feature.attr["height"][i]);
        // }
      }
    }
    set_value("decomposed_polygons", decomposed_polygons);
  }
};