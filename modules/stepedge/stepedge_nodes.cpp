#include "stepedge_nodes.hpp"
// #include "gloo.h"
#include "ptinpoly.h"
#include "earcut.hpp"

// #include "nlohmann/json.hpp"
// using json = nlohmann::json;

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include <boost/tuple/tuple.hpp>

// #include <filesystem>
#include <numeric>

// namespace fs=std::filesystem;

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

void AlphaShapeNode::process(){
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
  auto points = inputs("points").get<PNL_vector>();
  
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
  
  outputs("edge_points").set(edge_points);
  outputs("edge_points_vec3f").set(edge_points_vec3f);
}

void PolygonExtruderNode::process(){
  auto polygons = inputs("polygons").get<Feature>();
  auto point_clouds = inputs("point_clouds").get<Feature>();

  if(polygons.geom.size()!=point_clouds.geom.size()) return;

  polygons.attr["height"] = vec1f();
  for (auto& point_cloud : point_clouds.geom) {
    double sum_elevation = 0;
    for (auto& p : point_cloud) {
      sum_elevation += p[2];
    }
    polygons.attr["height"].push_back(sum_elevation/point_cloud.size());
  }
  outputs("polygons_extruded").set(polygons);
}

void Arr2LinearRingsNode::process(){
  auto arr = inputs("arrangement").get<Arrangement_2>();

  LinearRingCollection linear_rings;
  AttributeMap attributes;
  for (auto face: arr.face_handles()){
    if(face->data().is_finite) {
      vec2f polygon, face_triangles;
      arrangementface_to_polygon(face, polygon);
      vec3f polygon3d;
      for (auto& p : polygon) {
        polygon3d.push_back({p[0],p[1],0});
      }
      linear_rings.push_back(polygon3d);
      attributes["height"].push_back(face->data().elevation_avg);
    }
  }
  outputs("linear_rings").set(linear_rings);
  outputs("attributes").set(attributes);
}

void ExtruderNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  // auto polygons = std::any_cast<inputs("polygons").get<vec2f>>();
  // auto elevations = std::any_cast<inputs("elevations").get<float>>();
  auto arr = inputs("arrangement").get<Arrangement_2>();

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

  outputs("normals_vec3f").set(normals);
  outputs("cell_id_vec1i").set(cell_id_vec1i);
  outputs("triangles_vec3f").set(triangles);
  outputs("labels_vec1i").set(labels);
}

void ProcessArrangementNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto points = inputs("points").get<PNL_vector>();
  auto arr = inputs("arrangement").get<Arrangement_2>();

  process_arrangement(points, arr, c);
  
  outputs("arrangement").set(arr);
}

void BuildArrangementNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto edge_segments = inputs("edge_segments").get<std::vector<std::pair<Point,Point>>>();
  auto fp_vec3f = inputs("footprint_vec3f").get<vec3f>();

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
  outputs("arr_segments_vec3f").set(polygons);
  outputs("arrangement").set(arr);
}

void DetectLinesNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto edge_points = inputs("edge_points").get<std::vector<linedect::Point>>();
  std::vector<std::pair<Point,Point>> edge_segments;
  detect_lines(edge_segments, edge_points, c);
  outputs("edge_segments").set(edge_segments);
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
  outputs("edge_segments_c").set(edge_segments_c);
}

void ClassifyEdgePointsNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto points = inputs("points").get<PNL_vector>();
  std::vector<linedect::Point> edge_points;
  classify_edgepoints(edge_points, points, c);
  outputs("edge_points").set(edge_points);

  vec3f edge_points_vec3f;
  for(auto& p : edge_points) {
      std::array<float,3> a = {{
        float(p.x()), 
        float(p.y()), 
        float(p.z())
      }};
      edge_points_vec3f.push_back(a);
    }
  outputs("edge_points_vec3f").set(edge_points_vec3f);
}

void ComputeMetricsNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto points = inputs("points_vec3f").get<vec3f>();
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
  outputs("points").set(pnl_points);
  outputs("points_c").set(points_c);
  outputs("plane_id").set(plane_id);
  outputs("is_wall").set(is_wall);
  outputs("line_dist").set(line_dist);
  outputs("jump_count").set(jump_count);
  outputs("jump_ele").set(jump_ele);
}

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

void LASInPolygonsNode::process() {
  polygons = inputs("polygons").get<LinearRingCollection>();

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

  outputs("point_clouds").set(point_clouds);
  outputs("points_vec3f").set(point_clouds.geom[footprint_id]);
  outputs("footprint_vec3f").set(polygons[footprint_id]);
}

// class PointsInFootprintNode:public Node {
//   private:
//   std::vector<bg::model::polygon<point_type>> footprints;
//   std::vector<PNL_vector> points_vec;
//   std::vector<vec3f> points_vec3f;
//   std::vector<vec3f> footprints_vec3f;

//   public:
//   config c;
//   int footprint_id=0;
//   bool run_on_change=false;
//   bool isInitialised = false;

//   // char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
//   char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
//   // char csv_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/rdam_sample_0.csv";
//   char csv_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/bag_amersfoort_0.csv";
//   // char las_filepath[256] = "/Users/ravi/data/VOLTA-ICGC-BCN/VOLTA_LAS/LAS_ETOH/116102.LAS";
//   // char csv_filepath[256] = "/Users/ravi/data/VOLTA-ICGC-BCN/DGN-SHP/Footprints_MUC/tile_117102/footprints2d.csv";
// //   /Users/ravi/data/VOLTA-ICGC-BCN/VOLTA_LAS/LAS_ETOH/117102.LAS
// // /Users/ravi/data/VOLTA-ICGC-BCN/DGN-SHP/Footprints_MUC/tile_117102/footprints2d.csv
//   // char pc_cache_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/bcn116102_cache.cbor";
//   char pc_cache_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/cache.cbor";

//   PointsInFootprintNode(NodeManager& manager):Node(manager) {
//     add_output("points", TT_any);
//     add_output("points_vec3f", TT_vec3f);
//     add_output("footprint", TT_any);
//     add_output("footprint_vec3f", TT_vec3f);
//   }

//   void gui(){
//     ImGui::InputText("LAS file path", las_filepath, IM_ARRAYSIZE(las_filepath));
//     ImGui::InputText("CSV file path", csv_filepath, IM_ARRAYSIZE(csv_filepath));
//     ImGui::InputText("cache path", pc_cache_filepath, IM_ARRAYSIZE(pc_cache_filepath));
//     ImGui::Checkbox("Run on change", &run_on_change);
//     if (ImGui::SliderInt("#", &footprint_id, 0, footprints.size()-1)) {
//       if(run_on_change) {
//         manager.run(*this);
//       } else {
//         notify_children();
//         outputs("points").set(points_vec[footprint_id]);
//         outputs("points_vec3f").set(points_vec3f[footprint_id]);
//         outputs("footprint").set(footprints[footprint_id]);
//         outputs("footprint_vec3f").set(footprints_vec3f[footprint_id]);
//         propagate_outputs();
//       }
//     }
//     if(ImGui::Button("deinit")) {
//       isInitialised=false;
//     }
//     if(ImGui::Button("save to cache")) {
//       save();
//     }
//   }

//   void save() {
//     //json j;
//     //j["footprints_vec3f"] = footprints_vec3f;
//     //j["points_vec3f"] = points_vec3f;
//     //std::ofstream os(pc_cache_filepath);
//     //json::to_cbor(j, os);
//     //os.close();
//   }
//   void load() {
//     //std::ifstream is(pc_cache_filepath);
//     //json j = json::from_cbor(is);
//     //is.close();
//     //footprints_vec3f = (const std::vector<vec3f>) j["footprints_vec3f"];
//     //footprints.clear();
//     //for(auto& ring : footprints_vec3f){
//     //  bg::model::polygon<point_type> new_fp;
//     //  for(auto& p : ring) {
//     //    bg::append(new_fp.outer(), point_type(p[0], p[1]));
//     //  }
//     //  footprints.push_back(new_fp);
//     //}
//     //
//     //points_vec3f = (const std::vector<vec3f>) j["points_vec3f"];
//     //points_vec.clear();
//     //for(auto& points : points_vec3f){
//     //  PNL_vector new_pc;
//     //  for (auto& p : points){
//     //    PNL pv;
//     //    CGAL::cpp11::get<0>(pv) = Point(p[0], p[1], p[2]);
//     //    new_pc.push_back(pv);
//     //  }
//     //  points_vec.push_back(new_pc);
//     //}
//   }

//   void process(){
//     if(!isInitialised) {
//       fs::path path(pc_cache_filepath);
//       if(fs::exists(path)) {
//         load();
//         isInitialised = true;
//       } else {        
//         footprints.clear();
//         footprints_vec3f.clear();
//         points_vec.clear();
//         points_vec3f.clear();
//         // Set up vertex data (and buffer(s)) and attribute pointers
//         auto csv_footprints = std::ifstream(csv_filepath);
        
//         std::string column_names, row;
//         std::getline(csv_footprints, column_names);
//         point_type centroid; // we'll set the origin to the centroid of the first footprint
//         bool read_first=false;
//         while (std::getline(csv_footprints, row)) {
//             bg::model::polygon<point_type> bag_polygon;
//             bg::read_wkt(row, bag_polygon);
//             bg::unique(bag_polygon);

//             if(!read_first) {
//               bg::centroid(bag_polygon, centroid);
//               read_first=true;
//             }
//             footprints.push_back(bag_polygon);
//         } csv_footprints.close();

//         pc_in_footprint(std::string(las_filepath), footprints, points_vec);

//         for (auto& fp : footprints) {
//           vec3f fp_vec3f;
//           for (auto& p : fp.outer()){
//               bg::subtract_point(p, centroid);
//               fp_vec3f.push_back({float(bg::get<0>(p)), float(bg::get<1>(p)), 0});
//           }
//           footprints_vec3f.push_back(fp_vec3f);
//         }

//         for(auto& pc : points_vec) {
//           for(auto& p : pc) {
//             boost::get<0>(p) = Point(
//               boost::get<0>(p).x()-bg::get<0>(centroid),
//               boost::get<0>(p).y()-bg::get<1>(centroid),
//               boost::get<0>(p).z());
//           }
//         }

//         for(auto& pc : points_vec) {
//           vec3f v;
//           for(auto& p : pc) {
//             std::array<float,3> a = {
//               float(boost::get<0>(p).x()),
//               float(boost::get<0>(p).y()),
//               float(boost::get<0>(p).z())
//             };
//             v.push_back(a);
//           }
//           points_vec3f.push_back(v);
//         }
//         std::cout << footprints.size() << "\n";
//         isInitialised = true;
//       }
//     }
//     outputs("points").set(points_vec[footprint_id]);
//     outputs("points_vec3f").set(points_vec3f[footprint_id]);
//     outputs("footprint").set(footprints[footprint_id]);
//     outputs("footprint_vec3f").set(footprints_vec3f[footprint_id]);
//   }
// };

void RegulariseLinesNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto edges = inputs("edge_segments").get<std::vector<std::pair<Point,Point>>>();
  auto footprint_vec3f = inputs("footprint_vec3f").get<vec3f>();
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
  outputs("tmp_vec3f").set(tmp_vec3f);

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

  outputs("edges_out").set(edges_out);
  outputs("edges_out_vec3f").set(edges_out_vec3f);
}

void LOD13GeneratorNode::process(){
  auto point_clouds = inputs("point_clouds").get<Feature>();
  auto polygons = inputs("polygons").get<Feature>();
  
  // for each pair of polygon and point_cloud
    //create nodes and connections
    //run the thing
  if (point_clouds.geom.size()!=polygons.geom.size()) return;

  LinearRingCollection all_cells;
  AttributeMap all_attributes;
  
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
    auto Arr2LinearRings_node = std::make_shared<Arr2LinearRingsNode>(N);

    ComputeMetrics_node->inputs("points_vec3f").set(points_vec3f);
    BuildArrangement_node->inputs("footprint_vec3f").set(polygon_vec3f);
    RegulariseLines_node->inputs("footprint_vec3f").set(polygon_vec3f);

    connect(*ComputeMetrics_node, *AlphaShape_node, "points", "points");
    connect(*ComputeMetrics_node, *ProcessArrangement_node, "points", "points");
    connect(*AlphaShape_node, *DetectLines_node, "edge_points", "edge_points");
    connect(*DetectLines_node, *RegulariseLines_node, "edge_segments", "edge_segments");
    connect(*RegulariseLines_node, *BuildArrangement_node, "edges_out", "edge_segments");
    connect(*BuildArrangement_node, *ProcessArrangement_node, "arrangement", "arrangement");
    connect(*ProcessArrangement_node, *Arr2LinearRings_node, "arrangement", "arrangement");

    // config and run
    ProcessArrangement_node->c.step_height_threshold = step_threshold;
    N.run(*ComputeMetrics_node);

    auto cells = Arr2LinearRings_node->outputs("linear_rings").get<LinearRingCollection>();
    auto attributes = Arr2LinearRings_node->outputs("attributes").get<AttributeMap>();

    for (int i=0; i<cells.size(); i++) {
      // if(polygons_feature.attr["height"][i]!=0) { //FIXME this is a hack!!
        all_cells.push_back(cells[i]);
        all_attributes["height"].push_back(attributes["height"][i]);
      // }
    }
  }
  outputs("decomposed_footprints").set(all_cells);
  outputs("attributes").set(all_attributes);
}