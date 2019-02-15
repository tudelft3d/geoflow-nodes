#include "cgal_nodes.hpp"

#include "tinsimp.hpp"
#include "linesimp.hpp"
#include "isolines.hpp"
#include "visvalingam_cost.hpp"

#include <lasreader.hpp>
#include <fstream>
#include <algorithm>

// CDT
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Triangulation_vertex_base_with_id_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

// DT
#include <CGAL/Delaunay_triangulation_3.h>

// AABB tree
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

// line simplification
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

// PLY writing
#include <CGAL/property_map.h>
#include <CGAL/IO/write_ply_points.h>

// iso lines
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/IO/Complex_2_in_triangulation_3_file_writer.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

// line height calculator
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>

namespace geoflow::nodes::cgal {

template<typename T> inline std::array<float,3> to_arr3f(T& p) {
  return {float(p.x()), float(p.y()), float(p.z())};
}

void CDTNode::process(){
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Projection_traits_xy_3<K>								Gt;
  typedef CGAL::Exact_predicates_tag									  Itag;
  typedef CGAL::Constrained_Delaunay_triangulation_2<Gt, CGAL::Default, Itag>	CDT;
  typedef CDT::Point													          Point;

  auto geom_term = input("geometries");

  auto create_triangles = param<bool>("create_triangles");

  CDT cdt;

  if (geom_term.connected_type == TT_point_collection) {
    auto points = geom_term.get<geoflow::PointCollection>();
    
    std::cout << "Adding points to CDT\n";
    for (auto& p : points) {
      cdt.insert(Point(p[0], p[1], p[2]));
    }
  }
  else if (geom_term.connected_type == TT_line_string_collection) {
    auto lines = geom_term.get<geoflow::LineStringCollection>();

    std::cout << "Adding lines to CDT\n";
    for (auto& line : lines) {
      std::vector<Point> cgal_points;
      cgal_points.reserve(line.size());
      for (auto& p : line) {
        cgal_points.push_back(Point(p[0], p[1], p[2]));
      }
      cdt.insert_constraint(cgal_points.begin(), cgal_points.end());
    }
  }

  std::cout << "Completed CDT with " << cdt.number_of_faces() << " triangles...\n";
  // assert(cdt.is_valid());
  TriangleCollection triangles;
  if (create_triangles) {
    for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
      fit != cdt.finite_faces_end();
      ++fit) {
      auto& p0 = fit->vertex(0)->point();
      auto& p1 = fit->vertex(1)->point();
      auto& p2 = fit->vertex(2)->point();
      triangles.push_back({
        to_arr3f<Point>(p0),
        to_arr3f<Point>(p1),
        to_arr3f<Point>(p2)
        });
    }
    cdt.clear();
  }

  output("cgal_cdt").set(cdt);
  output("triangles").set(triangles);
}

void DTNode::process() {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Delaunay_triangulation_3<K>	            DT;
  typedef K::Point_3													          Point;

  // Set up vertex data (and buffer(s)) and attribute pointers
  auto points = input("points").get<PointCollection>();


  std::cout << "Adding points to DT\n";
  DT dt;
  for (auto& point : points) {
    dt.insert(Point(point[0], point[1], point[2]));
  }

  std::cout << "Completed DT with " << dt.number_of_finite_facets() << " triangles...\n";
  output("cgal_dt").set(dt);
}

void ComparePointDistanceNode::process(){
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::FT FT;
  typedef K::Ray_3 Ray;
  typedef K::Line_3 Line;
  typedef K::Point_3 Point;
  typedef K::Triangle_3 Triangle;
  typedef std::list<Triangle>::iterator Iterator;
  typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
  typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

  // params
  auto las_filepath = param<std::string>("las_filepath").c_str();
  auto log_filepath = param<std::string>("log_filepath").c_str();
  auto thin_nth = param<int>("thin_nth");

  // Triangles 1
  auto trin1 = input("triangles1_vec3f").get<vec3f>();
  std::list<Triangle> triangles1;
  for(size_t i=0; i< trin1.size()/3; i++){
    auto a = Point(trin1[i*3+0][0], trin1[i*3+0][1], trin1[i*3+0][2]);
    auto b = Point(trin1[i*3+1][0], trin1[i*3+1][1], trin1[i*3+1][2]);
    auto c = Point(trin1[i*3+2][0], trin1[i*3+2][1], trin1[i*3+2][2]);
    triangles1.push_back(Triangle(a,b,c));
  }
  Tree tree1(triangles1.begin(),triangles1.end());
  tree1.accelerate_distance_queries();

  // Triangles 2
  auto trin2 = input("triangles2_vec3f").get<vec3f>();
  std::list<Triangle> triangles2;
  for(size_t i=0; i< trin2.size()/3; i++){
    auto a = Point(trin2[i*3+0][0], trin2[i*3+0][1], trin2[i*3+0][2]);
    auto b = Point(trin2[i*3+1][0], trin2[i*3+1][1], trin2[i*3+1][2]);
    auto c = Point(trin2[i*3+2][0], trin2[i*3+2][1], trin2[i*3+2][2]);
    triangles2.push_back(Triangle(a,b,c));
  }
  Tree tree2(triangles2.begin(),triangles2.end());
  tree2.accelerate_distance_queries();

  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(las_filepath);
  LASreader* lasreader = lasreadopener.open();

  vec1f distances1, distances2, diff;
  vec3f points;
  std::ofstream f_out(log_filepath);
  f_out << std::fixed << std::setprecision(2);
  size_t i=0;
  while (lasreader->read_point()) {
    if (lasreader->point.get_classification() == 2){

      if (i++ % thin_nth == 0){
        auto q = Point(lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z());
        float d1 = std::sqrt(tree1.squared_distance(q));
        distances1.push_back(d1);
        float d2 = std::sqrt(tree2.squared_distance(q));
        distances2.push_back(d2);
        auto difference = d2-d1;
        diff.push_back(difference);
        points.push_back({
          float(lasreader->point.get_x()), 
          float(lasreader->point.get_y()), 
          float(lasreader->point.get_z())}
        );
        f_out << float(lasreader->point.get_x()) << " " << float(lasreader->point.get_y()) << " " << float(lasreader->point.get_z()) << " ";
        f_out << std::sqrt(d1) << " " << std::sqrt(d2) << " " << difference << "\n";
      }
      if(i%100000000==0) std::cout << "Read " << i << " points...\n";
      // laswriter->write_point(&lasreader->point);
    }
  }
  lasreader->close();
  delete lasreader;
  f_out.close();
  
  for(int i=0; i<points.size(); i++) {
    f_out << points[i][0] << " " << points[i][1] << " " << points[i][2] << " ";
    f_out << std::sqrt(distances1[i]) << " " << std::sqrt(distances2[i]) << " " << diff[i] << "\n";
  }
  

  output("points").set(points);
  output("diff").set(diff);
  output("distances1").set(distances1);
  output("distances2").set(distances2);
}

void PointDistanceNode::process(){
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::FT FT;
  typedef K::Ray_3 Ray;
  typedef K::Line_3 Line;
  typedef K::Point_3 Point;
  typedef K::Triangle_3 Triangle;
  typedef std::list<Triangle>::iterator Iterator;
  typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
  typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

  auto filepath = param<std::string>("filepath").c_str();
  auto thin_nth = param<int>("thin_nth");
  auto overwritez = param<bool>("overwritez");

  auto trin = input("triangles").get<TriangleCollection>();
  std::list<Triangle> triangles;
  for(auto& t : trin){
    auto a = Point(t[0][0], t[0][1], t[0][2]);
    auto b = Point(t[1][0], t[1][1], t[1][2]);
    auto c = Point(t[2][0], t[2][1], t[2][2]);
    triangles.push_back(Triangle(a,b,c));
  }
  
  // constructs AABB tree
  Tree tree(triangles.begin(),triangles.end());
  tree.accelerate_distance_queries();

  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(filepath);
  LASreader* lasreader = lasreadopener.open();

  vec1f distances;
  PointCollection points;
  size_t i=0;
  auto offset = manager.data_offset.value_or(std::array<double,3>({0,0,0}));
  while (lasreader->read_point()) {
    if (lasreader->point.get_classification() == 2){

      if (i++ % thin_nth == 0){
        auto q = Point(lasreader->point.get_x() - offset[0], lasreader->point.get_y() - offset[1], lasreader->point.get_z() - offset[2]);
        FT sqd = tree.squared_distance(q);
        distances.push_back(sqd);
        if (overwritez) {
          points.push_back({
            float(lasreader->point.get_x() - offset[0]),
            float(lasreader->point.get_y() - offset[1]),
            float(sqrt(sqd)) }
          );
        }
        else {
          points.push_back({
            float(lasreader->point.get_x() - offset[0]),
            float(lasreader->point.get_y() - offset[1]),
            float(lasreader->point.get_z() - offset[2]) }
          );
        }
      }
      if(i%100000==0) std::cout << "Read " << i << " points...\n";
      // laswriter->write_point(&lasreader->point);
    }
  }
  lasreader->close();
  delete lasreader;

 auto minmax = std::minmax_element(distances.begin(), distances.end());

  output("points").set(points);
  output("distances").set(distances);
  output("distance_min").set(sqrt(*(minmax.first)));
  output("distance_max").set(sqrt(*(minmax.second)));
}

LineStringCollection densify_linestrings(LineStringCollection line_strings, float interval)
{
  LineStringCollection dense_linestrings;
  for (auto& line : line_strings) {
    vec3f dense_linestring;
    dense_linestring.push_back(line[0]);
    for(size_t i=1; i<line.size(); ++i) {
      auto s = glm::make_vec3(line[i-1].data());
      auto t = glm::make_vec3(line[i].data());
      auto d = glm::distance(s,t);
      if (d > interval) {
        auto n = glm::normalize(t-s);
        size_t count = glm::floor(d/interval);
        for (size_t j=0; j<count; ++j) {
          auto new_p = s+j*interval*n;
          dense_linestring.push_back({new_p.x, new_p.y, new_p.z});
        }
      }
      dense_linestring.push_back(line[i]);
    }
    dense_linestrings.push_back(dense_linestring);
  }
  return dense_linestrings;
}

void DensifyNode::process(){
  auto geom_term = input("geometries");

  auto interval = param<float>("interval");

  if (geom_term.connected_type == TT_line_string_collection) {
    auto lines = geom_term.get<geoflow::LineStringCollection>();
    output("dense_linestrings").set(densify_linestrings(lines, interval));
  }

}

void build_initial_tin(tinsimp::CDT& cdt, geoflow::Box& bbox){ 
  float min_x = bbox.min()[0]-1;
  float min_y = bbox.min()[1]-1;
  float max_x = bbox.max()[0]+1;
  float max_y = bbox.max()[1]+1;
  float center_z = (bbox.max()[2]-bbox.min()[2])/2;

  std::vector<tinsimp::Point> initial_points = {
    tinsimp::Point(min_x, min_y, center_z),
    tinsimp::Point(max_x, min_y, center_z),
    tinsimp::Point(max_x, max_y, center_z),
    tinsimp::Point(min_x, max_y, center_z)
  };
  cdt.insert(initial_points.begin(), initial_points.end());
}

void TinSimpNode::process(){
  auto geom_term = input("geometries");

  auto thres_error = param<float>("thres_error");
  auto densify_interval = param<float>("densify_interval");

  tinsimp::CDT cdt;

  if (geom_term.connected_type == TT_point_collection) {
    auto points = geom_term.get<geoflow::PointCollection>();
    build_initial_tin(cdt, points.box());
    tinsimp::greedy_insert(cdt, points, double(thres_error));
  } else if (geom_term.connected_type == TT_line_string_collection) {
    auto lines = geom_term.get<geoflow::LineStringCollection>();
    build_initial_tin(cdt, lines.box());
    std::vector<size_t> line_counts, selected_line_counts;
    std::vector<float> line_errors, selected_line_errors;
    std::tie(line_counts, line_errors) = tinsimp::greedy_insert(cdt, densify_linestrings(lines, densify_interval), double(thres_error));
    LineStringCollection selected_lines;
    for (size_t i=0; i<lines.size(); ++i) {
      if (line_counts[i] > 0) {
        selected_lines.push_back(lines[i]);
        // selected_line_counts.push_back(line_counts[i]);
        // selected_line_errors.push_back(line_errors[i]);
      }
    }
    output("selected_lines").set(selected_lines);
    // output("count").set(line_counts);
    // output("error").set(line_errors);
  }

  TriangleCollection triangles;
  for (tinsimp::CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
    fit != cdt.finite_faces_end();
    ++fit)
  {
      auto& p0 = fit->vertex(0)->point();
      auto& p1 = fit->vertex(1)->point();
      auto& p2 = fit->vertex(2)->point();
      triangles.push_back({to_arr3f<tinsimp::Point>(p0), to_arr3f<tinsimp::Point>(p1), to_arr3f<tinsimp::Point>(p2)});
  }
  vec3f normals;
  for(auto& t : triangles){
    auto a = glm::make_vec3(t[0].data());
    auto b = glm::make_vec3(t[1].data());
    auto c = glm::make_vec3(t[2].data());
    auto n = glm::cross(b-a, c-b);

    normals.push_back({n.x,n.y,n.z});
    normals.push_back({n.x,n.y,n.z});
    normals.push_back({n.x,n.y,n.z});
  }

  output("triangles").set(triangles);
  output("normals").set(normals);
}

void SimplifyLine3DNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto lines = input("lines").get<LineStringCollection>();

  auto area_threshold = param<float>("area_threshold");

  LineStringCollection simplified_lines;
  for (auto& line_string : lines) {
    simplified_lines.push_back( linesimp::visvalingam(line_string, area_threshold) );
  }

  output("lines").set(simplified_lines);
}

void SimplifyLineNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto lines = input("lines").get<LineStringCollection>();

  auto threshold_stop_cost = param<float>("threshold_stop_cost");
  
  namespace PS = CGAL::Polyline_simplification_2;
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef PS::Vertex_base_2<K>  Vb;
  typedef CGAL::Constrained_triangulation_face_base_2<K> Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb, Fb> TDS;
  typedef CGAL::Exact_predicates_tag                          Itag;
  typedef CGAL::Constrained_Delaunay_triangulation_2<K,TDS, Itag> CDT;
  typedef CGAL::Constrained_triangulation_plus_2<CDT>     CT;
  typedef PS::Stop_below_count_ratio_threshold Stop_count_ratio;
  typedef PS::Stop_above_cost_threshold        Stop_cost;
  typedef PS::Squared_distance_cost            Cost;

  Cost cost;

  LineStringCollection lines_out;
  vec3f vertices_vec3f;
  for (auto& linestring : lines) {
    std::vector<K::Point_2> line;
    for (auto& p : linestring ) {
      line.push_back(K::Point_2(p[0], p[1], p[2]));
    }
    std::vector <K::Point_2> points_out;
    PS::simplify(line.begin(), line.end(), cost, Stop_cost(threshold_stop_cost), points_out.begin());
    
    auto points_begin = points_out.begin();
    auto points_end = points_out.end();
    size_t psize = points_out.size();
    LineString simpline;
    for(auto pit = points_begin; pit != points_end; ++pit) {
      if(pit!=points_begin && pit!=points_end)
        vertices_vec3f.push_back({float(pit->x()), float(pit->y()), 0});
      vertices_vec3f.push_back({float(pit->x()), float(pit->y()), 0});
      simpline.push_back({float(pit->x()), float(pit->y()), 0});
    }
    lines_out.push_back(simpline);
  }
  output("lines").set(lines_out);
}

void SimplifyLinesNode::process() {
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto lines = input("lines").get<LineStringCollection>();
  auto lines2 = input("lines2").get<LineStringCollection>();

  auto threshold_stop_cost = param<float>("threshold_stop_cost");

  namespace PS = CGAL::Polyline_simplification_2;
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Projection_traits_xy_3<K>								  Gt;
  typedef PS::Vertex_base_2<Gt>                           Vb;
  typedef CGAL::Constrained_triangulation_face_base_2<Gt> Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb, Fb>    TDS;
  typedef CGAL::Exact_predicates_tag                      Itag;
  typedef CGAL::Constrained_Delaunay_triangulation_2<Gt, TDS, Itag> CDT;
  typedef CGAL::Constrained_triangulation_plus_2<CDT>     CT;
  typedef PS::Stop_above_cost_threshold                   Stop_cost;
  typedef PS::Visvalingam_cost                            Cost;

  CT ct;
  Stop_cost stop = Stop_cost(threshold_stop_cost);

  size_t s_index = 0;
  for (auto& linestring : lines) {
    std::vector<CDT::Point> line;
    for (auto& p : linestring) {
      line.push_back(CDT::Point(p[0], p[1], p[2]));
    }
    ct.insert_constraint(line.begin(), line.end());
  }
  for (auto& linestring : lines2) {
    std::vector<CDT::Point> line;
    for (auto& p : linestring) {
      line.push_back(CDT::Point(p[0], p[1], p[2]));
    }
    ct.insert_constraint(line.begin(), line.end());
  }

  PS::simplify(ct, Cost(), stop);
  LineStringCollection lines_out;

  for (auto cit = ct.constraints_begin(); cit != ct.constraints_end(); ++cit) {
    vec3f ls;
    for (auto vit = ct.vertices_in_constraint_begin(*cit); vit != ct.vertices_in_constraint_end(*cit); ++vit) {
      ls.push_back({ float((*vit)->point().x()), float((*vit)->point().y()), float((*vit)->point().z()) });
    }
    lines_out.push_back(ls);
  }

  output("lines").set(lines_out);
}

void SimplifyFootprintsCDTNode::process(){
  auto polygons = input("polygons").get<LinearRingCollection>();

  auto threshold_stop_cost = param<float>("threshold_stop_cost");

  namespace PS = CGAL::Polyline_simplification_2;
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Polygon_2<K>                              Polygon_2;
  typedef PS::Vertex_base_2<K>                            Vb;
  typedef CGAL::Constrained_triangulation_face_base_2<K>  Fb;
  typedef CGAL::Triangulation_data_structure_2<Vb, Fb>    TDS;
  typedef CGAL::Exact_predicates_tag                      Itag;
  typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
  typedef CGAL::Constrained_triangulation_plus_2<CDT>     CT;
  typedef CT::Point                                       Point;
  typedef CT::Constraint_iterator                         Cit;
  typedef PS::Stop_above_cost_threshold                   Stop_cost;
  typedef PS::Squared_distance_cost                       Cost;

  CT ct;
  Stop_cost stop = Stop_cost(threshold_stop_cost * threshold_stop_cost);

  for (auto& linearring : polygons) {
    Polygon_2 polygon;
    for (auto& p : linearring) {
      polygon.push_back(Point(p[0], p[1]));
    }
    // keep constraint id to link results back to input
    ct.insert_constraint(polygon);
  }

  PS::simplify(ct, Cost(), stop);
  LinearRingCollection polygons_out;

  for (Cit cit = ct.constraints_begin(); cit != ct.constraints_end(); ++cit) {
    vec3f ls;
    for (auto vit = ct.vertices_in_constraint_begin(*cit); vit != ct.vertices_in_constraint_end(*cit); ++vit) {
      ls.push_back({ float((*vit)->point().x()), float((*vit)->point().y()) });
    }
    polygons_out.push_back(ls);
  }

  output("polygons_simp").set(polygons_out);
}

void PLYWriterNode::process() {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
  typedef Kernel::Point_3 Point;
  typedef boost::tuple<Point, int> PL;
  typedef CGAL::Nth_of_tuple_property_map<0, PL> Point_map;
  // typedef CGAL::Nth_of_tuple_property_map<1, PL> Normal_map;
  typedef CGAL::Nth_of_tuple_property_map<1, PL> Label_map;
  typedef std::vector<PL>                        PL_vector;

  auto points = input("points").get<PointCollection>();
  auto labels = input("labels").get<vec1i>();

  auto filepath = param<std::string>("filepath");
  auto write_binary = param<bool>("write_binary");
  
  PL_vector pl_points;
  pl_points.resize(points.size());
  for (size_t i=0; i<points.size(); ++i) {
    pl_points[i].get<0>()=Point(points[i][0]+(*manager.data_offset)[0], points[i][1]+(*manager.data_offset)[1], points[i][2]+(*manager.data_offset)[2]);
    pl_points[i].get<1>()=labels[i];
  }

  std::ofstream f(filepath);
  if (write_binary)
    CGAL::set_binary_mode(f); // The PLY file will be written in the binary format
  else
    f << std::fixed << std::setprecision(2);

  CGAL::write_ply_points_with_properties
    (f, pl_points,
     CGAL::make_ply_point_writer (Point_map()),
    //  CGAL::make_ply_normal_writer (Normal_map()),
     std::make_pair (Label_map(), CGAL::PLY_property<int>("segment_id"))
    );
  f.close();
}

void IsoLineNode::process() {
  auto cdt = input("cgal_cdt").get<isolines::CDT>();
  float min = input("min").get<float>();
  float max = input("max").get<float>();

  int start = std::floor(min);
  int end = std::ceil(max);

  vec1f heights;
  for (int i = start; i < end; i++) {
    heights.push_back(i);
  }

  LineStringCollection lines;
  AttributeMap attributes;
  std::map< double, std::vector< CGAL::Segment_3<isolines::K> > > segmentVec;

  for (auto isoDepth : heights) {
    std::cout << "Slicing ISO lines at height " << isoDepth << "\n";
    // faceCache is used to ensure line segments are outputted only once. It will contain faces that have an edge exactly on the contouring depth.
    std::set<isolines::Face_handle> faceCache;

    // iterate over all triangle faces
    for (isolines::Face_iterator ib = cdt.finite_faces_begin();
      ib != cdt.finite_faces_end(); ++ib) {
      // shorthand notations for the 3 triangle vertices and their position w.r.t. the contouring depth
      isolines::Vertex_handle v0 = ib->vertex(0);
      isolines::Vertex_handle v1 = ib->vertex(1);
      isolines::Vertex_handle v2 = ib->vertex(2);
      int v0_ = isolines::cntrEvalVertex(v0, isoDepth);
      int v1_ = isolines::cntrEvalVertex(v1, isoDepth);
      int v2_ = isolines::cntrEvalVertex(v2, isoDepth);

      // following is a big if-else-if statement to identify the basic triangle configuration (wrt the contouring depth)

      //its on a horizontal plane: skip it
      if (v0_ == v1_ && v1_ == v2_)
        continue;

      //one edge is equal to isodepth: extract that edge. Use faceCache to check if this segment hasn't been extracted earlier.
      else if (v0_ == 0 && v1_ == 0) {
        faceCache.insert(ib);
        if (faceCache.find(ib->neighbor(2)) == faceCache.end())
          segmentVec[isoDepth].push_back(CGAL::Segment_3<isolines::K>(v0->point(), v1->point()));
      }
      else if (v1_ == 0 && v2_ == 0) {
        faceCache.insert(ib);
        if (faceCache.find(ib->neighbor(0)) == faceCache.end())
          segmentVec[isoDepth].push_back(CGAL::Segment_3<isolines::K>(v1->point(), v2->point()));
      }
      else if (v2_ == 0 && v0_ == 0) {
        faceCache.insert(ib);
        if (faceCache.find(ib->neighbor(1)) == faceCache.end())
          segmentVec[isoDepth].push_back(CGAL::Segment_3<isolines::K>(v2->point(), v0->point()));

        //there is an intersecting line segment in between the interiors of 2 edges: calculate intersection points and extract that edge
      }
      else if ((v0_ == -1 && v1_ == 1 && v2_ == 1) || (v0_ == 1 && v1_ == -1 && v2_ == -1)) {
        isolines::Point p1 = isolines::cntrIntersectEdge(v0, v1, isoDepth);
        isolines::Point p2 = isolines::cntrIntersectEdge(v0, v2, isoDepth);
        segmentVec[isoDepth].push_back(CGAL::Segment_3<isolines::K>(p1, p2));
      }
      else if ((v0_ == 1 && v1_ == -1 && v2_ == 1) || (v0_ == -1 && v1_ == 1 && v2_ == -1)) {
        isolines::Point p1 = isolines::cntrIntersectEdge(v1, v0, isoDepth);
        isolines::Point p2 = isolines::cntrIntersectEdge(v1, v2, isoDepth);
        segmentVec[isoDepth].push_back(CGAL::Segment_3<isolines::K>(p1, p2));
      }
      else if ((v0_ == 1 && v1_ == 1 && v2_ == -1) || (v0_ == -1 && v1_ == -1 && v2_ == 1)) {
        isolines::Point p1 = isolines::cntrIntersectEdge(v2, v0, isoDepth);
        isolines::Point p2 = isolines::cntrIntersectEdge(v2, v1, isoDepth);
        segmentVec[isoDepth].push_back(CGAL::Segment_3<isolines::K>(p1, p2));

        // one vertex is on the isodepth the others are above and below: return segment, consisting out of the vertex on the isodepth and the intersection on the opposing edge
      }
      else if (v0_ == 0 && v1_ != v2_) {
        isolines::Point p = isolines::cntrIntersectEdge(v1, v2, isoDepth);
        segmentVec[isoDepth].push_back(CGAL::Segment_3<isolines::K>(v0->point(), p));
      }
      else if (v1_ == 0 && v0_ != v2_) {
        isolines::Point p = isolines::cntrIntersectEdge(v0, v2, isoDepth);
        segmentVec[isoDepth].push_back(CGAL::Segment_3<isolines::K>(v1->point(), p));
      }
      else if (v2_ == 0 && v0_ != v1_) {
        isolines::Point p = isolines::cntrIntersectEdge(v0, v1, isoDepth);
        segmentVec[isoDepth].push_back(CGAL::Segment_3<isolines::K>(v2->point(), p));
      }
    }
    for (auto s : segmentVec[isoDepth]) {
      vec3f line_vec3f;
      line_vec3f.push_back({ float(s.start().x()),float(s.start().y()), isoDepth });
      line_vec3f.push_back({ float(s.end().x()),float(s.end().y()), isoDepth });
      lines.push_back(line_vec3f);
      attributes["height"].push_back(isoDepth);
    }
  }

  output("lines").set(lines);
  output("attributes").set(attributes);
}

void IsoLineSlicerNode::process() {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_3 Point;
  typedef K::Plane_3 Plane;
  typedef CGAL::Surface_mesh<K::Point_3> Mesh;

  auto cdt = input("cgal_cdt").get<isolines::CDT>();
  //auto heights = input("heights").get<vec1f>();
  //TODO get iso line seperation distance, get min/max height and create list of heights
  vec1f heights;
  for (int i = 1; i < 20; i++) {
    heights.push_back(i);
  }

  std::cout << "Transforming CDT to Surface Mesh\n";
  LineStringCollection lines;
  AttributeMap attributes;

  Mesh mesh;
  isolines::link_cdt_to_face_graph(cdt, mesh);

  //std::ofstream out("D:\\Projects\\3D Geluid\\Hoogtelijnen\\surface_difference_small_box.off");
  //CGAL::write_off(out, mesh);
  
  std::cout << "Start slicing\n";
  CGAL::Polygon_mesh_slicer<Mesh, K> slicer(mesh);
  for (auto h : heights) {
    std::cout << "Slicing ISO ring at height "<< h << "\n";
    std::vector< std::vector< Point > > polylines;
    slicer(Plane(0, 0, 1, -h), std::back_inserter(polylines));

    std::cout << "At z = " << h << ", the slicer intersects "
      << polylines.size() << " polylines" << std::endl;
    
    // transform polylines to vec3f
    for (auto& p : polylines) {
      if (p.size() > 1) {
        vec3f line_vec3f;
        for (auto v = p.begin(); v != p.end(); v++) {
          line_vec3f.push_back({ float(v->x()),float(v->y()), h });
        }
        lines.push_back(line_vec3f);
        attributes["height"].push_back(h);
      }
    }
  }
  output("lines").set(lines);
  output("attributes").set(attributes);
}

void LineHeightNode::process() {
  auto lines = input("lines").get<LineStringCollection>();

  auto filepath = param<std::string>("filepath").c_str();
  auto thin_nth = param<int>("thin_nth");

  typedef CGAL::Simple_cartesian<float> K;
  typedef CGAL::Search_traits_3<K> TreeTraits;
  typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
  typedef Neighbor_search::Tree Tree;
  typedef Tree::Point_d Point_d;

  Tree tree;

  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(filepath);
  LASreader* lasreader = lasreadopener.open();

  LineStringCollection lines_out;
  size_t i = 0;
  auto offset = manager.data_offset.value_or(std::array<double, 3>({ 0,0,0 }));
  while (lasreader->read_point()) {
    if (lasreader->point.get_classification() == 2) {
      if (i++ % thin_nth == 0) {
        tree.insert(Point_d(lasreader->point.get_x() - offset[0], lasreader->point.get_y() - offset[1], lasreader->point.get_z() - offset[2]));
      }
      if (i % 100000 == 0) std::cout << "Read " << i << " points...\n";
    }
  }

  for (auto& line_string : lines) {
    vec3f ls;
    for (auto& p : line_string) {
      Point_d query(p[0], p[1], p[2]);
      Neighbor_search search(tree, query, 1);
      ls.push_back({ p[0], p[1], float(search.begin()->first.z()) });
    }
    lines_out.push_back(ls);
  }

  lasreader->close();
  delete lasreader;

  output("lines").set(lines_out);
}

void SimplifyLinesBufferNode::process() {
  typedef linesimp::AproximateLine AproximateLine;
  typedef linesimp::Point2 Point2;

  auto polygons = input("polygons").get<LinearRingCollection>();

  auto threshold = param<float>("threshold");
  LinearRingCollection polygonssimp;

  std::cout << "Creating simplified polylines by buffering and joining\n";
  for (auto& poly : polygons) {
    std::vector<AproximateLine> lines;
    // Create lines from polygon
    for (auto it = poly.begin(); it != poly.end() - 1; it++) {
      AproximateLine line = AproximateLine(Point2((*it)[0], (*it)[1]), Point2((*(it + 1))[0], (*(it + 1))[1]));
      lines.push_back(line);
    }
    
    // sort lines on length
    std::sort(lines.begin(), lines.end(), AproximateLine::compare);

    // Start merging lines
    std::cout << "Lines count: " << lines.size() << std::endl;
    int i = 0;
    while (i + 1 < lines.size()) {
      if (lines[i].canMerge(lines[i + 1], threshold)) {
        lines[i].merge(lines[i + 1]);
        auto it = lines.begin() + i + 1;
        lines.erase(it);
      }
      else {
        i++;
      }
    }
    std::cout << "Lines count: " << lines.size() << std::endl;

    // Create new lines

  }
  output("polygons_simp").set(polygonssimp);
}
}