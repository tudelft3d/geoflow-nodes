#include "cgal_nodes.hpp"

#include "tinsimp.hpp"
#include "linesimp.hpp"

#include <lasreader.hpp>
#include <fstream>

// CDT
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Triangulation_vertex_base_with_id_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

// AABB tree
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

// line simplification
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace geoflow;

template<typename T> inline std::array<float,3> to_arr3f(T& p) {
  return {float(p.x()), float(p.y()), float(p.z())};
}

void CDTNode::process(){
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Projection_traits_xy_3<K>								Gt;
  typedef CGAL::Exact_predicates_tag									Itag;
  typedef CGAL::Constrained_Delaunay_triangulation_2<Gt, CGAL::Default, Itag>	CDT;
  typedef CDT::Point													Point;

  // Set up vertex data (and buffer(s)) and attribute pointers
  auto lines = inputs("lines").get<LineStringCollection>();
  
  CDT cdt;

  for (auto& line : lines) {
    std::vector<Point> cgal_points;
    cgal_points.reserve(line.size());
    for (auto& p : line) {
      cgal_points.push_back(Point(p[0], p[1], p[2]));
    }
    cdt.insert_constraint( cgal_points.begin(), cgal_points.end() );
  }
  
  std::cout << "Completed CDT with " << cdt.number_of_faces() << " triangles...\n";
  // assert(cdt.is_valid());
  TriangleCollection triangles;
  for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
    fit != cdt.finite_faces_end();
    ++fit)
  {
    auto& p0 = fit->vertex(0)->point();
    auto& p1 = fit->vertex(1)->point();
    auto& p2 = fit->vertex(2)->point();
    triangles.push_back({
      to_arr3f<Point>(p0),
      to_arr3f<Point>(p1),
      to_arr3f<Point>(p2)
    });
  }

  // set_value("cgal_CDT", cdt);
  outputs("triangles").set(triangles);
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

  // Triangles 1
  auto trin1 = inputs("triangles1_vec3f").get<vec3f>();
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
  auto trin2 = inputs("triangles2_vec3f").get<vec3f>();
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
  

  outputs("points").set(points);
  outputs("diff").set(diff);
  outputs("distances1").set(distances1);
  outputs("distances2").set(distances2);
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

  auto trin = inputs("triangles").get<TriangleCollection>();
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
  while (lasreader->read_point()) {
    if (lasreader->point.get_classification() == 2){

      if (i++ % thin_nth == 0){
        auto q = Point(lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z());
        FT sqd = tree.squared_distance(q);
        distances.push_back(sqd);
        points.push_back({
          float(lasreader->point.get_x()), 
          float(lasreader->point.get_y()), 
          float(lasreader->point.get_z())}
        );
      }
      if(i%10000==0) std::cout << "Read " << i << " points...\n";
      // laswriter->write_point(&lasreader->point);
    }
  }
  lasreader->close();
  delete lasreader;

  outputs("points").set(points);
  outputs("distances").set(distances);
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
  auto geom_term = inputs("geometries");

  if (geom_term.connected_type == TT_line_string_collection) {
    auto lines = geom_term.get<geoflow::LineStringCollection>();
    outputs("dense_linestrings").set(densify_linestrings(lines, interval));
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
  auto geom_term = inputs("geometries");
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
    outputs("selected_lines").set(selected_lines);
    // outputs("count").set(line_counts);
    // outputs("error").set(line_errors);
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

  outputs("triangles").set(triangles);
  outputs("normals").set(normals);

}

void SimplifyLine3DNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto lines = inputs("lines").get<LineStringCollection>();

  LineStringCollection simplified_lines;
  for (auto& line_string : lines) {
    simplified_lines.push_back( linesimp::visvalingam(line_string, area_threshold) );
  }

  outputs("lines").set(simplified_lines);
}

void SimplifyLineNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto geometry = inputs("lines").get<gfGeometry3D>();
  
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

  size_t s_index=0;
  size_t sum_count=0,count=0;
  gfGeometry3D geometry_out;
  vec3f vertices_vec3f;
  for (auto c : geometry.counts) {
    std::vector<K::Point_2> line;
    std::cerr << "count: " <<c << "\n";
    for (size_t i=0; i<c; i++ ) {
      auto p = geometry.vertices[s_index+i];
      line.push_back(K::Point_2(p[0], p[1], p[2]));
      std::cerr << "\t "<< p[0] << ", " << p[1] << "\n";
    }
    s_index+=c;
    std::vector <K::Point_2> points_out;
    points_out.reserve(c);
    PS::simplify(line.begin(), line.end(), cost, Stop_cost(threshold_stop_cost), points_out.begin());
    
    auto points_begin = points_out.begin();
    auto points_end = points_out.end();
    size_t psize = points_out.size();
    for(auto pit = points_begin; pit != points_end; ++pit) {
      if(pit!=points_begin && pit!=points_end)
        vertices_vec3f.push_back({float(pit->x()), float(pit->y()), 0});
      vertices_vec3f.push_back({float(pit->x()), float(pit->y()), 0});
      geometry_out.vertices.push_back({float(pit->x()), float(pit->y()), 0});
      count++;
    }
    geometry_out.counts.push_back(count);
    geometry_out.firsts.push_back(sum_count);
    sum_count += count;
  }

  outputs("lines_vec3f").set(vertices_vec3f);
  outputs("lines").set(geometry_out);
}

void SimplifyLinesNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto geometry = inputs("lines").get<gfGeometry3D>();
  
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

  CT ct;

  size_t s_index=0;
  for (auto c : geometry.counts) {
    std::vector<CDT::Point> line;
    std::cerr << "count: " <<c << "\n";
    for (size_t i=0; i<c; i++ ) {
      auto p = geometry.vertices[s_index+i];
      line.push_back(CDT::Point(p[0], p[1]));
      std::cerr << "\t "<< p[0] << ", " << p[1] << "\n";
    }
    s_index+=c;
    ct.insert_constraint(line.begin(), line.end());
  }
  
  size_t n_removed = PS::simplify(ct, cost, Stop_cost(threshold_stop_cost));
  
  gfGeometry3D geometry_out;
  vec3f vertices_vec3f;
  for(auto cid = ct.constraints_begin(); cid != ct.constraints_end(); ++cid) {
    // auto p = (*ct.vertices_in_constraint_begin())->point();
    // vertices_vec3f.push_back({p.x(), p.y(), 0});
    auto vit_begin = ct.vertices_in_constraint_begin(*cid);
    auto vit_end = ct.vertices_in_constraint_end(*cid);
    size_t sum_count=0,count=0;
    for(auto vit = vit_begin; vit != vit_end; ++vit) {
      auto p = (*vit)->point();
      if(vit!=vit_begin && vit!=vit_end)
        vertices_vec3f.push_back({float(p.x()), float(p.y()), 0});
      vertices_vec3f.push_back({float(p.x()), float(p.y()), 0});
      geometry_out.vertices.push_back({float(p.x()), float(p.y()), 0});
      count++;
    }
    geometry_out.counts.push_back(count);
    geometry_out.firsts.push_back(sum_count);
    sum_count += count;
  }

  outputs("lines_vec3f").set(vertices_vec3f);
  outputs("lines").set(geometry_out);
}

void SimplifyFootprintNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers

  namespace PS = CGAL::Polyline_simplification_2;
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 Point_2;
  typedef CGAL::Polygon_2<K>                   Polygon_2;
  typedef PS::Stop_below_count_ratio_threshold Stop_count_ratio;
  typedef PS::Stop_above_cost_threshold        Stop_cost;
  typedef PS::Squared_distance_cost            Cost;

  auto polygons = inputs("polygons").get<LinearRingCollection>();

  LinearRingCollection polygons_out;
  
  for (auto& polygon : polygons) {
    Polygon_2 cgal_polygon;
    Cost cost;

    for (auto& p : polygon) {
      cgal_polygon.push_back(Point_2(p[0], p[1]));
    }
    // polygon.erase(polygon.vertices_end()-1); // remove repeated point from the boost polygon
    
    // polygon = PS::simplify(polygon, cost, Stop_count_ratio(0.5));

    cgal_polygon = PS::simplify(cgal_polygon, cost, Stop_cost(threshold_stop_cost));
    
    vec3f footprint_vec3f;
    for (auto v = cgal_polygon.vertices_begin(); v!=cgal_polygon.vertices_end(); v++){
      footprint_vec3f.push_back({float(v->x()),float(v->y()),0});
    }
    auto bv = cgal_polygon.vertices_begin(); // repeat first pt as last
    footprint_vec3f.push_back({float(bv->x()),float(bv->y()),0});
    polygons_out.push_back(footprint_vec3f);
  }
  outputs("polygons_simp").set(polygons_out);
}