#include "stepedge_nodes.hpp"
#include "region_growing_plane.h"
// #include "gloo.h"
#include "ptinpoly.h"
#include <earcut.hpp>

// #include "nlohmann/json.hpp"
// using json = nlohmann::json;

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include <boost/tuple/tuple.hpp>

#include <numeric>

// line simplification
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

// #include <filesystem>
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

// interval list
#include "interval.hpp"
#include "line_regulariser.hpp"

#include <unordered_set>
#include <stack>
#include <utility>

namespace as {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
  typedef CGAL::Projection_traits_xy_3<K>								       Gt;
  typedef K::FT                                                FT;
  // typedef K::Point_2                                           Point;
  // typedef K::Segment_2                                         Segment;
  typedef CGAL::Alpha_shape_vertex_base_2<Gt>                  Vb;
  typedef CGAL::Alpha_shape_face_base_2<Gt>                    Fb;
  // class AlphaShapeFaceWithLabel : public Fb {
  //   public: 
  //   int label = 0;
  //   bool visited = false;
  //   // using Fb::Fb;
  // };
  typedef CGAL::Triangulation_data_structure_2<Vb,Fb>          Tds;
  typedef CGAL::Delaunay_triangulation_2<Gt,Tds>               Triangulation_2;
  typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;
  typedef Alpha_shape_2::Vertex_handle                        Vertex_handle;
  typedef Alpha_shape_2::Edge                                 Edge;
  typedef Alpha_shape_2::Face_handle                          Face_handle;
  typedef Alpha_shape_2::Vertex_circulator                    Vertex_circulator;
  typedef Alpha_shape_2::Edge_circulator                      Edge_circulator;


  class AlphaShapeRegionGrower {
    Alpha_shape_2 &A;
    enum Mode {
      ALPHA, // stop at alpha boundary
      EXTERIOR // stop at faces labels as exterior
    };
    int label_cnt; // label==-1 means exterior, -2 mean never visiter, 0+ means a regular region
    public:
    std::unordered_map<Face_handle, int> face_map;
    std::unordered_map<int, Vertex_handle> region_map; //label: (boundary vertex)
    AlphaShapeRegionGrower(Alpha_shape_2& as) : A(as), label_cnt(0) {};

    void grow() {
      std::stack<Face_handle> seeds;
      for (auto fh = A.all_faces_begin(); fh!=A.all_faces_end(); ++fh) {
        seeds.push(fh);
        face_map[fh] = -2;
      }
      auto inf_face = A.infinite_face();
      face_map[inf_face] = -1;
      grow_region(inf_face, ALPHA); // this sets label of exterior faces to -1
      while (!seeds.empty()) {
        auto fh = seeds.top(); seeds.pop();
        if (face_map[fh] == -2) {
          face_map[fh] = label_cnt;
          grow_region(fh, EXTERIOR);
          ++label_cnt;
        }
      }
    }

    void grow_region (Face_handle face_handle, Mode mode) {
      std::stack<Face_handle> candidates;
      candidates.push(face_handle);

      while(candidates.size()>0) {
        auto fh = candidates.top(); candidates.pop();
        // check the 3 neighbors of this face
        for (int i=0; i<3; ++i) {
          auto e = std::make_pair(fh,i);
          auto neighbor = fh->neighbor(i);

          if (mode == ALPHA) {
            // add neighbor if it is not on the ohter side of alpha boundary
            // check if this neighbor hasn't been visited before
            if (face_map[neighbor] == -2) {
              auto edge_class = A.classify(e);
              if ( ! (edge_class == Alpha_shape_2::REGULAR || edge_class == Alpha_shape_2::SINGULAR) ) {
                face_map[neighbor] = -1;
                candidates.push(neighbor);
              }
            }
          } else if (mode == EXTERIOR) {
            // check if this neighbor hasn't been visited before and is not exterior
            if (face_map[neighbor] == -2) {
              face_map[neighbor] = label_cnt;
              candidates.push(neighbor);
            // if it is exterior, we store this boundary edge
            } else if (face_map[neighbor] == -1) {
              if( region_map.find(label_cnt)==region_map.end() ) {
                region_map[label_cnt] = fh->vertex(A.cw(i));
              }
            }
          }

        }
      }
    }
  };
}
namespace geoflow::nodes::stepedge {

void AlphaShapeNode::process(){
  auto points_per_segment = input("pts_per_roofplane").get<std::unordered_map<int, std::vector<Point>>>();

  auto thres_alpha = param<float>("thres_alpha");
  auto optimal_alpha = param<bool>("optimal_alpha");
  auto optimal_only_if_needed = param<bool>("optimal_only_if_needed");

  PointCollection edge_points, boundary_points;
  LineStringCollection alpha_edges;
  LinearRingCollection alpha_rings;
  TriangleCollection alpha_triangles;
  vec1i segment_ids, plane_idx;
  for (auto& it : points_per_segment ) {
    if (it.first == -1) continue; // skip points if they put at index -1 (eg if we care not about slanted surfaces for ring extraction)
    plane_idx.push_back(it.first);
    auto points = it.second;
    as::Triangulation_2 T;
    T.insert(points.begin(), points.end());
    as::Alpha_shape_2 A(T,
                as::FT(thres_alpha),
                as::Alpha_shape_2::GENERAL);
    
    if (optimal_alpha && optimal_only_if_needed) {
      thres_alpha = std::max(float(*A.find_optimal_alpha(1)), thres_alpha);
    } else if (optimal_alpha) {
      thres_alpha = *A.find_optimal_alpha(1);
    }
    A.set_alpha(as::FT(thres_alpha));

    for (auto it = A.alpha_shape_vertices_begin(); it!=A.alpha_shape_vertices_end(); it++) {
      auto p = (*it)->point();
      edge_points.push_back({float(p.x()), float(p.y()), float(p.z())});
    }
    for (auto it = A.alpha_shape_edges_begin(); it!=A.alpha_shape_edges_end(); it++) {
      auto p1 = it->first->vertex(A.cw(it->second))->point();
      auto p2 = it->first->vertex(A.ccw(it->second))->point();

      alpha_edges.push_back({
        {float(p1.x()), float(p1.y()), float(p1.z())},
        {float(p2.x()), float(p2.y()), float(p2.z())}
      });
    }
    
    // flood filling 
    auto grower = as::AlphaShapeRegionGrower(A);
    grower.grow();

    for (auto fh = A.finite_faces_begin(); fh != A.finite_faces_end(); ++fh) {
        arr3f p0 = {float (fh->vertex(0)->point().x()), float (fh->vertex(0)->point().y()), float (fh->vertex(0)->point().z())};
        arr3f p1 = {float (fh->vertex(1)->point().x()), float (fh->vertex(1)->point().y()), float (fh->vertex(1)->point().z())};
        arr3f p2 = {float (fh->vertex(2)->point().x()), float (fh->vertex(2)->point().y()), float (fh->vertex(2)->point().z())
        };
      alpha_triangles.push_back({ p0,p1,p2 });
      segment_ids.push_back(grower.face_map[fh]);
      segment_ids.push_back(grower.face_map[fh]);
      segment_ids.push_back(grower.face_map[fh]);
    }

    for (auto& kv : grower.region_map) {
      auto region_label = kv.first;
      auto v_start = kv.second;
      boundary_points.push_back({
        float(v_start->point().x()),
        float(v_start->point().y()),
        float(v_start->point().z())
      });

      // find edges of outer boundary in order
      LinearRing ring;

      ring.push_back( {float(v_start->point().x()), float(v_start->point().y()), float(v_start->point().z())} );
      // secondly, walk along the entire boundary starting from v_start
      as::Vertex_handle v_next, v_prev = v_start, v_cur = v_start;
      size_t v_cntr = 0;
      do {
        as::Edge_circulator ec(A.incident_edges(v_cur)), done(ec);
        do {
          // find the vertex on the other side of the incident edge ec
          auto v = ec->first->vertex(A.cw(ec->second));
          if (v_cur == v) v = ec->first->vertex(A.ccw(ec->second));
          // find labels of two adjacent faces
          auto label1 = grower.face_map[ ec->first ];
          auto label2 = grower.face_map[ ec->first->neighbor(ec->second) ];
          // check if the edge is on the boundary of the region and if we are not going backwards
          bool exterior = label1==-1 || label2==-1;
          bool region = label1==region_label || label2==region_label;
          if(( exterior && region )  && (v != v_prev)) {
            v_next = v;
            ring.push_back( {float(v_next->point().x()), float(v_next->point().y()), float(v_next->point().z())} );
            break;
          }
        } while (++ec != done);
        v_prev = v_cur;
        v_cur = v_next;

      } while (v_next != v_start);
      // finally, store the ring 
      alpha_rings.push_back(ring);
    }
  }
  
  output("alpha_rings").set(alpha_rings);
  output("alpha_edges").set(alpha_edges);
  output("alpha_triangles").set(alpha_triangles);
  output("segment_ids").set(segment_ids);
  output("edge_points").set(edge_points);
  output("boundary_points").set(boundary_points);
}

void Ring2SegmentsNode::process() {
  auto rings = input("rings").get<LinearRingCollection>();
  SegmentCollection segments;
  std::vector<std::vector<size_t>> ring_idx(rings.size());
  size_t ring_i=0;
  size_t seg_i=0;
  for (auto& ring : rings) {
    for (size_t i=0; i< ring.size()-1; ++i) {
      segments.push_back({ring[i], ring[i+1]});
      ring_idx[ring_i].push_back(seg_i++);
    }
    segments.push_back({ring[ring.size()-1], ring[0]});
    ring_idx[ring_i++].push_back(seg_i++);
  }
  output("edge_segments").set(segments);
  output("ring_idx").set(ring_idx);
}

void PolygonExtruderNode::process(){
  auto polygons = input("polygons").get<LinearRingCollection>();
  auto point_clouds = input("point_clouds").get<std::vector<PointCollection>>();

  if(polygons.size()!=point_clouds.size()) return;

  vec1f heights;
  for (auto& point_cloud : point_clouds) {
    double sum_elevation = 0;
    for (auto& p : point_cloud) {
      sum_elevation += p[2];
    }
    heights.push_back(sum_elevation/point_cloud.size());
  }
  output("polygons_extruded").set(polygons);
  output("heights").set(heights);
}

void Arr2LinearRingsNode::process(){
  auto arr = input("arrangement").get<Arrangement_2>();

  LinearRingCollection linear_rings;
  AttributeMap attributes;
  for (auto face: arr.face_handles()){
    if(face->data().in_footprint) {
      vec2f polygon;
      arrangementface_to_polygon(face, polygon);
      vec3f polygon3d;
      for (auto& p : polygon) {
        polygon3d.push_back({p[0],p[1],0});
      }
      linear_rings.push_back(polygon3d);
      attributes["height"].push_back(face->data().elevation_avg);
      // attributes["rms_error"].push_back(face->data().rms_error_to_avg);
      // attributes["max_error"].push_back(face->data().max_error);
      // attributes["coverage"].push_back(face->data().segid_coverage);
      attributes["count"].push_back(face->data().total_count);
      attributes["segid"].push_back(face->data().segid);
    }
  }
  output("linear_rings").set(linear_rings);
  output("attributes").set(attributes);
}

void ExtruderNode::process(){
  // if (!(do_walls || do_roofs)) return;
  // Set up vertex data (and buffer(s)) and attribute pointers
  // auto polygons = std::any_cast<input("polygons").get<vec2f>>();
  // auto elevations = std::any_cast<input("elevations").get<float>>();
  auto arr = input("arrangement").get<Arrangement_2>();

  TriangleCollection triangles;
  vec3f normals;
  vec1i cell_id_vec1i, plane_id;
  vec1i labels;
  vec1f rms_errors, max_errors, segment_coverages, elevations;
  using N = uint32_t;

  
  size_t cell_id=0, pid;
  float rms_error, max_error, segment_coverage;
  for (auto face: arr.face_handles()){
    // bool extract = param<bool>("in_footprint") ? face->data().in_footprint : face->data().is_finite;
    bool extract = face->data().in_footprint;
    if(extract) {
      cell_id++;
      rms_error = face->data().rms_error_to_avg;
      max_error = face->data().max_error;
      segment_coverage = face->data().segid_coverage;
      pid = face->data().segid;
      vec2f polygon, vertices;
      arrangementface_to_polygon(face, polygon);
      std::vector<N> indices = mapbox::earcut<N>(std::vector<vec2f>({polygon}));
      for(auto i : indices) {
        vertices.push_back({polygon[i]});
      }
      // floor triangles
      for (size_t i=0; i<indices.size()/3; ++i) {
        Triangle triangle;
        for (size_t j=0; j<3; ++j) {
          triangle[j] = {vertices[i*3+j][0], vertices[i*3+j][1], 0};
          labels.push_back(0);
          normals.push_back({0,0,-1});
          cell_id_vec1i.push_back(cell_id);
          plane_id.push_back(pid);
          rms_errors.push_back(rms_error);
          elevations.push_back(face->data().elevation_avg);
          max_errors.push_back(max_error);
          segment_coverages.push_back(segment_coverage);
        }
        triangles.push_back(triangle);
      }
      if (do_roofs) {
        // roof triangles
        for (size_t i=0; i<indices.size()/3; ++i) {
          Triangle triangle;
          for (size_t j=0; j<3; ++j) {
            triangle[j] = {vertices[i*3+j][0], vertices[i*3+j][1], face->data().elevation_avg};
            labels.push_back(1);
            normals.push_back({0,0,1});
            cell_id_vec1i.push_back(cell_id);
            plane_id.push_back(pid);
            // rms_errors.push_back(rms_error);
            rms_errors.push_back(rms_error);
            elevations.push_back(face->data().elevation_avg);
            max_errors.push_back(max_error);
            segment_coverages.push_back(segment_coverage);
          }
          triangles.push_back(triangle);
        }
      }
    }
  }
  if (do_walls) {
    vertex n;
    for (auto edge : arr.edge_handles()) {
      // skip if faces on both sides of this edge are not finite
      // bool left = edge->twin()->face()->data().in_footprint;
      // bool right = edge->face()->data().in_footprint;
      bool left = edge->twin()->face()->data().in_footprint;
      bool right = edge->face()->data().in_footprint;
      if (left || right) {
        int wall_label = 2;
        if (left && right)
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
        triangles.push_back({u1,l2,l1});
        labels.push_back(wall_label);
        // triangles.push_back(l2);
        labels.push_back(wall_label);
        // triangles.push_back(l1);
        labels.push_back(wall_label);

        n = get_normal(u1,l2,l1);
        normals.push_back(n);
        normals.push_back(n);
        normals.push_back(n);

        cell_id_vec1i.push_back(0);
        plane_id.push_back(0);
        cell_id_vec1i.push_back(0);
        plane_id.push_back(0);
        cell_id_vec1i.push_back(0);
        plane_id.push_back(0);
        rms_errors.push_back(-1);
        elevations.push_back(-1);
        rms_errors.push_back(-1);
        elevations.push_back(-1);
        rms_errors.push_back(-1);
        elevations.push_back(-1);
        max_errors.push_back(-1);
        max_errors.push_back(-1);
        max_errors.push_back(-1);
        segment_coverages.push_back(-1);
        segment_coverages.push_back(-1);
        segment_coverages.push_back(-1);

        // 2nd triangle
        triangles.push_back({u1,u2,l2});
        labels.push_back(wall_label);
        // triangles.push_back(u2);
        labels.push_back(wall_label);
        // triangles.push_back(l2);
        labels.push_back(wall_label);

        n = get_normal(u1,u2,l2);
        normals.push_back(n);
        normals.push_back(n);
        normals.push_back(n);

        cell_id_vec1i.push_back(0);
        plane_id.push_back(0);
        cell_id_vec1i.push_back(0);
        plane_id.push_back(0);
        cell_id_vec1i.push_back(0);
        plane_id.push_back(0);
        rms_errors.push_back(-1);
        elevations.push_back(-1);
        rms_errors.push_back(-1);
        elevations.push_back(-1);
        rms_errors.push_back(-1);
        elevations.push_back(-1);
        max_errors.push_back(-1);
        max_errors.push_back(-1);
        max_errors.push_back(-1);
        segment_coverages.push_back(-1);
        segment_coverages.push_back(-1);
        segment_coverages.push_back(-1);
      }
    }
  }
  
  output("normals_vec3f").set(normals);
  output("cell_id_vec1i").set(cell_id_vec1i);
  output("plane_id").set(plane_id);
  output("rms_errors").set(rms_errors);
  output("max_errors").set(max_errors);
  output("elevations").set(elevations);
  output("segment_coverages").set(segment_coverages);
  output("triangles").set(triangles);
  output("labels_vec1i").set(labels);
}

void ProcessArrangementNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto points = input("points").get<PNL_vector>();
  auto arr = input("arrangement").get<Arrangement_2>();

  config c;
  c.step_height_threshold = param<float>("step_height_threshold");
  c.zrange_threshold = param<float>("zrange_threshold");
  c.merge_segid = param<bool>("merge_segid");
  c.merge_zrange = param<bool>("merge_zrange");
  c.merge_step_height = param<bool>("merge_step_height");
  c.merge_unsegmented = param<bool>("merge_unsegmented");
  c.merge_dangling_egdes = param<bool>("merge_dangling_egdes");

  process_arrangement(points, arr, c);
  
  output("arrangement").set(arr);
}

void arr2segments(Face_handle& face, LineStringCollection& segments) {
  auto he = face->outer_ccb();
  auto first = he;

  while(true){
    segments.push_back({
      {
        float(CGAL::to_double(he->source()->point().x())),
        float(CGAL::to_double(he->source()->point().y())),
        0
      },{
        float(CGAL::to_double(he->target()->point().x())),
        float(CGAL::to_double(he->target()->point().y())),
        0
      }
    });

    he = he->next();
    if (he==first) break;
  }
  segments.push_back({
    {
      float(CGAL::to_double(he->source()->point().x())),
      float(CGAL::to_double(he->source()->point().y())),
      0
    },{
      float(CGAL::to_double(he->target()->point().x())),
      float(CGAL::to_double(he->target()->point().y())),
      0
    }
  });
}

void BuildArrangementNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto footprint = input("footprint").get<LinearRing>();
  auto geom_term = input("edge_segments");

  Arrangement_2 arr;
  auto edge_segments = geom_term.get<LineStringCollection>();
  build_arrangement(footprint, edge_segments, arr, remove_unsupported);
  LineStringCollection segments;
  for (auto& face: arr.face_handles()){
    if (face->data().in_footprint)
      arr2segments(face, segments);
  }
  output("arr_segments").set(segments);
  output("arrangement").set(arr);
};

void LinearRingtoRingsNode::process(){
  auto lr = input("linear_ring").get<LinearRing>();
  LinearRingCollection lrc;
  lrc.push_back(lr);
  output("linear_rings").set(lrc);
}

Polygon_2 arr_cell2polygon(Face_handle& fh) {
  Polygon_2 poly;
  auto he = fh->outer_ccb();
  auto first = he;
  do {
    poly.push_back(he->target()->point());
    he = he->next();
  } while (he!=first);
  return poly;
}
void arr_process(Arrangement_2& arr, const bool& flood_unsegmented, const bool& dissolve_edges, const bool& dissolve_stepedges, const float& step_height_threshold) {
  if (flood_unsegmented) {
    std::map<float, Face_handle> face_map;
    for (auto& face : arr.face_handles()) {
      if (face->data().segid!=0)
        face_map[face->data().elevation_avg] = face;
    }
    for (auto& kv : face_map) {
      std::stack<Face_handle> candidate_stack;
      // std::cout << "Growing face with elevation=" << kv.first << "\n";
      auto& cur_segid = kv.second->data().segid;
      auto& cur_elev = kv.second->data().elevation_avg;
      candidate_stack.push(kv.second);
      while (!candidate_stack.empty()) {
        auto fh = candidate_stack.top(); candidate_stack.pop();
        auto he = fh->outer_ccb();
        auto first = he;
        do {
          // std::cout << &(*curr) << "\n";
          // ignore weird nullptrs (should not be possible...)
          if (he==nullptr) break;
          auto candidate = he->twin()->face();
          if (candidate->data().segid == 0) {
            candidate->data().segid = cur_segid;
            candidate->data().elevation_avg = cur_elev;
            candidate_stack.push(candidate);
          }
          he = he->next();
        } while (he != first);
      }
    }
  }
  //remove edges that have the same segid on both sides
  if (dissolve_edges) {
    std::vector<Halfedge_handle> to_remove;
    for (auto he : arr.edge_handles()) {
      auto d1 = he->face()->data();
      auto d2 = he->twin()->face()->data();
      if ((d1.segid == d2.segid )&& (d1.in_footprint && d2.in_footprint))
        to_remove.push_back(he);
    }
    for (auto he : to_remove) {
      arr.remove_edge(he);
    }
  }
  if (dissolve_stepedges) {
    std::vector<Arrangement_2::Halfedge_handle> edges;
    for (auto edge : arr.edge_handles()) {
      edges.push_back(edge);
    }
    for (auto& edge : edges) {
      auto f1 = edge->face();
      auto f2 = edge->twin()->face();
      if((f1->data().in_footprint && f2->data().in_footprint) && (f1->data().segid!=0 && f2->data().segid!=0)) {
        if(std::abs(f1->data().elevation_avg - f2->data().elevation_avg) < step_height_threshold){
          // should add face merge call back in face observer class...
          // pick elevation of the segment with the highest count
          if (f2->data().elevation_avg < f1->data().elevation_avg) {
            f2->data().elevation_avg = f1->data().elevation_avg;
            f2->data().segid = f1->data().segid;
          } else {
            f1->data().elevation_avg = f2->data().elevation_avg;
            f1->data().segid = f2->data().segid;
          }
          arr.remove_edge(edge);
        }
      }
    }
  }
}

void arr_filter_biggest_face(Arrangement_2& arr, const float& rel_area_thres) {
  // check number of faces
  typedef std::pair<Polygon_2, double> polyar;
  std::vector<polyar> polygons;
  double total_area=0;
  for (auto& fh : arr.face_handles()) {
    if (fh->data().segid != 0 || fh->data().in_footprint == true) {
      auto poly = arr_cell2polygon(fh);
      double area = CGAL::to_double(CGAL::abs(poly.area()));
      total_area += area;
      polygons.push_back(std::make_pair(poly, area));
    }
  }
  std::sort(polygons.begin(), polygons.end(), [](const polyar& a, const polyar& b) {
    return a.second < b.second;   
  });
  arr.clear();
  for (auto& poly_a : polygons) {
    if (poly_a.second > rel_area_thres * total_area)
      insert_non_intersecting_curves(arr, poly_a.first.edges_begin(), poly_a.first.edges_end());
  }
}
std::pair<double, double> arr_measure_nosegid(Arrangement_2& arr) {
  // check number of faces
  double total_area=0;
  double no_segid_area=0;
  for (auto& fh : arr.face_handles()) {
    if (fh->data().in_footprint) {
      auto poly = arr_cell2polygon(fh);
      double area = CGAL::to_double(CGAL::abs(poly.area()));
      if (fh->data().segid == 0) {
        no_segid_area += area;
      }
      total_area += area;
    }
  }
  return std::make_pair(no_segid_area, no_segid_area/total_area);
}
// size_t get_percentile(const std::vector<linedect::Point>& points, const float& percentile) {
//   std::sort(points.begin(), points.end(), [](linedect::Point& p1, linedect::Point& p2) {
//     return p1.z() < p2.z();
//   });
//   return int(percentile*float(points.size()/2));
// }
void arr_assign_pts_to_unsegmented(Arrangement_2& arr, std::vector<Point>& points, const float& percentile) {
  typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2> Point_location;

  std::unordered_map<Face_handle, std::vector<Point>> points_per_face;

  // collect for each face the points it contains
  Point_location pl(arr);
  for (auto& p : points){
    auto obj = pl.locate( Point_2(p.x(), p.y()) );
    if (auto f = boost::get<Face_const_handle>(&obj)) {
      auto fh = arr.non_const_handle(*f);
      if (fh->data().segid==0) {
        points_per_face[fh].push_back(p);
      }
    }
  }
  // find elevation percentile for each face
  for(auto& ppf : points_per_face) {
    if (ppf.second.size()>0) {
      std::sort(ppf.second.begin(), ppf.second.end(), [](linedect::Point& p1, linedect::Point& p2) {
        return p1.z() < p2.z();
      });
      auto pid = int(percentile*float(ppf.second.size()-1));
      // auto pid = get_percentile(ppf.second, percentile);
      ppf.first->data().segid = -1;
      ppf.first->data().elevation_avg = ppf.second[pid].z();
    }
  }
}

void BuildArrFromRingsExactNode::process() {
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto rings = input("rings").get<std::vector<linereg::Polygon_2>>();
  // auto plane_idx = input("plane_idx").get<vec1i>();
  auto points_per_plane = input("pts_per_roofplane").get<std::unordered_map<int, std::vector<Point>>>();

  auto fp_in = input("footprint");
  linereg::Polygon_2 footprint;
  if (fp_in.connected_type == TT_any)
    footprint = fp_in.get<linereg::Polygon_2>();
  else {
    auto& lr = fp_in.get<LinearRing&>();
    for (auto& p : lr) {
      footprint.push_back(linereg::EK::Point_2(p[0], p[1]));
    }
  }

  Arrangement_2 arr_base;
  {
    Face_index_observer obs (arr_base, true, 0, 0);
    insert(arr_base, footprint.edges_begin(), footprint.edges_end());
    // insert_non_intersecting_curves(arr_base, footprint.edges_begin(), footprint.edges_end());
    if (!footprint.is_simple()) {
      arr_filter_biggest_face(arr_base, param<float>("rel_area_thres"));
    }
  }
  // insert step-edge lines
  {
    Arrangement_2 arr_overlay;
    size_t i=0;
    // NOTE: rings and points_per_plane must be aligned!! (matching length and order)
    for (auto& kv : points_per_plane) {
      if (kv.first==-1) continue;
      auto& polygon = rings[i++];
      if (polygon.size()>2) {
        auto plane_id = kv.first;
        auto& points = kv.second;
        std::sort(points.begin(), points.end(), [](linedect::Point& p1, linedect::Point& p2) {
          return p1.z() < p2.z();
        });
        int elevation_id = std::floor(param<float>("z_percentile")*float(points.size()-1));

        // wall_planes.push_back(std::make_pair(Plane(s.first, s.second, s.first+Vector(0,0,1)),0));
        Arrangement_2 arr;
        Face_index_observer obs (arr, false, plane_id, points[elevation_id].z());
        insert(arr, polygon.edges_begin(), polygon.edges_end());

        if (!polygon.is_simple()) {
          arr_filter_biggest_face(arr, param<float>("rel_area_thres"));
        }

        Overlay_traits overlay_traits;
        arr_overlay.clear();
        overlay(arr_base, arr, arr_overlay, overlay_traits);
        arr_base = arr_overlay;
      }
    }
  }
  // fix unsegmented face: 1) sort segments on elevation, from low to high, 2) starting with lowest segment grow into unsegmented neighbours

  if(param<bool>("extrude_unsegmented") && points_per_plane.count(-1)) {
    arr_assign_pts_to_unsegmented(arr_base, points_per_plane[-1], param<float>("z_percentile"));
  }
  
  auto nosegid_area = arr_measure_nosegid(arr_base);
  
  arr_process(arr_base, 
    param<bool>("flood_to_unsegmented"), 
    param<bool>("dissolve_edges"),
    param<bool>("dissolve_stepedges"),
    param<float>("step_height_threshold")
  );

  LineStringCollection segments;
  for (auto& face: arr_base.face_handles()){
    if (face->data().in_footprint)
      arr2segments(face, segments);
  }
  output("noseg_area_a").set(float(CGAL::sqrt(nosegid_area.first)));
  output("noseg_area_r").set(float(nosegid_area.second));
  output("arr_segments").set(segments);
  output("arrangement").set(arr_base);
}

void BuildArrFromRingsNode::process() {
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto footprint = input("footprint").get<LinearRing>();
  auto rings = input("rings").get<LinearRingCollection>();
  // auto plane_idx = input("plane_idx").get<vec1i>();
  auto points_per_plane = input("pts_per_roofplane").get<std::unordered_map<int, std::vector<Point>>>();


  Arrangement_2 arr_base;
  Polygon_2 cgal_footprint = ring_to_cgal_polygon(footprint);
  if (cgal_footprint.is_simple()) {
      
    // std::cout << "fp size=" <<footprint_pts.size() << "; " << footprint_pts[0].x() <<","<<footprint_pts[0].y()<<"\n";
    {
      Face_index_observer obs (arr_base, true, 0, 0);
      // insert(arr_base, cgal_footprint.edges_begin(), cgal_footprint.edges_end());
      insert_non_intersecting_curves(arr_base, cgal_footprint.edges_begin(), cgal_footprint.edges_end());
    }
    // insert step-edge lines
    {
      Arrangement_2 arr_overlay;
      size_t i=0;
      // NOTE: rings and points_per_plane must be aligned!! (matching length and order)
      for (auto& kv : points_per_plane) {
        auto& ring = rings[i++];
        if (ring.size()>2) {
          auto polygon = ring_to_cgal_polygon(ring);
          if (polygon.is_simple()) {
            auto plane_id = kv.first;
            auto& points = kv.second;
            std::sort(points.begin(), points.end(), [](linedect::Point& p1, linedect::Point& p2) {
              return p1.z() < p2.z();
            });
            auto elevation_id = int(param<float>("z_percentile")*float(points.size()));

            // wall_planes.push_back(std::make_pair(Plane(s.first, s.second, s.first+Vector(0,0,1)),0));
            Arrangement_2 arr;
            Face_index_observer obs (arr, false, plane_id, points[elevation_id].z());
            insert(arr, polygon.edges_begin(), polygon.edges_end());

            Overlay_traits overlay_traits;
            arr_overlay.clear();
            overlay(arr_base, arr, arr_overlay, overlay_traits);
            arr_base = arr_overlay;
          } else std::cout << "This alpha ring is no longer simple after regularisation!\n";
          // std::cout << "overlay success\n";
          // std::cout << "facecount: " << arr_base.number_of_faces() << "\n\n";
        }
      }
    }
  } else {
    std::cout << "This polygon is no longer simple after regularisation!\n";
  }
  // fix unsegmented face: 1) sort segments on elevation, from low to high, 2) starting with lowest segment grow into unsegmented neighbours
  arr_process(arr_base, 
    param<bool>("flood_to_unsegmented"), 
    param<bool>("dissolve_edges"),
    param<bool>("dissolve_stepedges"),
    param<float>("step_height_threshold")
  );

  LineStringCollection segments;
  for (auto& face: arr_base.face_handles()){
    if (face->data().in_footprint)
      arr2segments(face, segments);
  }
  output("arr_segments").set(segments);
  output("arrangement").set(arr_base);
}

// inline int circindex(int i, size_t& N) {
//   // https://codereview.stackexchange.com/questions/57923/index-into-array-as-if-it-is-circular
//   bool wasNegative = false;
//   if (i < 0) {
//       wasNegative = true;
//       i = -i; //there's definitely a bithack for this.
//   }
//   int offset = i % N;
//   return (wasNegative) ? (N - offset) : (offset);
// }

inline void DetectLinesNode::detect_lines_ring_m1(linedect::LineDetector& LD, SegmentCollection& segments_out) {
  LD.dist_thres = param<float>("dist_thres") * param<float>("dist_thres");
  LD.N = param<int>("k");
  auto& c_upper = param<int>("min_cnt_upper");
  auto& c_lower = param<int>("min_cnt_lower");
  std::vector<size_t> detected_regions;
  size_t ringsize = LD.point_segment_idx.size();
  RingSegMap ring_seg_map;
  for (size_t i=c_upper; i>=c_lower; --i){
    LD.min_segment_count = i;
    auto new_regions = LD.detect();

    // extract the new regions from the rring
    std::vector<size_t> reset_ids;
    for (const auto& cur_reg : new_regions) {
      bool stitch = (cur_reg != LD.point_segment_idx.back()) && (cur_reg != LD.point_segment_idx[0]);

      std::vector<std::vector<size_t>> other_regs(1);
      size_t j=0;
      for (auto& regid : LD.point_segment_idx) {
        if (regid != cur_reg) {
          other_regs.back().push_back(j);
        } else {
          other_regs.resize(other_regs.size()+1);
        }
        ++j;
      }
      if (stitch) {
        auto& front = other_regs.front();
        auto& back = other_regs.back();
        front.insert(front.begin(), back.begin(), back.end());
        other_regs.pop_back();
      }
      size_t largest = 0, largest_id;
      j=0;
      for (const auto& other_reg : other_regs) {
        auto s = other_reg.size();
        if (s >largest) {
          largest=s;
          largest_id=j;
        } ++j;
      }
      auto idend = other_regs[largest_id].front()-1 % ringsize;
      auto idstart = other_regs[largest_id].back()+1 % ringsize;
      ring_seg_map[std::make_pair(idstart,idend)] = cur_reg;
      reset_ids.push_back(idend);
      reset_ids.push_back(idstart);
    }
    for (const auto& j : reset_ids) {
      LD.point_segment_idx[j] = 0;
    }
  }
  for (auto& [seg,rid] : ring_seg_map) {
    segments_out.push_back(LD.project(seg.first, seg.second));
  }
}
inline void DetectLinesNode::detect_lines(linedect::LineDetector& LD) {
  LD.dist_thres = param<float>("dist_thres") * param<float>("dist_thres");
  LD.N = param<int>("k");
  auto& c_upper = param<int>("min_cnt_upper");
  auto& c_lower = param<int>("min_cnt_lower");
  for (size_t i=c_upper; i>=c_lower; --i){
    LD.min_segment_count = i;
    LD.detect();
  }
}
inline size_t DetectLinesNode::detect_lines_ring_m2(linedect::LineDetector& LD, SegmentCollection& segments_out) {
  LD.dist_thres = param<float>("dist_thres") * param<float>("dist_thres");
  LD.N = param<int>("k");
  auto& c_upper = param<int>("min_cnt_upper");
  auto& c_lower = param<int>("min_cnt_lower");
  for (size_t i=c_upper; i>=c_lower; --i){
    LD.min_segment_count = i;
    LD.detect();
  }
  size_t ringsize = LD.point_segment_idx.size();
            // chain the detected lines, to ensure correct order
  if (LD.segment_shapes.size()>1) {
    std::vector<std::pair<size_t,size_t>> new_ring_ids;
    bool start_seg = LD.point_segment_idx[0];
    int prev_i=ringsize-1,
      prev_seg=LD.point_segment_idx[prev_i], 
      cur_seg, 
      i_last_seg = -1;
    bool perfect_aligned=false; // is the first point of the ring also the first point of a segment? If yes, we are perfectly aligned!
    for( int i=0; i<ringsize; ++i ) {
      cur_seg = LD.point_segment_idx[i];
      if(cur_seg==prev_seg && cur_seg!=0) { // we are inside a segment

      } else if (cur_seg!=0 && prev_seg==0) { // from unsegmented to segmented
        new_ring_ids.push_back(std::make_pair(i, cur_seg)); // first of cur
        if(i==0) perfect_aligned=true;
        // new_ring_ids.push_back(i); // end of unsegmented linesegment
      } else if (cur_seg!=0 && prev_seg!=0) { // from one segment to another
        new_ring_ids.push_back(std::make_pair(prev_i, prev_seg)); // last of prev
        new_ring_ids.push_back(std::make_pair(i, cur_seg)); // first of cur
      } else if (cur_seg==0 && prev_seg!=0) { //from segment to unsegmented
        new_ring_ids.push_back(std::make_pair(prev_i, prev_seg)); // last of prev
        // new_ring_ids.push_back(prev_i); // begin of unsegmented linesegment
      } // else: we are inside an unsegmented or segmented zone
      prev_seg = cur_seg;
      prev_i = i;
    }
    if (!perfect_aligned) { // add the segment that runs through the first point in the original ring
      new_ring_ids.insert(new_ring_ids.begin(), new_ring_ids.back());
      new_ring_ids.pop_back();
    }
    //ensure the ring is aligned wrt diff region ids around origin of the ring
    if (new_ring_ids.front().second == new_ring_ids.back().second) {
      size_t region = new_ring_ids.front().second;
      do {
        new_ring_ids.push_back(new_ring_ids.front());
        new_ring_ids.erase(new_ring_ids.begin());
      } while (region == new_ring_ids.front().second);
    }
    //merge multiple segments of the same region
    std::unordered_map<size_t,std::pair<size_t,size_t>> map_by_region;
    for (auto el = new_ring_ids.begin(); el<new_ring_ids.end(); ++el) {
      if(!map_by_region.count(el->second)) {
        map_by_region[el->second] = std::make_pair(el->first, el->first);
      } else {
        map_by_region[el->second].second = el->first;
      }
    }
    //sort the segments acc to order in ring
    typedef std::set<std::pair<size_t,size_t>,Cmp> SegSet;
    SegSet sorted_segments;
    for (auto& el : map_by_region) {
      sorted_segments.insert(el.second);
    }
    // TODO: better check for overlapping segments! Following is not 100% robust...
    auto el_prev = sorted_segments.begin();
    // el_prev.first-=sorted_segments.size();
    // el_prev.second-=sorted_segments.size();
    std::vector<SegSet::iterator> to_remove;
    for (auto el = ++sorted_segments.begin(); el != sorted_segments.end(); ++el ){
      el_prev = el;
      --el_prev;
      if (el_prev->second > el->first)
        to_remove.push_back(el);
    }
    for (auto el : to_remove) {
      sorted_segments.erase(el);
    }
    if (param<bool>("perform_chaining")) {
      std::vector<SCK::Segment_2> prechain_segments;
      std::vector<size_t> idx; size_t idcnt=0;
      for (auto& [i0,i1] : sorted_segments) {
        // segments_out.push_back( LD.project(i0, i1) );
        prechain_segments.push_back( LD.project_cgal(i0, i1, param<float>("line_extend")) );
        idx.push_back(idcnt++);
      }
      // TODO: chain the ring? for better regularisation results
      auto chained_segments = linereg::chain_ring<SCK>(idx, prechain_segments, param<float>("snap_threshold"));

      // for (auto e=prechain_segments.begin(); e!=prechain_segments.end(); ++e) {
      for (auto e=chained_segments.edges_begin(); e!=chained_segments.edges_end(); ++e) {
        segments_out.push_back({
          arr3f{
            float(CGAL::to_double(e->source().x())),
            float(CGAL::to_double(e->source().y())),
            0},
          arr3f{
            float(CGAL::to_double(e->target().x())),
            float(CGAL::to_double(e->target().y())),
            0},
        });
      }
      return chained_segments.size();
    } else {
      return sorted_segments.size();
    }
  } else return 0;
  
}

void DetectLinesNode::process(){
  auto input_geom = input("edge_points");

  SegmentCollection edge_segments;
  vec1i ring_order, ring_id, is_start;
  std::vector<std::vector<size_t>> ring_idx;
  // fit lines in all input points
  if (input_geom.connected_type == TT_point_collection) {
    std::vector<linedect::Point> cgal_pts;
    auto points = input_geom.get<PointCollection>();
    for( auto& p : points ) {
      cgal_pts.push_back(linedect::Point(p[0], p[1], p[2]));
    }
    linedect::LineDetector LD(cgal_pts);
    detect_lines(LD);
    LD.get_bounded_edges(edge_segments);

  // fit lines per ring
  } else if (input_geom.connected_type == TT_linear_ring_collection) {
    auto rings = input_geom.get<LinearRingCollection>();
    int n = param<int>("k");
    ring_idx.resize(rings.size());
    
    size_t ring_cntr=0;
    size_t seg_cntr=0;
    for (auto& ring : rings) {
      
      std::vector<linedect::Point> cgal_pts;
      for( auto& p : ring ) {
        cgal_pts.push_back(linedect::Point(p[0], p[1], p[2]));
      }

      if (param<bool>("linear_knn")) {
        int kb = n/2; //backward neighbors
        int kf = n-kb-1; //forward neighbours

        linedect::NeighbourVec neighbours;
        for( int i=0; i<ring.size(); ++i ) {
          std::vector<size_t> idx;
          for (int j = i-kb; j <= i+kf; ++j) {
            if (j<0)
              idx.push_back( n+(j%n) );
            else
              idx.push_back( j%n );
          }
          neighbours.push_back(idx);
        }
        linedect::LineDetector LD(cgal_pts, neighbours);
        detect_lines(LD);
        LD.get_bounded_edges(edge_segments);
      } else {
        
        linedect::LineDetector LD(cgal_pts);
        // SegmentCollection ring_edges;
        auto n_detected = detect_lines_ring_m2(LD, edge_segments);
        // LD.get_bounded_edges(edge_segments);

        for (size_t j=0; j<n_detected; ++j) {
          // edge_segments.push_back(ring_edges[j]);
          ring_idx[ring_cntr].push_back(seg_cntr++);
          ring_order.push_back(j);
          ring_id.push_back(ring_cntr);
          ring_order.push_back(j);
          ring_id.push_back(ring_cntr);
          is_start.push_back(1);
          is_start.push_back(0);
        }
        ++ring_cntr;
        // std::cout << "number of shapes: " << LD.segment_shapes.size() <<"\n";
        // std::cout << "number of segments: " << order_cnt <<"\n";
      }
    }
  }

  output("edge_segments").set(edge_segments);
  // output("ring_edges").set(ring_edges);
  output("ring_idx").set(ring_idx);
  output("ring_id").set(ring_id);
  output("ring_order").set(ring_order);
  output("is_start").set(is_start);
}

void ClassifyEdgePointsNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto points = input("points").get<PNL_vector>();

  config c;
  c.classify_jump_count_min = param<int>("classify_jump_count_min");
  c.classify_jump_count_max = param<int>("classify_jump_count_max");
  c.classify_line_dist = param<float>("classify_line_dist");
  c.classify_jump_ele = param<float>("classify_jump_ele");

  std::vector<linedect::Point> edge_points;
  classify_edgepoints(edge_points, points, c);
  output("edge_points").set(edge_points);

  vec3f edge_points_vec3f;
  for(auto& p : edge_points) {
      std::array<float,3> a = {{
        float(p.x()), 
        float(p.y()), 
        float(p.z())
      }};
      edge_points_vec3f.push_back(a);
    }
  output("edge_points_vec3f").set(edge_points_vec3f);
}

void DetectPlanesNode::process() {
  auto points = input("points").get<PointCollection>();

  auto metrics_normal_k = param<int>("metrics_normal_k");
  auto metrics_plane_min_points = param<int>("metrics_plane_min_points");
  auto metrics_plane_epsilon = param<float>("metrics_plane_epsilon");
  auto metrics_plane_normal_threshold = param<float>("metrics_plane_normal_threshold");
  auto metrics_is_wall_threshold = param<float>("metrics_is_wall_threshold");
  auto metrics_is_horizontal_threshold = param<float>("metrics_is_horizontal_threshold");

  // convert to cgal points with attributes
  PNL_vector pnl_points;
  for (auto& p : points) {
    PNL pv;
    boost::get<0>(pv) = Point(p[0], p[1], p[2]);
    boost::get<2>(pv) = 0;
    boost::get<3>(pv) = 0;
    boost::get<9>(pv) = 0;
    pnl_points.push_back(pv);
  }
  // estimate normals
  CGAL::pca_estimate_normals<Concurrency_tag>(
    pnl_points, metrics_normal_k,
    CGAL::parameters::point_map(Point_map()).
    normal_map(Normal_map())
  );
  // orient normals upwards
  auto up = Vector(0,0,1);
  for ( auto& pv : pnl_points) {
    auto &n = boost::get<1>(pv);
    if (n*up<0) 
      boost::get<1>(pv) = -n;
  }

  // convert to lists required by the planedetector class
  // size_t i=0;
  std::vector<Point> points_vec;
  std::vector<Vector> normals_vec;
  points_vec.reserve(points.size());
  for (auto &p : pnl_points) {
    points_vec.push_back(boost::get<0>(p));
    normals_vec.push_back(boost::get<1>(p));
  }
  // perform plane detection
  planedect::PlaneDetector PD(points_vec, normals_vec);
  PD.dist_thres = metrics_plane_epsilon * metrics_plane_epsilon;
  PD.normal_thres = metrics_plane_normal_threshold;
  PD.min_segment_count = metrics_plane_min_points;
  PD.N = metrics_normal_k;
  PD.n_refit = param<int>("n_refit");
  PD.detect();

  // classify horizontal/vertical planes using plane normals
  std::unordered_map<int, std::vector<Point>> pts_per_roofplane;
  size_t horiz_roofplane_cnt=0;
  size_t slant_roofplane_cnt=0;
  if (param<bool>("only_horizontal"))
    pts_per_roofplane[-1] = std::vector<Point>();
  for(auto seg: PD.segment_shapes){
    auto& plane = seg.second;
    Vector n = plane.orthogonal_vector();
    // this dot product is close to 0 for vertical planes
    auto horizontality = CGAL::abs(n*Vector(0,0,1));
    bool is_wall = horizontality < metrics_is_wall_threshold;
    bool is_horizontal = horizontality > metrics_is_horizontal_threshold;

    // put slanted surface points at index -1 if we care only about horzontal surfaces
    if (!is_wall) {
      auto segpts = PD.get_points(seg.first);
      if (!param<bool>("only_horizontal") ||
          (param<bool>("only_horizontal") && is_horizontal)) {
        pts_per_roofplane[seg.first] = segpts;
      } else if (!is_horizontal) {
        pts_per_roofplane[-1].insert(
          pts_per_roofplane[-1].end(),
          segpts.begin(),
          segpts.end()
        );
      }
    }
    if (is_horizontal)
      ++horiz_roofplane_cnt;
    else if (!is_wall && !is_horizontal)
      ++slant_roofplane_cnt;

    auto plane_idx = PD.get_point_indices(seg.first);
    for (size_t& i : plane_idx) {
      boost::get<2>(pnl_points[i]) = seg.first;
      boost::get<3>(pnl_points[i]) = is_wall;
      boost::get<9>(pnl_points[i]) = is_horizontal;
    }
  }

  int building_type=-2; // as built: -2=undefined; -1=no pts; 0=LOD1, 1=LOD1.3, 2=LOD2
  if (horiz_roofplane_cnt==1 && slant_roofplane_cnt==0)
    building_type=0;
  else if (horiz_roofplane_cnt!=0 && slant_roofplane_cnt==0)
    building_type=1;
  else if (slant_roofplane_cnt!=0)
    building_type=2;
  else if (PD.segment_shapes.size()==0)
    building_type=-1;

  output("class").set(building_type);
  output("classf").set(float(building_type));
  output("horiz_roofplane_cnt").set(float(horiz_roofplane_cnt));
  output("slant_roofplane_cnt").set(float(slant_roofplane_cnt));

  vec1i plane_id, is_wall, is_horizontal;
  for(auto& p : pnl_points) {
    plane_id.push_back(boost::get<2>(p));
    is_wall.push_back(boost::get<3>(p));
    is_horizontal.push_back(boost::get<9>(p));
  }
  output("pts_per_roofplane").set(pts_per_roofplane);
  output("plane_id").set(plane_id);
  output("is_wall").set(is_wall);
  output("is_horizontal").set(is_horizontal);
}

void ComputeMetricsNode::process() {
  auto points = input("points").get<PointCollection>();

  config c;
  c.metrics_normal_k = param<int>("metrics_normal_k");
  c.metrics_plane_min_points = param<int>("metrics_plane_min_points");
  c.metrics_plane_epsilon = param<float>("metrics_plane_epsilon");
  c.metrics_plane_normal_threshold = param<float>("metrics_plane_normal_threshold");
  c.metrics_is_wall_threshold = param<float>("metrics_is_wall_threshold");
  c.metrics_is_horizontal_threshold = param<float>("metrics_is_horizontal_threshold");
  c.metrics_k_linefit = param<int>("metrics_k_linefit");
  c.metrics_k_jumpcnt_elediff = param<int>("metrics_k_jumpcnt_elediff");

  PNL_vector pnl_points;
  for (auto& p : points) {
    PNL pv;
    boost::get<0>(pv) = Point(p[0], p[1], p[2]);
    pnl_points.push_back(pv);
  }
  compute_metrics(pnl_points, c);
  vec1f line_dist, jump_count, jump_ele;
  vec1i plane_id, is_wall, is_horizontal;
  PointCollection points_c;
  for(auto& p : pnl_points){
    plane_id.push_back(boost::get<2>(p));
    is_wall.push_back(boost::get<3>(p));
    is_horizontal.push_back(boost::get<9>(p));
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
  output("points").set(pnl_points);
  output("points_c").set(points_c);
  output("plane_id").set(plane_id);
  output("is_wall").set(is_wall);
  output("is_horizontal").set(is_horizontal);
  output("line_dist").set(line_dist);
  output("jump_count").set(jump_count);
  output("jump_ele").set(jump_ele);
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
  GridSetup(&pgon[0], pgon.size(), Grid_Resolution, grid_set);
  for (int i = 0; i < size; i++) {
    delete pgon[i];
  }
  return grid_set;
}

void LASInPolygonsNode::process() {
  auto polygons = input("polygons").get<LinearRingCollection>();
  std::vector<PointCollection> point_clouds(polygons.size());

  std::vector<pGridSet> poly_grids;
          
  for (auto& ring : polygons) {
    poly_grids.push_back(build_grid(ring));
  }

  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(param<std::string>("las_filepath").c_str());
  LASreader* lasreader = lasreadopener.open();

  while (lasreader->read_point()) {
    if (lasreader->point.get_classification() == 6) {
      int i=0;
      pPipoint point = new Pipoint{ lasreader->point.get_x()-(*manager.data_offset)[0], lasreader->point.get_y()-(*manager.data_offset)[1] };
      
      for (auto& poly_grid:poly_grids) {
        if (GridTest(poly_grid, point)) {
          point_clouds[i].push_back({
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

  output("point_clouds").set(point_clouds);
}

void BuildingSelectorNode::process() {
  auto& point_clouds = input("point_clouds").get<std::vector<PointCollection>&>();
  auto& polygons = input("polygons").get<LinearRingCollection&>();
  polygon_count = polygons.size();
  output("point_cloud").set(point_clouds[building_id]);
  output("polygon").set(polygons[building_id]);
};

void RegulariseLinesNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto edges = input("edge_segments").get<SegmentCollection>();
  auto footprint = input("footprint").get<LinearRing>();

  auto dist_threshold = param<float>("dist_threshold");
  auto angle_threshold = param<float>("angle_threshold");

  typedef CGAL::Exact_predicates_inexact_constructions_kernel::Point_2 Point_2;
  typedef CGAL::Exact_predicates_inexact_constructions_kernel::Point_3 Point_3;
  typedef std::array<float,2> arr3f;
  std::vector<std::pair<Segment, bool>> input_edges;

  // build vector of all input edges
  for(auto edge : edges) {
    input_edges.push_back(std::make_pair(Segment({edge[0], edge[1]}), false));
  }
  for(size_t i=0; i<footprint.size()-1; ++i) {
    input_edges.push_back(std::make_pair(Segment({footprint[i], footprint[i+1]}), true));
  }
  input_edges.push_back(std::make_pair(Segment({footprint[footprint.size()-1], footprint[0]}), true));

  //compute attributes for each segment
  typedef std::tuple<double, Point_2, double, double, bool, double, double, size_t> linetype; 
  // new angle, midpoint, distance in angle cluster, elevation, is_footprint, initial angle, squared distance from midpoint to an end point, id_cntr
  std::vector<linetype> lines;
  // add non-footprint lines
  size_t id_cntr = 0;
  for(auto& ie : input_edges) {
    auto& is_footprint = ie.second;
    auto& edge = ie.first;
    auto source = Point_3(edge[0][0], edge[0][1], edge[0][2]);
    auto target = Point_3(edge[1][0], edge[1][1], edge[1][2]);
    auto v = target-source;
    auto p_ = source + v/2;
    auto p = Point_2(p_.x(),p_.y());
    auto l = std::sqrt(v.squared_length()/2);
    auto angle = std::atan2(v.x(),v.y());
    if (angle < 0) angle += pi;
    lines.push_back(std::make_tuple(angle,p,0,p_.z(), is_footprint, angle, l, id_cntr++));
  }

  //sort by angle, smallest on top
  std::vector<size_t> edge_idx(input_edges.size());
  for (size_t i=0; i<input_edges.size(); ++i) {
    edge_idx[i]=i;
  }
  std::sort(edge_idx.begin(), edge_idx.end(), [&lines=lines](size_t a, size_t b) {
    return std::get<0>(lines[a]) < std::get<0>(lines[b]);   
  });
  //cluster by angle difference
  std::vector<ValueCluster> angle_clusters(1);
  auto last_angle = std::get<0>(lines[edge_idx[0]]);
  for(auto edge_id : edge_idx ) {
    auto& line = lines[edge_id];
    if((std::get<0>(line) - last_angle) < angle_threshold)
      angle_clusters.back().idx.push_back(edge_id);
    else {
      angle_clusters.resize(angle_clusters.size()+1);
      angle_clusters.back().idx.push_back(edge_id);
      }
    last_angle=std::get<0>(line);
  }

  // get average angle for each cluster
  // vec3f directions_before, directions_after;
  // vec1i angles;
  for(auto& cluster : angle_clusters) {
    // average angle:
    double sum=0;
    for(auto& i : cluster.idx) {
      sum+=std::get<0>(lines[i]);
    }
    double angle = sum/cluster.idx.size();
    Vector_2 n(-1.0, std::tan(angle));
    cluster.ref_vec = n/std::sqrt(n.squared_length()); // normalize
    
    // or median angle:
    // size_t median_id = cluster.idx[cluster.idx.size()/2];
    // cluster.value = std::get<0>(lines[median_id]);
  }

  // vec1f distances;
  // snap nearby lines that are close
  std::vector<ValueCluster> dist_clusters;
  for(auto& cluster : angle_clusters) {
    auto n = cluster.ref_vec;
    // compute distances along n wrt to first line in cluster
    auto p = std::get<1>(lines[cluster.idx[0]]);
    for(auto& i : cluster.idx) {
      auto q = std::get<1>(lines[i]);
      auto v = p-q;
      std::get<2>(lines[i]) = v*n;
      // distances.push_back(v*n);
    }
    // sort by distance, ascending
    auto sorted_by_dist = cluster.idx;
    std::sort(sorted_by_dist.begin(), sorted_by_dist.end(), [&lines=lines](size_t a, size_t b){
        return std::get<2>(lines[a]) < std::get<2>(lines[b]);
    });
    // cluster nearby lines using separation threshold
    double last_dist = std::get<2>(lines[sorted_by_dist[0]]);
    dist_clusters.resize(dist_clusters.size()+1);
    dist_clusters.back().ref_vec = n;
    for(auto& i : sorted_by_dist) {
      auto& line = lines[i];
      double dist_diff = std::get<2>(line) - last_dist;
      if (dist_diff < dist_threshold) {
        dist_clusters.back().idx.push_back(i);
      } else {
        dist_clusters.resize(dist_clusters.size()+1);
        dist_clusters.back().ref_vec = n;
        dist_clusters.back().idx.push_back(i);
      }
      last_dist = std::get<2>(line);
    }
  }

  // compute one line per dist cluster => the one with the highest elevation
  LineStringCollection edges_out;
  // std::vector<std::pair<Point,Point>> edges_out;
  
  for(auto& cluster : dist_clusters) {
    // find average midpoint
    // double sum=0;
    Vector_2 sum_p(0,0);
    for(auto& i : cluster.idx) {
      // sum+=std::get<2>(lines[i]);
      auto& q = std::get<1>(lines[i]);
      sum_p += Vector_2(q.x(), q.y());
    }
    // cluster.distance = sum/cluster.idx.size();
    cluster.ref_point = sum_p/cluster.idx.size();
    cluster.ref_vec = Vector_2(cluster.ref_vec.y(), -cluster.ref_vec.x());
    
    
    //try to find a footprint line
    linetype best_line;
    double best_angle;
    bool found_fp=false, found_non_fp=false;
    for(auto& i : cluster.idx) {
      auto& line = lines[i];
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
    if(!found_fp) {
      double max_z=0;
      linetype high_line;
      for(auto& i : cluster.idx) {
      auto& line = lines[i];
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
    auto halfdist = std::get<6>(best_line);
    // Vector_2 n(-1.0, std::tan(angle));
    // n = n/std::sqrt(n.squared_length();()); // normalize
    Vector_2 l(std::tan(best_angle),1.0);
    l = l/std::sqrt(l.squared_length()); // normalize
    auto p_center = p0;// + average_dist*n;
    auto p_begin = p_center + halfdist*l;
    auto p_end = p_center - halfdist*l;
    edges_out.push_back({
      {float(p_begin.x()), float(p_begin.y()), 0},
      {float(p_end.x()), float(p_end.y()), 0}
    });
  }

  // std::vector<LineCluster> line_clusters;
  LineStringCollection merged_edges_out;
  vec1i cluster_labels;
  int i=0;
  for(auto& cluster : dist_clusters) {
    LineCluster final_cluster;
    auto ref_v = cluster.ref_vec;
    auto ref_p = Point_2(cluster.ref_point.x(), cluster.ref_point.y());

    // IntervalList interval_list;
    for(auto& i : cluster.idx) {
      bool& is_footprint = std::get<4>(lines[i]);
      if (!is_footprint) {
        auto& edge = input_edges[i].first;
        auto s = Point_2(edge[0][0], edge[0][1]);
        auto t = Point_2(edge[1][0], edge[1][1]);
        auto d1 = (s-ref_p)*ref_v;
        auto d2 = (t-ref_p)*ref_v;
        // interval_list.insert({d1,d2});
        auto source = ref_p + d1*ref_v;
        auto target = ref_p + d2*ref_v;
        merged_edges_out.push_back({
          {float(source.x()), float(source.y()), 0},
          {float(target.x()), float(target.y()), 0}
        });
        cluster_labels.push_back(i);
      }
    }
    // merge non-footprint segments that have an overlap
    // std::cout << "size of intervallist: " << interval_list.size()  << "\n";
    // auto merged_intervals = interval_list.merge_overlap();

    // for (auto& s : segments_in_cluster) {
      
    // }
    ++i;
    // clip non footprint segments on overlapping footprint segments?
    // output all resulting segments as a LineCluster?
  }

  output("merged_edges_out").set(merged_edges_out);
  output("edges_out").set(edges_out);
}

void chain(Segment& a, Segment& b, LinearRing& ring, const float& snap_threshold) {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  K::Line_2 l_a(K::Point_2(a[0][0], a[0][1]), K::Point_2(a[1][0], a[1][1]));
  K::Line_2 l_b(K::Point_2(b[0][0], b[0][1]), K::Point_2(b[1][0], b[1][1]));
  K::Segment_2 s(K::Point_2(a[1][0], a[1][1]), K::Point_2(b[0][0], b[0][1]));
  auto result = CGAL::intersection(l_a, l_b);
  if (result) {
    if (auto p = boost::get<K::Point_2>(&*result)) {
      if (CGAL::squared_distance(*p, s) < snap_threshold) {
        ring.push_back({float(p->x()), float(p->y()), 0});
      } else {
        ring.push_back(a[1]);
        ring.push_back(b[0]);
      }
    }
  // } else if (auto l = boost::get<K::Line_2>(&*result)) {

  // }
  } else { // there is no intersection
    ring.push_back(a[1]);
    ring.push_back(b[0]);
  }
}
void RegulariseRingsNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto edges = input("edge_segments").get<SegmentCollection>();
  auto ring_idx = input("ring_idx").get<std::vector<std::vector<size_t>>>();
  auto footprint = input("footprint").get<LinearRing>();
  // auto ring_id = input("ring_id").get<vec1i>();
  // auto ring_order = input("ring_order").get<vec1i>();

  SegmentCollection all_edges;

  // build vector of all input edges
  // for(auto edge : edges) {
  //   all_edges.push_back(edge);
  // }
  // size_t fpi_begin = all_edges.size();
  SegmentCollection fp_edges;
  for(size_t i=0; i<footprint.size()-1; ++i) {
    fp_edges.push_back(Segment({footprint[i], footprint[i+1]}));
  }
  fp_edges.push_back(
    Segment({footprint[footprint.size()-1], footprint[0]})
  );
  // size_t fpi_end = all_edges.size()-1;

  // get clusters from line regularisation 
  auto LR = linereg::LineRegulariser();
  LR.add_segments(0,edges);
  LR.add_segments(1,fp_edges);
  LR.dist_threshold = param<float>("dist_threshold");
  LR.angle_threshold = param<float>("angle_threshold");
  LR.cluster(param<bool>("weighted_avg"), param<bool>("angle_per_distcluster"));

  std::vector<linereg::Polygon_2> exact_polygons;
  for (auto& idx : ring_idx) {
    exact_polygons.push_back(
      linereg::chain_ring<linereg::EK>(idx, LR.get_segments(0), param<float>("snap_threshold"))
    );
  }
  std::vector<size_t> fp_idx;
  for (size_t i=0; i < LR.get_segments(1).size(); ++i) {
    fp_idx.push_back(i);
  }
  auto exact_fp = linereg::chain_ring<linereg::EK>(fp_idx, LR.get_segments(1), param<float>("snap_threshold"));
  output("exact_rings_out").set(exact_polygons);
  output("exact_footprint_out").set(exact_fp);

  LinearRingCollection lrc;
  for (auto& poly : exact_polygons) {
    LinearRing lr;
    for (auto p=poly.vertices_begin(); p!=poly.vertices_end(); ++p) {
      lr.push_back({
        float(CGAL::to_double(p->x())),
        float(CGAL::to_double(p->y())),
        0
      });
    }
    lrc.push_back(lr);
  }
  output("rings_out").set(lrc);

  SegmentCollection new_segments;
  vec1i priorities;
  for(auto& kv : LR.segments) {
    for(auto& ekseg : kv.second) {
      new_segments.push_back(Segment());
      new_segments.back()[0] = {float(CGAL::to_double(ekseg.source().x())), float(CGAL::to_double(ekseg.source().y())), 0};
      new_segments.back()[1] = {float(CGAL::to_double(ekseg.target().x())), float(CGAL::to_double(ekseg.target().y())), 0};
      priorities.push_back(kv.first);
      priorities.push_back(kv.first);
    }
  }


  output("priorities").set(priorities);
  output("edges_out").set(new_segments);
  // output("rings_out").set(new_rings);
  // output("footprint_out").set(new_fp);
}

LinearRing simplify_footprint(LinearRing& polygon, float& threshold_stop_cost) {
  namespace PS = CGAL::Polyline_simplification_2;
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 Point_2;
  typedef CGAL::Polygon_2<K>                   Polygon_2;
  typedef PS::Stop_below_count_ratio_threshold Stop_count_ratio;
  typedef PS::Stop_above_cost_threshold        Stop_cost;
  typedef PS::Squared_distance_cost            Cost;

  if (polygon.size()>2) {
      Polygon_2 cgal_polygon;
      Cost cost;

      for (auto& p : polygon) {
        cgal_polygon.push_back(Point_2(p[0], p[1]));
      }
      // cgal_polygon.erase(cgal_polygon.vertices_end()-1); // remove repeated point from the boost polygon
      
      // polygon = PS::simplify(polygon, cost, Stop_count_ratio(0.5));

      cgal_polygon = PS::simplify(cgal_polygon, cost, Stop_cost(threshold_stop_cost));
      
      LinearRing footprint_vec3f;
      for (auto v = cgal_polygon.vertices_begin(); v!=cgal_polygon.vertices_end(); v++){
        footprint_vec3f.push_back({float(v->x()),float(v->y()),0});
      }

      // HACK: CGAL does not seem to remove the first point of the input polygon in any case, so we need to check ourselves
      auto p_0 = *(cgal_polygon.vertices_begin());
      auto p_1 = *(cgal_polygon.vertices_begin()+1);
      auto p_end = *(cgal_polygon.vertices_end()-1);
      // check the distance between the first vertex and the line between its 2 neighbours
      if (CGAL::squared_distance(Point_2(p_0), K::Segment_2(p_end, p_1)) < threshold_stop_cost) {
        footprint_vec3f.erase(footprint_vec3f.begin());
      }

      return footprint_vec3f;
    } else 
      return polygon;
}

void SimplifyPolygonNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers

  auto geom_term = input("polygons");

  auto threshold_stop_cost = param<float>("threshold_stop_cost");

  if (geom_term.connected_type==TT_linear_ring) {
    auto& polygon = geom_term.get<LinearRing&>();
    output("polygon_simp").set(
      simplify_footprint(polygon, threshold_stop_cost)
    );
  } else if (geom_term.connected_type==TT_linear_ring_collection) {
    auto& polygons = geom_term.get<LinearRingCollection&>();
    LinearRingCollection polygons_out;
    for (auto& polygon : polygons) {
      polygons_out.push_back(
        simplify_footprint(polygon, threshold_stop_cost)
      );
    }
    output("polygons_simp").set(polygons_out);
  }
  
  
}


// void PlaneDetectorNode::process() {
//   auto points = input("point_clouds").get<Feature>();

//   planedect::PlaneDetector PD(points_vec, normals_vec);
//   PD.dist_thres = c.metrics_plane_epsilon * c.metrics_plane_epsilon;
//   PD.normal_thres = c.metrics_plane_normal_threshold;
//   PD.min_segment_count = c.metrics_plane_min_points;
//   PD.N = c.metrics_normal_k;
//   PD.detect();
//   std::cout << PD.segment_shapes.size() << " shapes detected." << std::endl;

//   // // Instantiates shape detection engine.
//   // Region_growing shape_detection;

//   // // Sets parameters for shape detection.
//   // Region_growing::Parameters parameters;
//   // // Sets probability to miss the largest primitive at each iteration.
//   // // parameters.probability = 0.05;
//   // // Detect shapes with at least 500 points.
//   // parameters.min_points = c.metrics_plane_min_points;
//   // // Sets maximum Euclidean distance between a point and a shape.
//   // parameters.epsilon = c.metrics_plane_epsilon;
//   // // Sets maximum Euclidean distance between points to be clustered.
//   // // parameters.cluster_epsilon = 0.01;
//   // // Sets maximum normal deviation.
//   // // 0.9 < dot(surface_normal, point_normal); 
//   // parameters.normal_threshold = c.metrics_plane_normal_threshold;

//   // // Provides the input data.
//   // shape_detection.set_input(points);
//   // // Registers planar shapes via template method.
//   // shape_detection.add_shape_factory<SCPlane>();
//   // // Detects registered shapes with parameters.
//   // std::cout << "points.size: " << points.size() << "\n";
//   // shape_detection.detect(parameters);
//   // // Prints number of detected shapes.
//   // std::cout << shape_detection.shapes().end() - shape_detection.shapes().begin() << " shapes detected." << std::endl;

//   i=1;
//   for(auto seg: PD.segment_shapes){
//     auto& plane = seg.second;
//     auto plane_idx = PD.get_point_indices(seg.first);
//   }

//   output("decomposed_footprints").set(all_cells);
//   output("attributes").set(all_attributes);
// }

}
