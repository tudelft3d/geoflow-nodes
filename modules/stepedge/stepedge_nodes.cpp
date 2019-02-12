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

#include <numeric>

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

  auto points = input("points").get<PNL_vector>();
  auto thres_alpha = param<float>("thres_alpha");
  auto extract_alpha_rings = param<bool>("extract_alpha_rings");
  
  // collect plane points
  std::unordered_map<int, std::vector<Point>> points_per_segment;
  for (auto& p : points) {
    if (boost::get<2>(p)==0) // unsegmented
      continue;
    if (boost::get<3>(p)) // classified as wall
      continue;
    points_per_segment[boost::get<2>(p)].push_back(boost::get<0>(p));
  }
  PointCollection edge_points, boundary_points;
  LineStringCollection alpha_edges;
  LinearRingCollection alpha_rings;
  TriangleCollection alpha_triangles;
  vec1i segment_ids;
  for (auto& it : points_per_segment ) {
    auto points = it.second;
    as::Triangulation_2 T;
    T.insert(points.begin(), points.end());
    as::Alpha_shape_2 A(T,
                as::FT(thres_alpha),
                as::Alpha_shape_2::GENERAL);
    // thres_alpha = *A.find_optimal_alpha(1);
    A.set_alpha(as::FT(thres_alpha));
    // std::vector<std::pair<Vertex_handle, Vertex_handle>> alpha_edges;
    // for (auto it = A.alpha_shape_edges_begin(); it!=A.alpha_shape_edges_end(); it++) {
    //   auto face = (it)->first;
    //   auto i = (it)->second;
    //   alpha_edges.push_back(std::make_pair(face->vertex(T.cw(i)), face->vertex(T.ccw(i))));
    // }

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
      if (extract_alpha_rings) {
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
  }
  
  if (extract_alpha_rings) {
    output("alpha_rings").set(alpha_rings);
  }
  output("alpha_edges").set(alpha_edges);
  output("alpha_triangles").set(alpha_triangles);
  output("segment_ids").set(segment_ids);
  output("edge_points").set(edge_points);
  output("boundary_points").set(boundary_points);
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
    if(face->data().is_finite) {
      vec2f polygon, face_triangles;
      arrangementface_to_polygon(face, polygon);
      vec3f polygon3d;
      for (auto& p : polygon) {
        polygon3d.push_back({p[0],p[1],0});
      }
      linear_rings.push_back(polygon3d);
      attributes["height"].push_back(face->data().elevation_avg);
      attributes["rms_error"].push_back(face->data().rms_error_to_avg);
      attributes["max_error"].push_back(face->data().max_error);
      attributes["coverage"].push_back(face->data().segid_coverage);
      attributes["count"].push_back(face->data().total_count);
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
  vec1i cell_id_vec1i;
  vec1i labels;
  vec1f rms_errors, max_errors, segment_coverages;
  using N = uint32_t;

  
  size_t cell_id=0;
  float rms_error, max_error, segment_coverage;
  for (auto face: arr.face_handles()){
    if(face->data().is_finite) {
      cell_id++;
      rms_error = face->data().rms_error_to_avg;
      max_error = face->data().max_error;
      segment_coverage = face->data().segid_coverage;
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
          rms_errors.push_back(rms_error);
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
            rms_errors.push_back(rms_error);
            rms_errors.push_back(rms_error);
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
        cell_id_vec1i.push_back(0);
        cell_id_vec1i.push_back(0);
        rms_errors.push_back(-1);
        rms_errors.push_back(-1);
        rms_errors.push_back(-1);
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
        cell_id_vec1i.push_back(0);
        cell_id_vec1i.push_back(0);
        rms_errors.push_back(-1);
        rms_errors.push_back(-1);
        rms_errors.push_back(-1);
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
  output("rms_errors").set(rms_errors);
  output("max_errors").set(max_errors);
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

void BuildArrangementNode::process(){
  // Set up vertex data (and buffer(s)) and attribute pointers
  auto footprint = input("footprint").get<LinearRing>();
  auto geom_term = input("edge_segments");

  // bg::model::polygon<point_type> fp;
  // for(auto& p : footprint) {
  //     bg::append(fp.outer(), point_type(p[0], p[1]));
  //   }

  Arrangement_2 arr;
  arr.clear();
  if (geom_term.connected_type == TT_linear_ring_collection) {
    auto rings = geom_term.get<LinearRingCollection>();
    build_arrangement(footprint, rings, arr, remove_unsupported);
  } else if (geom_term.connected_type == TT_line_string_collection) {
    auto edge_segments = geom_term.get<LineStringCollection>();
    build_arrangement(footprint, edge_segments, arr, remove_unsupported);
  }
  LineStringCollection segments;
  for (auto face: arr.face_handles()){
      if(face->data().is_finite){
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
  }
  output("arr_segments").set(segments);
  output("arrangement").set(arr);
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

void DetectLinesNode::process(){
  auto input_geom = input("edge_points");

  SegmentCollection edge_segments;
  vec1i ring_order, ring_id, is_start;
  // std::vector<SegmentCollection> segment_collections;
  std::vector<std::vector<size_t>> ring_idx;
  // fit lines in all input points
  if (input_geom.connected_type == TT_point_collection) {
    std::vector<linedect::Point> cgal_pts;
    auto points = input_geom.get<PointCollection>();
    for( auto& p : points ) {
      cgal_pts.push_back(linedect::Point(p[0], p[1], p[2]));
    }
    linedect::LineDetector LD(cgal_pts);
    LD.dist_thres = c.linedetect_dist_threshold * c.linedetect_dist_threshold;
    LD.min_segment_count = c.linedetect_min_segment_count;
    LD.N = c.linedetect_k;
    LD.detect();
    LD.get_bounded_edges(edge_segments);

  // fit lines per ring
  } else if (input_geom.connected_type == TT_linear_ring_collection) {
    auto rings = input_geom.get<LinearRingCollection>();
    int n = c.linedetect_k;
    ring_idx.resize(rings.size());
    
    size_t ring_cntr=0;
    size_t seg_cntr=0;
    for (auto& ring : rings) {
      
      std::vector<linedect::Point> cgal_pts;
      for( auto& p : ring ) {
        cgal_pts.push_back(linedect::Point(p[0], p[1], p[2]));
      }

      if (use_linear_neighboorhood) {
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
        LD.dist_thres = c.linedetect_dist_threshold * c.linedetect_dist_threshold;
        LD.min_segment_count = c.linedetect_min_segment_count;
        LD.N = n;
        LD.detect();
        LD.get_bounded_edges(edge_segments);
      } else {
        linedect::LineDetector LD(cgal_pts);
        LD.dist_thres = c.linedetect_dist_threshold * c.linedetect_dist_threshold;
        LD.min_segment_count = c.linedetect_min_segment_count;
        LD.N = n;
        LD.detect();
        // size_t n_edges = LD.get_bounded_edges(edge_segments);
        // for (size_t i=0; i<n_edges; ++i) {
        //   ring_order.push_back(i);
        //   ring_id.push_back(ring_cntr);
        //   is_start.push_back(1);
        //   ring_order.push_back(i);
        //   ring_id.push_back(ring_cntr);
        //   is_start.push_back(0);
        // } ++ring_cntr;

        // chain the detected lines, to ensure correct order
        vec1i new_ring_ids;
        bool start_seg = LD.point_segment_idx[0];
        int prev_i=ring.size()-1,
          prev_seg=LD.point_segment_idx[prev_i], 
          cur_seg, 
          i_last_seg = -1;
        bool no_offset=false;
        for( int i=0; i<ring.size(); ++i ) {
          cur_seg = LD.point_segment_idx[i];
          if(cur_seg==prev_seg && cur_seg!=0) { // we are inside a segment

          } else if (cur_seg!=0 && prev_seg==0) { // from unsegmented to segmented
            new_ring_ids.push_back(i); // first of cur
            if(i==0) no_offset=true;
            // new_ring_ids.push_back(i); // end of unsegmented linesegment
          } else if (cur_seg!=0 && prev_seg!=0) { // from one segment to another
            new_ring_ids.push_back(prev_i); // last of prev
            new_ring_ids.push_back(i); // first of cur
          } else if (cur_seg==0 && prev_seg!=0) { //from segment to unsegmented
            new_ring_ids.push_back(prev_i); // last of prev
            // new_ring_ids.push_back(prev_i); // begin of unsegmented linesegment
          } // else: we are inside an unsegmented or segmented zone
          prev_seg = cur_seg;
          prev_i = i;
        }
        int last = new_ring_ids.size()-1;
        int order_cnt=0;
        
        // ring_idx.push_back();
        for(int i = no_offset ? 0:1; i<last; i += 2) {
          // TODO: reproject points on fitted line!!
          auto& p0 = ring[new_ring_ids[i]];
          auto& p1 = ring[new_ring_ids[i+1]];
          edge_segments.push_back({p0,p1});
          ring_idx[ring_cntr].push_back(seg_cntr++);
          // segment_collections.back().push_back({p0,p1});
          ring_order.push_back(order_cnt);
          ring_id.push_back(ring_cntr);
          ring_order.push_back(order_cnt++);
          ring_id.push_back(ring_cntr);
          is_start.push_back(1);
          is_start.push_back(0);
        }
        if(!no_offset) {
          auto& p0 = ring[new_ring_ids[last]];
          auto& p1 = ring[new_ring_ids[0]];
          edge_segments.push_back({p0,p1});
          ring_idx[ring_cntr].push_back(seg_cntr++);
          // segment_collections.back().push_back({p0,p1});
          ring_order.push_back(order_cnt);
          ring_id.push_back(ring_cntr);
          ring_order.push_back(order_cnt++);
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

void ComputeMetricsNode::process() {
  // Set up vertex data (and buffer(s)) and attribute pointers
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
  if (auto p = boost::get<K::Point_2>(&*result)) {
    if (CGAL::squared_distance(*p, s) < snap_threshold) {
      ring.push_back({float(p->x()), float(p->y()), 0});
    } else {
      ring.push_back(a[1]);
      ring.push_back(b[0]);
    }

  } else if (auto l = boost::get<K::Line_2>(&*result)) {

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
  for(auto edge : edges) {
    all_edges.push_back(edge);
  }
  for(size_t i=0; i<footprint.size()-1; ++i) {
    all_edges.push_back(Segment({footprint[i], footprint[i+1]}));
  }
  all_edges.push_back(
    Segment({footprint[footprint.size()-1], footprint[0]})
  );

  // get clusters from line regularisation 
  auto LR = LineRegulariser(all_edges);
  LR.dist_threshold = param<float>("dist_threshold");
  LR.angle_threshold = param<float>("angle_threshold");
  LR.cluster();

  // get clusters from line regularisation 
  // std::vector<std::vector<size_t>> idx_per_ring;
  // size_t cur_rid, prev_rid = ring_id[0];
  // std::vector<size_t> idx;
  // for (size_t i=0; i<edges.size(); ++i) {
  //   cur_rid = ring_id[i];
  //   if (cur_rid == prev_rid) {
  //     idx.push_back(i);
  //   } else {
  //     idx_per_ring.push_back(idx);
  //     idx.clear();
  //     idx.push_back(i);
  //   }
  //   prev_rid = cur_rid;
  // } idx_per_ring.push_back(idx);

  LinearRingCollection new_rings;
  for (auto& idx : ring_idx) {
    LinearRing new_ring;
    for (size_t i=idx[1]; i<idx[0]+idx.size(); ++i) {
      chain(all_edges[i-1], all_edges[i], new_ring, param<float>("snap_threshold"));
    }
    chain(all_edges[idx[idx.size()-1]], all_edges[idx[0]], new_ring, param<float>("snap_threshold"));
    new_rings.push_back(new_ring);
  }

  // output("merged_edges_out").set(merged_edges_out);
  output("edges_out").set(all_edges);
  output("rings_out").set(new_rings);
}

void LOD13GeneratorNode::process(){
  auto point_clouds = input("point_clouds").get<std::vector<PointCollection>>();
  auto polygons = input("polygons").get<LinearRingCollection>();
  
  // for each pair of polygon and point_cloud
    //create nodes and connections
    //run the thing
  if (point_clouds.size()!=polygons.size()) return;

  LinearRingCollection all_cells;
  AttributeMap all_attributes;
  
  for(int i=0; i<point_clouds.size(); i++) {
    auto& points = point_clouds[i];
    auto& polygon = polygons[i];

    NodeRegister R("Nodes");
    R.register_node<ComputeMetricsNode>("ComputeMetrics");
    R.register_node<AlphaShapeNode>("AlphaShape");
    R.register_node<DetectLinesNode>("DetectLines");
    R.register_node<RegulariseLinesNode>("RegulariseLines");
    R.register_node<BuildArrangementNode>("BuildArrangement");
    R.register_node<ProcessArrangementNode>("ProcessArrangement");
    R.register_node<Arr2LinearRingsNode>("Arr2LinearRings");

    NodeManager N = NodeManager();

    auto ComputeMetrics_node = N.create_node(R, "ComputeMetrics");
    auto AlphaShape_node = N.create_node(R, "AlphaShape");
    auto DetectLines_node = N.create_node(R, "DetectLines");
    auto RegulariseLines_node = N.create_node(R, "RegulariseLines");
    auto BuildArrangement_node = N.create_node(R, "BuildArrangement");
    auto ProcessArrangement_node = N.create_node(R, "ProcessArrangement");
    auto Arr2LinearRings_node = N.create_node(R, "Arr2LinearRings");

    ComputeMetrics_node->input("points").set(points);
    BuildArrangement_node->input("footprint").set(polygon);
    RegulariseLines_node->input("footprint").set(polygon);

    connect(ComputeMetrics_node, AlphaShape_node, "points", "points");
    connect(ComputeMetrics_node, ProcessArrangement_node, "points", "points");
    connect(AlphaShape_node, DetectLines_node, "alpha_rings", "edge_points");
    connect(DetectLines_node, RegulariseLines_node, "edge_segments", "edge_segments");
    connect(RegulariseLines_node, BuildArrangement_node, "edges_out", "edge_segments");
    connect(BuildArrangement_node, ProcessArrangement_node, "arrangement", "arrangement");
    connect(ProcessArrangement_node, Arr2LinearRings_node, "arrangement", "arrangement");

    // config and run
    // this should copy all parameters from this LOD13Generator node to the ProcessArrangement node
    ProcessArrangement_node->set_params( dump_params() );
    
    N.run(ComputeMetrics_node);

    // note: the following will crash if the flowchart specified above is stopped halfway for some reason
    auto cells = Arr2LinearRings_node->output("linear_rings").get<LinearRingCollection>();
    auto attributes = Arr2LinearRings_node->output("attributes").get<AttributeMap>();

    for (int i=0; i<cells.size(); i++) {
      // if(polygons_feature.attr["height"][i]!=0) { //FIXME this is a hack!!
      all_cells.push_back(cells[i]);
      all_attributes["height"].push_back(attributes["height"][i]);
      all_attributes["rms_error"].push_back(attributes["rms_error"][i]);
      all_attributes["max_error"].push_back(attributes["max_error"][i]);
      all_attributes["count"].push_back(attributes["count"][i]);
      all_attributes["coverage"].push_back(attributes["coverage"][i]);
      // }
    }
  }
  output("decomposed_footprints").set(all_cells);
  output("attributes").set(all_attributes);
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