#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Triangulation_vertex_base_with_id_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <vector>
#include <unordered_set>

#include "../point_distance/tinsimp.hpp"

namespace tinsimp {

struct PointXYHash {
  std::size_t operator()(Point const& p) const noexcept {
    std::size_t h1 = std::hash<double>{}(p.x());
    std::size_t h2 = std::hash<double>{}(p.y());
    return h1 ^ (h2 << 1);
  }
};
struct PointXYEqual {
  bool operator()(Point const& p1, Point const& p2) const noexcept {
    auto ex = p1.x() == p2.x();
    auto ey = p1.y() == p2.y();
    return ex && ey;
  }
};

inline double compute_error(Point &p, CDT::Face_handle &face);

//--- TIN Simplification
// Greedy insertion/incremental refinement algorithm adapted from "Fast polygonal approximation of terrain and height fields" by Garland, Michael and Heckbert, Paul S.
inline double compute_error(Point &p, CDT::Face_handle &face) {
  if(!face->info().plane)
    face->info().plane = new CGAL::Plane_3<K>(
      face->vertex(0)->point(),
      face->vertex(1)->point(),
      face->vertex(2)->point());
  if(!face->info().points_inside)
    face->info().points_inside = new std::vector<heap_handle>();

  auto plane = face->info().plane;
  auto interpolate = - plane->a()/plane->c() * p.x() - plane->b()/plane->c()*p.y() - plane->d()/plane->c();
  double error = std::fabs(interpolate - p.z());
  return error;
}

void greedy_insert(CDT &T, const std::vector<std::array<float,3>> &pts, double threshold) {
  // assumes all lidar points are inside a triangle
  Heap heap;

  // Convert all elevation points to CGAL points
  std::vector<Point> cpts;
  cpts.reserve(pts.size());
  for (auto& p : pts) {
    cpts.push_back(Point(p[0], p[1], p[2]));
  }

  // compute initial point errors, build heap, store point indices in triangles
  {
    std::unordered_set<Point, PointXYHash, PointXYEqual> set;
    for(int i=0; i<cpts.size(); i++){
      auto p = cpts[i];
      // detect and skip duplicate points
      auto not_duplicate = set.insert(p).second;
      if(not_duplicate){
        auto face = T.locate(p);
        auto e = compute_error(p, face);
        auto handle = heap.push(point_error(i,e));
        face->info().points_inside->push_back(handle);
      }
    }
  }
  
  // insert points, update errors of affected triangles until threshold error is reached
  while (!heap.empty() && heap.top().error > threshold){
    // get top element (with largest error) from heap
    auto maxelement = heap.top();
    auto max_p = cpts[maxelement.index];

    // get triangles that will change after inserting this max_p
    std::vector<CDT::Face_handle> faces;
    T.get_conflicts ( max_p, std::back_inserter(faces) );

    // insert max_p in triangulation
    auto face_hint = faces[0];
    auto v = T.insert(max_p, face_hint);
    face_hint = v->face();
    
    // update clear info of triangles that just changed, collect points that were inside these triangles
    std::vector<heap_handle> points_to_update;
    for (auto face : faces) {
      if (face->info().plane){
        delete face->info().plane;
        face->info().plane = nullptr;
      }
      if (face->info().points_inside) {
        for (auto h :*face->info().points_inside){
          if( maxelement.index != (*h).index)
            points_to_update.push_back(h);
        }
        face->info().points_inside->clear();
      }
    }
    
    // remove the point we just inserted in the triangulation from the heap
    heap.pop();

    // update the errors of affected elevation points
    for (auto curelement : points_to_update){
      auto p = cpts[(*curelement).index];
      auto containing_face = T.locate(p, face_hint);
      const double e = compute_error(p, containing_face);
      const point_error new_pe = point_error((*curelement).index, e);
      heap.update(curelement, new_pe);
      containing_face->info().points_inside->push_back(curelement);
    }
  }

  //cleanup the stuff I put in face info of triangles
  for (CDT::Finite_faces_iterator fit = T.finite_faces_begin();
    fit != T.finite_faces_end(); ++fit) {
      if (fit->info().plane){
        delete fit->info().plane;
        fit->info().plane = nullptr;
      }
      if (fit->info().points_inside) {
        delete fit->info().points_inside;
        fit->info().points_inside = nullptr;
      }
    }

}

inline double compute_error_lines(Point &p, CDT::Face_handle &face) {
  auto plane = CGAL::Plane_3<K>(
      face->vertex(0)->point(),
      face->vertex(1)->point(),
      face->vertex(2)->point()
  );

  auto interpolate = - plane.a()/plane.c() * p.x() - plane.b()/plane.c()*p.y() - plane.d()/plane.c();
  double error = std::fabs(interpolate - p.z());
  return error;
}
std::vector<size_t> greedy_insert_lines(CDT &T, const std::vector<std::array<float,3>> &pts, const std::vector<size_t> &counts, const double threshold) {
  // assumes all pts points are inside a triangle

  // Convert all elevation points to CGAL points
  std::vector<Point> cpts;
  cpts.reserve(pts.size());
  for (auto& p : pts) {
    cpts.push_back(Point(p[0], p[1], p[2]));
  }

  typedef std::set<std::pair<size_t,size_t>> set_type;
  set_type S; // index, line_index
  typedef std::vector<set_type::iterator> vec_it_type;
  std::vector<vec_it_type> lines_handles;
  // initialise the set
  {
    size_t si = 0, line_index=0;
    for(auto& count : counts){
      std::unordered_set<Point, PointXYHash, PointXYEqual> dupe_set;
      vec_it_type line_handles;
      for(size_t i=0; i<count; i++){
        auto p = cpts[si+i];
        // detect and skip duplicate points
        auto not_duplicate = dupe_set.insert(p).second;
        if(not_duplicate){
          auto ret = S.insert(std::make_pair(si+i,line_index));
          line_handles.push_back(ret.first);
        } else {
          std::cout << "dup detected!\n";
        }
      }
      lines_handles.push_back(line_handles);
      si+=count;
      line_index++;
    }
  }
  
  // insert points, update errors until threshold error is reached
  double max_error = threshold;
  size_t max_index;
  CDT::Face_handle face_hint = T.locate(cpts[0]);
  std::vector<size_t> selected_lines;
  while (!S.empty() && max_error >= threshold){
    // get highest error
    size_t l_id;
    // std::cout << "new iteration greedy insertion, max_error=" << max_error << ", S.size()=" << S.size() <<"\n";
    max_error=0;
    for (auto element : S){
      auto p = cpts[element.first];
      face_hint = T.locate(p,face_hint);
      const double e = compute_error_lines(p, face_hint);
      if (e>max_error){
        max_error = e;
        max_index = element.first;
        l_id = element.second;
      }
    }

    // insert line in triangulation as a constraint, remove corresponding elements from S
    std::vector<Point> linestrip;
    for (auto& handle : lines_handles[l_id]) {
      S.erase(handle);
      linestrip.push_back(cpts[handle->first]);
    }
    T.insert_constraint(linestrip.begin(), linestrip.end());
    selected_lines.push_back(l_id);
    
  }
  return selected_lines;
}
}