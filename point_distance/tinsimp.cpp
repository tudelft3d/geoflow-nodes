#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Triangulation_vertex_base_with_id_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>

#include <vector>
#include <unordered_set>
#include <boost/heap/fibonacci_heap.hpp>

// fibonacci heap for greedy insertion code
struct point_error {
  point_error(int i, double e) : index(i), error(e){}
  int index;
  double error;
  
  bool operator<(point_error const & rhs) const
  {
    return error < rhs.error;
  }
};
typedef boost::heap::fibonacci_heap<point_error> Heap;
typedef Heap::handle_type heap_handle;

typedef CGAL::Exact_predicates_inexact_constructions_kernel			K;
typedef CGAL::Projection_traits_xy_3<K>								Gt;
typedef CGAL::Triangulation_vertex_base_with_id_2<Gt>				Vb;
struct FaceInfo2
{
  FaceInfo2() {}
  int nesting_level;
  bool in_domain() {
    return nesting_level % 2 == 1;
  }
  std::vector<heap_handle>* points_inside = nullptr;
  CGAL::Plane_3<K>* plane = nullptr;
};
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, Gt>	Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<Gt, Fbb>		Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>				Tds;
typedef CGAL::Exact_predicates_tag									Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Gt, Tds, Itag>	CDT;
typedef CDT::Point													Point;
typedef CGAL::Polygon_2<Gt>											Polygon_2;

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
void greedy_insert(CDT &T, const std::vector<Point3> &pts, double threshold);


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

void greedy_insert(CDT &T, const std::vector<Point3> &pts, double threshold) {
  // assumes all lidar points are inside a triangle
  Heap heap;

  // Convert all elevation points to CGAL points
  std::vector<Point> cpts;
  cpts.reserve(pts.size());
  for (auto& p : pts) {
    cpts.push_back(Point(bg::get<0>(p), bg::get<1>(p), bg::get<2>(p)));
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