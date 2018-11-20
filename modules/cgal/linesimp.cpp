#include "linesimp.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace linesimp {

//--- Visvalingam-Whyatt line simplification for 3D lines, for 2D version see https://bost.ocks.org/mike/simplify/
// inline double compute_error(point& p, point& q, point& r);
inline double compute_error(PointList::iterator qit) {
  auto pit = qit; --pit;
  auto rit = qit; ++qit;
  auto p = glm::make_vec3(pit->first.data());
  auto q = glm::make_vec3(qit->first.data());
  auto r = glm::make_vec3(rit->first.data());
  // the area of the triangle pqr is half of the magnitude of the cross product of q-p and q-r
  return glm::length(glm::cross(q-p,q-r))/2;
}

std::vector<Point> visvalingam(const std::vector<Point>& line_string, double threshold) {
  Heap heap;

  // compute errors for all points except first and last
  PointList point_list;
  for(auto& p : line_string){
    auto it = point_list.insert(point_list.end(), std::make_pair(p,heap_handle()));
  }
  auto start = std::next(point_list.begin()); // skip first point
  auto end = std::prev(point_list.end()); // the last point in the line_string (we'll stop before)
  for (PointList::iterator it=start; it!=end; ++it) {
    auto e = compute_error(it);
    auto handle = heap.push(point_error(it,e));
    it->second = handle;
  }
  
  // insert points, update errors of affected triangles until threshold error is reached
  while (!heap.empty() && heap.top().error < threshold){
    // get top element (with largest error) from heap
    auto maxelement = heap.top();
    auto max_p = *maxelement.it;

    point_list.erase(maxelement.it);
        
    auto it_before = std::prev(maxelement.it);
    auto it_after = std::next(maxelement.it);

    if(it_before != point_list.begin()) {
      (*it_before->second).error = compute_error(it_before);
    }
    if(std::next(it_after) != point_list.end()) {
      (*it_after->second).error = compute_error(it_after);
    }
    // remove the point we just inserted in the triangulation from the heap
    heap.pop();
  }

  std::vector<Point> simplified_lines;
  for (auto& p : point_list) {
    simplified_lines.push_back(p.first);
  }
  return simplified_lines;
}

}