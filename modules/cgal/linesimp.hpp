#include <boost/heap/fibonacci_heap.hpp>
#include <list>

#include <geoflow/core/geoflow.hpp>

namespace linesimp {

struct point_error;
typedef std::array<float,3> Point;

typedef boost::heap::fibonacci_heap<point_error> Heap;
typedef Heap::handle_type heap_handle;

typedef std::list<std::pair<Point, heap_handle>> PointList;

struct point_error {
  point_error(PointList::iterator i, double e) : it(i), error(e){}
  PointList::iterator it;
  double error;
  
  bool operator<(point_error const & rhs) const
  {
    return error > rhs.error; //smallest error on top
  }
};


std::vector<Point> visvalingam(const std::vector<Point> &line_string, double threshold);

}