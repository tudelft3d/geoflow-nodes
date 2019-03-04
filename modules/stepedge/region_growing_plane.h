#pragma once

#include <unordered_map>

#include <CGAL/property_map.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Plane_3.h>

namespace planedect {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel cgal_kernel;
  typedef cgal_kernel::Point_3 Point;
  typedef cgal_kernel::Vector_3 Vector;
  typedef cgal_kernel::Plane_3 Plane;

  using namespace std;

  class PlaneDetector {
    
    typedef std::pair<Point,size_t> point_index;
    typedef CGAL::Search_traits_3<cgal_kernel>                       Traits_base;
    typedef CGAL::Search_traits_adapter<point_index,
    CGAL::First_of_pair_property_map<point_index>,
    Traits_base>                                              TreeTraits;
    typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
    typedef Neighbor_search::Tree Tree;

    vector<point_index> indexed_points;
    vector<Vector> normals;
    Tree tree;
    vector<bool> point_seed_flags;
    size_t region_counter=1;
    
    public:
    vector<size_t> point_segment_idx; // 0=unsegmented, maybe put this on the heap...
    unordered_map<size_t, Plane> segment_shapes;
    int N = 5;
    double dist_thres = 0.2*0.2;
    double normal_thres = 0.9;
    size_t min_segment_count = 20;
    size_t n_refit = 5;

    PlaneDetector(vector<Point> &points, vector<Vector> &normals);
    vector<size_t> get_point_indices(size_t shape_id);
    vector<Point> get_points(size_t shape_id);
    void detect();

    private:
    inline Plane fit_plane(Neighbor_search search_result);
    inline bool valid_candidate(Plane &plane, Point &p, Vector &n);
    void grow_region(size_t seed_idx);
  };
}