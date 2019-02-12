#pragma once

#include <unordered_map>

#include <CGAL/property_map.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Search_traits_adapter.h>

#include <geoflow/core/geoflow.hpp>

namespace linedect {
  typedef CGAL::Exact_predicates_inexact_constructions_kernel cgal_kernel;
  typedef cgal_kernel::Vector_3 Vector;
  typedef cgal_kernel::Point_3 Point;
  typedef cgal_kernel::Line_3 Line;

  using namespace std;

  typedef vector<vector<size_t>> NeighbourVec;

  class LineDetector {
    
    typedef std::pair<Point,size_t> point_index;
    typedef CGAL::Search_traits_3<cgal_kernel>                       Traits_base;
    typedef CGAL::Search_traits_adapter<point_index,
    CGAL::First_of_pair_property_map<point_index>,
    Traits_base>                                              TreeTraits;
    typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
    typedef Neighbor_search::Tree Tree;

    vector<point_index> indexed_points;
    // Tree tree;
    // vector<bool> point_seed_flags;
    NeighbourVec neighbours;
    size_t region_counter=1;
    
    public:
    vector<size_t> point_segment_idx; // 0=unsegmented, maybe put this on the heap...
    unordered_map<size_t, Line> segment_shapes;
    int N = 5;
    double dist_thres = 0.2*0.2;
    size_t min_segment_count = 20;

    LineDetector(vector<Point> &points);
    LineDetector(vector<Point> &points, vector<vector<size_t>> neighbours);
    vector<size_t> get_point_indices(size_t shape_id);
    size_t get_bounded_edges(geoflow::SegmentCollection& edges);
    void detect();

    private:
    inline Line fit_line(vector<size_t>& neighbour_idx);
    inline bool valid_candidate(Line &line, Point &p);
    void grow_region(size_t seed_idx);
  };
}