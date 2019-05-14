#pragma once

#include <deque>
#include <random>
#include <algorithm>

#include <geoflow/common.hpp>

#include <CGAL/property_map.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Search_traits_adapter.h>

using namespace std;

class MaData {

  typedef CGAL::Exact_predicates_inexact_constructions_kernel cgal_kernel;
  typedef cgal_kernel::Point_3 Point;
  typedef std::pair<Point,size_t> point_index;
  typedef CGAL::Search_traits_3<cgal_kernel>                       Traits_base;
  typedef CGAL::Search_traits_adapter<point_index,
  CGAL::First_of_pair_property_map<point_index>,
  Traits_base>                                              TreeTraits;
  typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
  typedef Neighbor_search::Tree Tree;

  typedef vector<vector<size_t>> vecvecui;

  public:
  geoflow::PointCollection& ma_points;
  geoflow::vec3f& ma_bisector;
  geoflow::vec1f& ma_sepangle;
  geoflow::vec1f& ma_radii;
  vecvecui neighbours;
  size_t size;
  
  MaData(geoflow::PointCollection& ma_points, geoflow::vec3f& ma_bisector, geoflow::vec1f& ma_sepangle, geoflow::vec1f& ma_radii, size_t N=15) 
    : ma_points(ma_points), ma_bisector(ma_bisector), ma_sepangle(ma_sepangle), ma_radii(ma_radii)
  {
    size = ma_points.size();
    
    vector<point_index> indexed_points;
    indexed_points.reserve(size);
    
    size_t i=0;
    for(auto p: ma_points)
      indexed_points.push_back(std::make_pair(Point(p[0], p[1], p[2]),i++));
    Tree tree;
    tree.insert(indexed_points.begin(), indexed_points.end());
    neighbours.resize(size);
    
    for(auto pi : indexed_points){
      auto p = pi.first;
      neighbours[pi.second].reserve(N);
      Neighbor_search search(tree, p, N+1);
      auto gp = glm::make_vec3(ma_points[pi.second].data());
      // skip the first point since it is identical to the query point
      for (auto neighbour = search.begin()+1 ; neighbour < search.end(); ++neighbour) {
        neighbours[pi.second].push_back(neighbour->first.second);
        // std::cout << pi.second << " : " << glm::distance(gp,glm::make_vec3(ma_points[neighbour->first.second].data())) << "\n";
      }
    }
  };
  deque<size_t> get_seeds() {
    deque<size_t> seeds;
    for (size_t i=0; i<size; ++i) {
      seeds.push_back(i);
    }
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(seeds.begin(), seeds.end(), g);
    return seeds;
  }
  vector<size_t> get_neighbours(size_t idx) {
    return neighbours[idx];
  }
};

struct Region {
  size_t count=0;
};

class AngleOfVectorsTester {
  public:
  float threshold;
  AngleOfVectorsTester(float threshold=5) : 
  threshold(glm::cos(threshold*(3.14159265359/180))) {};

  bool is_valid(MaData& cds, size_t candidate, size_t neighbour, Region& shape) {
    auto b1 = glm::make_vec3(cds.ma_bisector[candidate].data());
    auto b2 = glm::make_vec3(cds.ma_bisector[neighbour].data());
    return glm::dot(b1,b2) > threshold;
  }
};
class DiffOfAnglesTester {
  public:
  float threshold;
  DiffOfAnglesTester(float threshold=5) : 
  threshold(threshold*(3.14159265359/180)) {};

  bool is_valid(MaData& cds, size_t candidate, size_t neighbour, Region& shape) {
    auto a1 = cds.ma_sepangle[candidate];
    auto a2 = cds.ma_sepangle[neighbour];
    // auto b1 = glm::make_vec3(cds.ma_points[candidate].data());
    // auto b2 = glm::make_vec3(cds.ma_points[neighbour].data());
    // std::cout << "neighbour " << neighbour << " of candidate " << candidate << ", dist: " << glm::distance(b1,b2) << "\n";
    return glm::abs(a1-a2) < threshold;
  }
};
class BallOverlapTester {
  public:
  float threshold;
  BallOverlapTester(float threshold=1.2) : 
  threshold(threshold) {};

  bool is_valid(MaData& cds, size_t candidate, size_t neighbour, Region& shape) {
    auto r1 = cds.ma_radii[candidate];
    auto r2 = cds.ma_radii[neighbour];
    auto c1 = glm::make_vec3(cds.ma_points[candidate].data());
    auto c2 = glm::make_vec3(cds.ma_points[neighbour].data());
    auto d = glm::distance(c1,c2);
    return (r1+r2)/d > threshold;
  }
};
class CountTester {
  public:
  size_t threshold;
  CountTester(size_t threshold=50) : 
  threshold(threshold) {};

  bool is_valid(MaData& cds, size_t candidate, size_t neighbour, Region& shape) {
    ++shape.count;
    return shape.count < threshold;
  }
};