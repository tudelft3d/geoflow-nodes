#pragma once

#include <geoflow/common.hpp>
#include "../region_grower_DS_CGAL.hpp"
#include "../region_grower.hpp"

using namespace std;

class MaData : public CGAL_RegionGrowerDS
  public:
  geoflow::vec3f& ma_bisector;
  geoflow::vec1f& ma_sepangle;
  geoflow::vec1f& ma_radii;
  
  MaData(geoflow::PointCollection& ma_points, geoflow::vec3f& ma_bisector, geoflow::vec1f& ma_sepangle, geoflow::vec1f& ma_radii, size_t N=15) 
    : CGAL_RegionGrowerDS(ma_points, N), ma_bisector(ma_bisector), ma_sepangle(ma_sepangle), ma_radii(ma_radii)
};

class Region : public: regiongrower::Region {
  public:
  using Region::Region;
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