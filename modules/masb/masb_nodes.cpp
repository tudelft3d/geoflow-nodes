#include "masb_nodes.hpp"

#include <algorithm>

namespace geoflow::nodes::mat {

void ComputeMedialAxisNode::process(){
  auto point_collection = input("points").get<PointCollection>();
  auto normals_vec3f = input("normals").get<vec3f>();

  masb::ma_data madata;
  madata.m = point_collection.size();
  
  masb::PointList coords;
  coords.reserve(madata.m);
  for(auto& p : point_collection) {
    coords.push_back(masb::Point(p.data()));
  }
  masb::VectorList normals;
  normals.reserve(madata.m);
  for(auto& n : normals_vec3f) {
    normals.push_back(masb::Vector(n.data()));
  }
  masb::PointList ma_coords_(madata.m*2);
  std::vector<int> ma_qidx_(madata.m*2);
  
  madata.coords = &coords;
  madata.normals = &normals;
  madata.ma_coords = &ma_coords_;
  madata.ma_qidx = ma_qidx_.data();

  masb::compute_masb_points(params, madata);

  vec1i ma_qidx;
  ma_qidx.reserve(madata.m*2);
  for(size_t i=0 ; i<madata.m*2; ++i) {
    ma_qidx.push_back(madata.ma_qidx[i]);
  }

  PointCollection ma_coords;
  ma_coords.reserve(madata.m*2);
  for(auto& c : *madata.ma_coords) {
    ma_coords.push_back({c[0], c[1], c[2]});
  }

  vec1i ma_is_interior(madata.m*2, 0);
  std::fill_n(ma_is_interior.begin(), madata.m, 1);

  output("ma_coords").set(ma_coords);
  output("ma_qidx").set(ma_qidx);
  output("ma_is_interior").set(ma_is_interior);
}


void ComputeNormalsNode::process(){
  auto point_collection = input("points").get<PointCollection>();

  masb::ma_data madata;
  madata.m = point_collection.size();
  masb::PointList coords;
  coords.reserve(madata.m);
  for(auto& p : point_collection) {
    coords.push_back(masb::Point(p.data()));
  }
  masb::VectorList normals(madata.m);
  madata.coords = &coords;
  madata.normals = &normals;

  masb::compute_normals(params, madata);

  vec3f normals_vec3f;
  normals_vec3f.reserve(madata.m);
  for(auto& n : *madata.normals) {
    normals_vec3f.push_back({n[0], n[1], n[2]});
  }

  output("normals").set(normals_vec3f);
}

}