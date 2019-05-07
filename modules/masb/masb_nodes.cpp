#include "masb_nodes.hpp"
#include "region_grower.hpp"
#include "region_grower_mat.hpp"

#include <cmath>
#include <algorithm>

namespace geoflow::nodes::mat {

void ComputeMedialAxisNode::process(){
  auto point_collection = input("points").get<PointCollection>();
  auto normals_vec3f = input("normals").get<vec3f>();

  // prepare data structures and transfer data
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

  // compute mat points
  masb::compute_masb_points(params, madata);

  // retrieve mat points
  vec1i ma_qidx;
  ma_qidx.reserve(madata.m*2);
  for(size_t i=0 ; i<madata.m*2; ++i) {
    ma_qidx.push_back(madata.ma_qidx[i]);
  }

  PointCollection ma_coords;
  ma_coords.reserve(madata.m*2);
  for(auto& c : ma_coords_) {
    ma_coords.push_back({c[0], c[1], c[2]});
  }

  // Compute medial geometry
  vec1f ma_radii(madata.m*2);
  vec1f ma_sepangle(madata.m*2);
  vec3f ma_spoke_f1(madata.m*2);
  vec3f ma_spoke_f2(madata.m*2);
  vec3f ma_bisector(madata.m*2);
  vec3f ma_spokecross(madata.m*2);
  for(size_t i=0; i<madata.m*2; ++i) {
    auto i_ = i%madata.m;
    auto& c = ma_coords_[i];
    // feature points
    auto& f1 = coords[i_];
    auto& f2 = coords[ma_qidx[i]];
    // radius
    ma_radii[i] = Vrui::Geometry::dist(f1, c);
    // spoke vectors
    auto s1 = f1-c;
    auto s2 = f2-c;
    ma_spoke_f1[i] = {s1[0], s1[1], s1[2]};
    ma_spoke_f2[i] = {s2[0], s2[1], s2[2]};
    // bisector
    s1.normalize();
    s2.normalize();
    auto b = (s1+s2).normalize();
    ma_bisector[i] = {b[0], b[1], b[2]};
    // separation angle
    ma_sepangle[i] = std::acos(s1*s2);
    // cross product of spoke vectors
    auto scross = Vrui::Geometry::cross(s1,s2).normalize();
    ma_spokecross[i] = {scross[0], scross[1], scross[2]};
  }
  vec1i ma_is_interior(madata.m*2, 0);
  std::fill_n(ma_is_interior.begin(), madata.m, 1);

  output("ma_coords").set(ma_coords);
  output("ma_qidx").set(ma_qidx);
  output("ma_radii").set(ma_radii);
  output("ma_is_interior").set(ma_is_interior);
  output("ma_sepangle").set(ma_sepangle);
  output("ma_bisector").set(ma_bisector);
  output("ma_spoke_f1").set(ma_spoke_f1);
  output("ma_spoke_f2").set(ma_spoke_f2);
  output("ma_spokecross").set(ma_spokecross);
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


void SegmentMakerNode::process(){
  auto sources = input("sources").get<PointCollection>();
  auto directions = input("directions").get<vec3f>();

  if (sources.size()!=directions.size()) {
    return;
  }

  SegmentCollection segments;
  for(size_t i=0; i<sources.size(); ++i) {
    arr3f target = {sources[i][0] + directions[i][0], sources[i][1] + directions[i][1], sources[i][2] + directions[i][2]};
    segments.push_back({sources[i], target});
  }
  output("segments").set(segments);
}

void SegmentMedialAxisNode::process() {
  auto ma_coords = input("ma_coords").get<PointCollection>();
  auto ma_bisector = input("ma_bisector").get<vec3f>();
  auto ma_sepangle = input("ma_sepangle").get<vec1f>();

  regiongrower::RegionGrower<MaData,Region> R;
  R.min_segment_count = param<int>("min_count");

  MaData D(ma_coords, ma_bisector, ma_sepangle, param<int>("k"));

  switch (param<int>("method"))
  {
  case 0:{
    AngleOfVectorsTester T_bisector_angle(param<float>("bisector_angle"));
    R.grow_regions(D, T_bisector_angle);
    break;}
  case 1:{
    DiffOfAnglesTester T_separation_angle(param<float>("separation_angle"));
    R.grow_regions(D, T_separation_angle);
    break;}
  case 2:{
    CountTester T_shape_count(param<int>("shape_count"));
    R.grow_regions(D, T_shape_count);
    break;}
  default:
    break;
  };
  
  vec1i segment_ids;
  for(auto& region_id : R.region_ids) {
    segment_ids.push_back(int(region_id));
  }
  output("segment_ids").set(segment_ids);
}

}