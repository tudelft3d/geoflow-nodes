#include "geoflow.hpp"
#include "imgui.h"

#include "compute_ma_processing.h"
#include "compute_normals_processing.h"

#include <algorithm>

class ComputeMedialAxisNode:public Node {
  public:
  masb::ma_parameters params;
  float interval = 2;
  double zero=0,pi=3.14;

  ComputeMedialAxisNode(NodeManager& manager):Node(manager) {
    add_input("points", TT_point_collection);
    add_input("normals", TT_vec3f);
    add_output("ma_coords", TT_point_collection);
    add_output("ma_qidx", TT_vec1ui);
    add_output("ma_is_interior", TT_vec1b);
  }

  void gui(){
    ImGui::SliderFloat("initial_radius", &params.initial_radius, 0, 1000);
    ImGui::SliderScalar("denoise_preserve", ImGuiDataType_Double, &params.denoise_preserve, &zero, &pi);
    ImGui::SliderScalar("denoise_planar", ImGuiDataType_Double, &params.denoise_planar, &zero, &pi);
    ImGui::Checkbox("nan_for_initr", &params.nan_for_initr);
  }

  void process(){
    auto point_collection = inputs("points").get<PointCollection>();
    auto normals_vec3f = inputs("normals").get<vec3f>();

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

    vec1ui ma_qidx;
    ma_qidx.reserve(madata.m*2);
    for(size_t i=0 ; i<madata.m*2; ++i) {
      ma_qidx.push_back(madata.ma_qidx[i]);
    }

    PointCollection ma_coords;
    ma_coords.reserve(madata.m*2);
    for(auto& c : *madata.ma_coords) {
      ma_coords.push_back({c[0], c[1], c[2]});
    }

    vec1b ma_is_interior(madata.m*2, false);
    std::fill_n(ma_is_interior.begin(), madata.m, true);

    outputs("ma_coords").set(ma_coords);
    outputs("ma_qidx").set(ma_qidx);
    outputs("ma_is_interior").set(ma_is_interior);
  }
};

class ComputeNormalsNode:public Node {
  public:
  masb::normals_parameters params;
  float interval = 2;

  ComputeNormalsNode(NodeManager& manager):Node(manager) {
    add_input("points", TT_point_collection);
    add_output("normals", TT_vec3f);
  }

  void gui(){
    ImGui::SliderInt("K", &params.k, 1, 100);
  }

  void process(){
    auto point_collection = inputs("points").get<PointCollection>();

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

    outputs("normals").set(normals_vec3f);
  }
};