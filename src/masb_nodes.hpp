#include "geoflow.hpp"
#include "imgui.h"


#include "compute_ma_processing.h"
#include "compute_normals_processing.h"

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
    madata.coords = &coords;

    masb::compute_normals(params, madata);

    vec3f normals;
    normals.reserve(madata.m);
    for(auto& n : *madata.normals) {
      normals.push_back({n[0], n[1], n[2]});
    }

    outputs("points").set(normals);
  }
};