#include "imgui.h"
#include "geoflow.hpp"

#include <lasreader.hpp>

using namespace geoflow;

class LASLoaderNode:public Node {
  public:
  char filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
  int thin_nth = 20;
  bool use_thin = true;

  LASLoaderNode(NodeManager& manager):Node(manager) {
    add_output("points", TT_point_collection);
    add_output("classification", TT_vec1i);
    add_output("intensity", TT_vec1f);
  }
  void gui(){
    ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 1, 100);
  }
  void process();
};