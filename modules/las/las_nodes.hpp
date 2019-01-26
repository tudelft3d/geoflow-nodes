#include "imgui.h"
#include "geoflow.hpp"

using namespace geoflow;

class LASLoaderNode:public Node {
  public:
  char filepath[256] = "";
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

class LASWriterNode:public Node {
  public:
  char filepath[256] = "out.las";
  bool multiple_files = true;

  LASWriterNode(NodeManager& manager):Node(manager) {
    add_input("point_clouds", {TT_point_collection, TT_point_collection_list});
    // add_output("classification", TT_vec1i);
    // add_output("intensity", TT_vec1f);
  }
  void gui(){
    ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::Checkbox("Write multiple files in case of point cloud list", &multiple_files);
  }
  void write_point_cloud_collection(PointCollection& point_cloud, std::string path);
  void process();
};