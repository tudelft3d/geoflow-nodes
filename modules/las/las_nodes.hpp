#include <geoflow/core/geoflow.hpp>

namespace geoflow::nodes::las {

  class LASLoaderNode:public Node {
    public:
    char filepath[256] = "";
    int thin_nth = 20;
    bool use_thin = true;
    
    using Node::Node;
    void init() {
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
    
    using Node::Node;
    void init() {
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

}