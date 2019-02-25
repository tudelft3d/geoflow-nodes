#include <geoflow/core/geoflow.hpp>

namespace geoflow::nodes::las {

  class LASLoaderNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_output("points", TT_point_collection);
      add_output("classification", TT_vec1i);
      add_output("intensity", TT_vec1f);

      add_param("filepath", (std::string) "");
      add_param("thin_nth", (int)5);
    }
    void gui(){
      ImGui::InputText("LAS file path", &param<std::string>("filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 0, 100);
    }
    void process();
  };

  class LASWriterNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("point_clouds", {TT_point_collection, TT_point_collection_list});
      add_output("classification", TT_vec1i);
      add_output("intensity", TT_vec1f);
      
      add_param("filepath", (std::string) "");
    }
    void gui(){
      ImGui::InputText("LAS file path", &param<std::string>("filepath"));
      ImGui::Checkbox("Write multiple files in case of point cloud list", &param<bool>("multiple_files"));
    }
    void write_point_cloud_collection(PointCollection& point_cloud, std::string path);
    void process();
  };

}