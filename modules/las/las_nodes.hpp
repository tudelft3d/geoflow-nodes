#include <geoflow/core/geoflow.hpp>
#include <geoflow/gui/osdialog.hpp>

namespace geoflow::nodes::las {

  class LASLoaderNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_output("points", typeid(PointCollection));
      add_output("classification", typeid(vec1i));
      add_output("intensity", typeid(vec1f));

      add_param("filepath", (std::string) "");
      add_param("thin_nth", (int)5);
    }
    void gui(){
      ImGui::FilePicker(OSDIALOG_OPEN, param<std::string>("filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 1, 100);
    }
    void process();
  };

  class LASGroundLoaderNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_output("points", typeid(PointCollection));

      add_param("filepath", (std::string) "");
      add_param("thin_nth", (int)5);
    }
    void gui() {
      ImGui::FilePicker(OSDIALOG_OPEN, param<std::string>("filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 1, 100);
    }
    void process();
  };

  class LASWriterNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("point_clouds", {typeid(PointCollection), typeid(std::vector<PointCollection>)});
      add_output("classification", typeid(vec1i));
      add_output("intensity", typeid(vec1f));
      
      add_param("filepath", (std::string) "");
    }
    void gui(){
      ImGui::FilePicker(OSDIALOG_SAVE, param<std::string>("filepath"));
      ImGui::Checkbox("Write multiple files in case of point cloud list", &param<bool>("multiple_files"));
    }
    void write_point_cloud_collection(PointCollection& point_cloud, std::string path);
    void process();
  };

}