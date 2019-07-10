#include <geoflow/geoflow.hpp>

namespace geoflow::nodes::las {

  class LASLoaderNode:public Node {
    std::string filepath = "";
    int thin_nth=5;
    public:
    using Node::Node;
    void init() {
      add_output("points", typeid(PointCollection));
      add_output("classification", typeid(vec1i));
      add_output("intensity", typeid(vec1f));

      add_param("filepath", ParamPath(filepath, "File path"));
      add_param("thin_nth", ParamBoundedInt(thin_nth, 0, 100, "Thin factor"));
    }
    void process();
  };

  class LASGroundLoaderNode:public Node {
    std::string filepath = "";
    int thin_nth=5;
    public:
    using Node::Node;
    void init() {
      add_output("points", typeid(PointCollection));

      add_param("filepath", ParamPath(filepath, "File path"));
      add_param("thin_nth", ParamBoundedInt(thin_nth, 0, 100, "Thin factor"));
    }
    void process();
  };

  class LASWriterNode:public Node {
    std::string filepath = "";
    public:
    using Node::Node;
    void init() {
      add_input("point_clouds", {typeid(PointCollection), typeid(std::vector<PointCollection>)});
      add_output("classification", typeid(vec1i));
      add_output("intensity", typeid(vec1f));
      
      add_param("filepath", ParamPath(filepath, "File path"));
    }
    void write_point_cloud_collection(PointCollection& point_cloud, std::string path);
    void process();
  };

}