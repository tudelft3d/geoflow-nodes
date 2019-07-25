#include <geoflow/geoflow.hpp>

namespace geoflow::nodes::general {
  class MergeGeometriesNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("geometries1", { typeid(PointCollection),
                                typeid(SegmentCollection),
                                typeid(LineStringCollection),
                                typeid(LinearRingCollection), });
      add_input("geometries2", { typeid(PointCollection),
                                typeid(SegmentCollection),
                                typeid(LineStringCollection),
                                typeid(LinearRingCollection) });
      // NOTE: following used to be a TT_any. Not working as intended anymore.
      add_output("geometries", typeid(LineStringCollection));
    }
    void process();
  };

  class MergeLinestringsNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("lines1", { typeid(LineStringCollection) });
      add_input("lines2", { typeid(LineStringCollection) });
      add_output("lines", typeid(LineStringCollection));
    }
    void process();
  };

  class LineStringFilterNode:public Node {
    float filter_length = 0.0;
  public:
    using Node::Node;
    void init() {
      add_input("line_strings", {typeid(LineStringCollection)});
      add_output("line_strings", typeid(LineStringCollection));

      add_param("filter_length", ParamFloat(filter_length, "Filter length"));
    }
    void process();
  };

  class OBJwriterNode:public Node {
    std::string filepath;
  public:
    using Node::Node;
    void init() {
      add_input("triangles", {typeid(TriangleCollection)});

      add_param("filepath", ParamPath(filepath, "File path"));
    }
    void process();
  };
}