#include <geoflow/core/geoflow.hpp>

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
  public:
    using Node::Node;
    void init() {
      add_input("line_strings", {typeid(LineStringCollection)});
      add_output("line_strings", typeid(LineStringCollection));

      add_param("filter_length", (float)0.0);
    }
    void gui() {
      ImGui::DragFloat("Length filter ", &param<float>("filter_length"), 10.0);
    }
    void process();
  };
}