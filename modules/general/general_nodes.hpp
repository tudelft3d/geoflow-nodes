#include <geoflow/core/geoflow.hpp>

const double PI = std::atan(1.0) * 4;

namespace geoflow::nodes::general {
  class Line {
  public:
    int id;
    double a, b;
    double length;
    arr3f p1;
    arr3f p2;
  };

  class Buffer {
  public:
    int id;
    double a, b, bmin, bmax;
    double length;
    bool marked;
    std::vector<Line> parallelLines;
    std::vector<Line> nonParallelLines;
    Buffer(Line &l);
  };

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

  class PolygonToLineStringNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("linear_rings", { typeid(LinearRingCollection) });
      add_output("line_strings", typeid(LineStringCollection));
    }
    void process();
  };

  class CreateLineEquationsNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("line_strings", { typeid(LineStringCollection) });
      add_output("line_equations", typeid(std::vector<Line>));
    }
    void process();
  };

  class CreateLineBuffersNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("line_equations", { typeid(std::vector<Line>) });
      add_output("line_buffers", typeid(std::vector<Buffer>));
      add_output("line_strings", typeid(LineStringCollection));

      add_param("alpha", (float)5.0);
      add_param("epsilon", (float)1.0);
    }
    void process();
  };
}