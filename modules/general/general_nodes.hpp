#include <geoflow/core/geoflow.hpp>

namespace geoflow::nodes::general {
  class MergeGeometriesNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("geometries1", { TT_point_collection,
                                TT_segment_collection,
                                TT_line_string_collection,
                                TT_linear_ring_collection, });
      add_input("geometries2", { TT_point_collection,
                                TT_segment_collection,
                                TT_line_string_collection,
                                TT_linear_ring_collection });
      add_output("geometries", TT_any);
    }
    void process();
  };

  class MergeLinestringsNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("lines1", { TT_line_string_collection });
      add_input("lines2", { TT_line_string_collection });
      add_output("lines", TT_line_string_collection);
    }
    void process();
  };

  class LineStringFilterNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("line_strings", {TT_line_string_collection});
      add_output("line_strings", TT_line_string_collection);

      add_param("filter_length", (float)0.0);
    }
    void process();
  };
}