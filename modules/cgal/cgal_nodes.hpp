#pragma once

#include <geoflow/core/geoflow.hpp>

namespace geoflow::nodes::cgal {
  class CDTNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("geometries", { TT_point_collection, TT_line_string_collection });
      add_output("cgal_cdt", TT_any);
      add_output("triangles", TT_triangle_collection);

      add_param("create_triangles", (bool)false);
    }
    void process();
  };

  class DTNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("points", TT_point_collection);
      add_output("cgal_dt", TT_any);
    }
    void process();
  };

  class ComparePointDistanceNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("triangles1_vec3f", TT_vec3f);
      add_input("triangles2_vec3f", TT_vec3f);
      add_output("points", TT_vec3f);
      add_output("distances1", TT_vec1f);
      add_output("distances2", TT_vec1f);
      add_output("diff", TT_vec1f);

      add_param("las_filpath", (std::string) "");
      add_param("log_filpath", (std::string) "");
      add_param("thin_nth", (int)20);
    }
    void gui() {
      ImGui::InputText("LAS file path", &param<std::string>("las_filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 0, 100);
    }
    void process();
  };

  class PointDistanceNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("triangles", TT_triangle_collection);
      add_output("points", TT_point_collection);
      add_output("distances", TT_vec1f);
      add_output("distance_min", TT_float);
      add_output("distance_max", TT_float);

      add_param("filepath", (std::string) "");
      add_param("thin_nth", (int)5);
      add_param("overwritez", (bool)false);
    }
    void gui() {
      ImGui::InputText("LAS file path", &param<std::string>("filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 0, 100);
    }
    void process();
  };

  class DensifyNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("geometries", { TT_line_string_collection });
      add_output("dense_linestrings", TT_line_string_collection);

      add_param("interval", (int)2);
    }
    void gui() {
      ImGui::SliderFloat("Interval", &param<float>("interval"), 0, 100);
    }
    void process();
  };

  class TinSimpNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("geometries", { TT_point_collection, TT_line_string_collection });
      add_output("triangles", TT_triangle_collection);
      add_output("normals", TT_vec3f);
      add_output("selected_lines", TT_line_string_collection);
      // add_output("count", TT_vec1ui);
      // add_output("error", TT_vec1f);

      add_param("thres_error", (float)2);
      add_param("densify_interval", (float)2);
    }
    void gui() {
      if (ImGui::SliderFloat("Error threshold", &param<float>("thres_error"), 0, 100)) {
        manager.run(*this);
      }
      if (input("geometries").connected_type == TT_line_string_collection)
        ImGui::SliderFloat("Line densify", &param<float>("densify_interval"), 0, 100);
    }
    void process();
  };

  class SimplifyLine3DNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("lines", TT_line_string_collection);
      add_output("lines", TT_line_string_collection);

      add_param("area_threshold", (float) 0.1);
    }
    void gui() {
      if (ImGui::DragFloat("stop cost", &param<float>("area_threshold"), 0.1)) {
        manager.run(*this);
      }
    }
    void process();
  };

  class SimplifyLineNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("lines", TT_line_string_collection);
      add_output("lines", TT_line_string_collection);

      add_param("threshold_stop_cost", (float) 0.1);
    }
    void gui() {
      if (ImGui::DragFloat("stop cost", &param<float>("threshold_stop_cost"), 0.01)) {
        manager.run(*this);
      }
    }
    void process();
  };

  class SimplifyLinesNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("lines", TT_line_string_collection);
      add_input("lines2", TT_line_string_collection);
      add_output("lines", TT_line_string_collection);

      add_param("threshold_stop_cost", (float) 0.1);
    }
    void gui() {
      if (ImGui::DragFloat("stop cost", &param<float>("threshold_stop_cost"), 0.01)) {
        manager.run(*this);
      }
    }
    void process();
  };

  class SimplifyFootprintsCDTNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("polygons", TT_linear_ring_collection);
      add_output("polygons_simp", TT_linear_ring_collection);

      add_param("threshold_stop_cost", (float) 0.1);
    }
    void gui() {
      if (ImGui::DragFloat("stop cost", &param<float>("threshold_stop_cost"), 0.01)) {
        manager.run(*this);
      }
    }
    void process();
  };

  class PLYWriterNode:public Node {
  public:
    bool multiple_files = true;

    using Node::Node;
    void init() {
      add_input("points", TT_point_collection); //TT_point_collection_list
      add_input("labels", TT_vec1i);

      add_param("filepath", (std::string) "out.ply");
      add_param("write_binary", (bool)false);
    }
    void gui() {
      ImGui::InputText("File path", &param<std::string>("filepath"));
      // ImGui::Checkbox("Write multiple files in case of point cloud list", &multiple_files);
      ImGui::Checkbox("Write binary output", &param<bool>("write_binary"));
    }
    void process();
  };

  class IsoLineNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("cgal_cdt", TT_any);
      add_input("min", TT_float);
      add_input("max", TT_float);
      add_output("lines", TT_line_string_collection);
      add_output("attributes", TT_attribute_map_f);
    }
    void process();
  };

  class IsoLineSlicerNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("cgal_cdt", TT_any);
      add_output("lines", TT_line_string_collection);
      add_output("attributes", TT_attribute_map_f);
    }
    void process();
  };

  class LineHeightNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("lines", TT_line_string_collection);
      add_output("lines", TT_line_string_collection);

      add_param("filepath", (std::string) "");
      add_param("thin_nth", (int)5);
    }
    void gui() {
      ImGui::InputText("LAS file path", &param<std::string>("filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 0, 100);
    }
    void process();
  };

  class SimplifyLinesBufferNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("polygons", TT_linear_ring_collection);
      add_output("polygons_simp", TT_linear_ring_collection);

      add_param("threshold", (float)1.0);
    }
    void gui() {
      ImGui::SliderFloat("Threshold", &param<float>("threshold"), 0, 100);
    }
    void process();
  };
}