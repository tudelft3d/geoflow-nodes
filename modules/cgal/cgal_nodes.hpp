#pragma once

#include <geoflow/core/geoflow.hpp>
#include "tinsimp.hpp"

namespace geoflow::nodes::cgal {

  typedef tinsimp::CDT  CDT;

  class CDTNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("geometries", { typeid(PointCollection), typeid(LineStringCollection) });
      add_output("cgal_cdt", typeid(CDT));
      add_output("triangles", typeid(TriangleCollection));

      add_param("create_triangles", (bool)false);
    }
    void gui() {
      ImGui::Checkbox("Create triangles", &param<bool>("create_triangles"));
    }
    void process();
  };

  class DTNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("points", typeid(PointCollection));
      add_output("cgal_dt", typeid(CDT));
    }
    void process();
  };

  class ComparePointDistanceNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("triangles1_vec3f", typeid(vec3f));
      add_input("triangles2_vec3f", typeid(vec3f));
      add_output("points", typeid(vec3f));
      add_output("distances1", typeid(vec1f));
      add_output("distances2", typeid(vec1f));
      add_output("diff", typeid(vec1f));

      add_param("las_filpath", (std::string) "");
      add_param("log_filpath", (std::string) "");
      add_param("thin_nth", (int)20);
    }
    void gui() {
      ImGui::InputText("LAS file path", &param<std::string>("las_filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 1, 100);
    }
    void process();
  };

  class PointDistanceNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("triangles", typeid(TriangleCollection));
      add_output("points", typeid(PointCollection));
      add_output("distances", typeid(vec1f));
      add_output("distance_min", typeid(float));
      add_output("distance_max", typeid(float));

      add_param("filepath", (std::string) "");
      add_param("thin_nth", (int)5);
      add_param("overwritez", (bool)false);
    }
    void gui() {
      ImGui::InputText("LAS file path", &param<std::string>("filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 1, 100);
    }
    void process();
  };

  class CDTDistanceNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("cgal_cdt_base", typeid(CDT));
      add_input("cgal_cdt_target", typeid(CDT));
      add_output("points", typeid(PointCollection));
      add_output("distance_min", typeid(float));
      add_output("distance_max", typeid(float));
    }
    void process();
  };

  class DensifyNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("geometries", { typeid(LineStringCollection) });
      add_output("dense_linestrings", typeid(LineStringCollection));

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
      add_input("geometries", { typeid(PointCollection), typeid(LineStringCollection) });
      add_output("triangles", typeid(TriangleCollection));
      add_output("normals", typeid(vec3f));
      add_output("selected_lines", typeid(LineStringCollection));
      add_output("cgal_cdt", typeid(CDT));
      // add_output("count", typeid(vec1ui));
      // add_output("error", typeid(vec1f));

      add_param("thres_error", (float)2);
      add_param("densify_interval", (float)2);
      add_param("create_triangles", (bool)true);
    }
    void gui() {
      if (ImGui::SliderFloat("Error threshold", &param<float>("thres_error"), 0, 100)) {
        manager.run(*this);
      }
      if (input("geometries").connected_type == typeid(LineStringCollection))
        ImGui::SliderFloat("Line densify", &param<float>("densify_interval"), 0, 100);
      ImGui::Checkbox("Create triangles", &param<bool>("create_triangles"));
    }
    void process();
  };

  class TinSimpLASReaderNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_output("triangles", typeid(TriangleCollection));
      add_output("cgal_cdt", typeid(CDT));

      add_param("filepath", (std::string) "");
      add_param("thin_nth", (int)5);
      add_param("thres_error", (float)2);
      add_param("create_triangles", (bool)true);
    }
    void gui() {
      ImGui::InputText("LAS file path", &param<std::string>("filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 1, 100);
      ImGui::SliderFloat("Error threshold", &param<float>("thres_error"), 0, 100);
      ImGui::Checkbox("Create triangles", &param<bool>("create_triangles"));
    }
    void process();
  };

  class SimplifyLine3DNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("lines", typeid(LineStringCollection));
      add_output("lines", typeid(LineStringCollection));

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
      add_input("lines", typeid(LineStringCollection));
      add_output("lines", typeid(LineStringCollection));

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
      add_input("lines", typeid(LineStringCollection));
      add_output("lines", typeid(LineStringCollection));

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
      add_input("polygons", typeid(LinearRingCollection));
      add_output("polygons_simp", typeid(LinearRingCollection));

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
      add_input("points", typeid(PointCollection)); //TT_point_collection_list
      add_input("labels", typeid(vec1i));

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
      add_input("cgal_cdt", typeid(CDT));
      add_input("min", typeid(float));
      add_input("max", typeid(float));
      add_output("lines", typeid(LineStringCollection));
      add_output("attributes", typeid(vec1i));

      add_param("interval", (float)1.0);
      add_param("exclude_begin", (float)-0.5);
      add_param("exclude_end", (float)0.5);
    }
    void gui() {
      ImGui::DragFloatRange2("Excluding range", &param<float>("exclude_begin"), &param<float>("exclude_end"), 0.5f, 0.0f, 100.0f, "Min: %.1f", "Max: %.1f");
      ImGui::SliderFloat("Interval", &param<float>("interval"), 1, 100);
    }
    void process();
  };

  class IsoLineSlicerNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("cgal_cdt", typeid(CDT));
      add_output("lines", typeid(LineStringCollection));
      add_output("attributes", typeid(AttributeMap));
    }
    void process();
  };

  class LineHeightNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("lines", typeid(LineStringCollection));
      add_output("lines", typeid(LineStringCollection));

      add_param("filepath", (std::string) "");
      add_param("thin_nth", (int)5);
    }
    void gui() {
      ImGui::InputText("LAS file path", &param<std::string>("filepath"));
      ImGui::SliderInt("Thin nth", &param<int>("thin_nth"), 1, 100);
    }
    void process();
  };  
  
  class LineHeightCDTNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("cgal_cdt", typeid(CDT));
      add_input("lines", typeid(LineStringCollection));
      add_output("lines", typeid(LineStringCollection));

      add_param("add_bbox", (bool)false);
      add_param("densify_interval", (float)2);
    }
    void gui() {
      ImGui::SliderFloat("Line densify", &param<float>("densify_interval"), 0, 100);
      ImGui::Checkbox("Add bounding box to lines", &param<bool>("add_bbox"));
    }
    void process();
  };

  class SimplifyLinesBufferNode:public Node {
  public:
    using Node::Node;
    void init() {
      add_input("polygons", typeid(LinearRingCollection));
      add_output("polygons_simp", typeid(LinearRingCollection));

      add_param("threshold", (float)1.0);
    }
    void gui() {
      ImGui::SliderFloat("Threshold", &param<float>("threshold"), 0, 100);
    }
    void process();
  };
}