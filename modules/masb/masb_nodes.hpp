#include <geoflow/core/geoflow.hpp>

#include <compute_ma_processing.h>
#include <compute_normals_processing.h>

namespace geoflow::nodes::mat {

  class ComputeMedialAxisNode:public Node {
    public:
    masb::ma_parameters params;
    const double zero=0,pi=3.14;
    using Node::Node;
    void init() {
      add_input("points", typeid(PointCollection));
      add_input("normals", typeid(vec3f));
      add_output("ma_coords", typeid(PointCollection));
      add_output("ma_radii", typeid(vec1f));
      add_output("ma_qidx", typeid(vec1i));
      add_output("ma_is_interior", typeid(vec1i));
      add_output("ma_sepangle", typeid(vec1f));
      add_output("ma_bisector", typeid(vec3f));
      add_output("ma_spoke_f1", typeid(vec3f));
      add_output("ma_spoke_f2", typeid(vec3f));
      add_output("ma_spokecross", typeid(vec3f));

      add_param("initial_radius", (float)200);
      add_param("denoise_preserve", (float)((PI / 180.0) * 20));
      add_param("denoise_planar", (float)((PI / 180.0) * 32));
      add_param("nan_for_initr", (bool)false);
    }
    void gui() {
      ImGui::SliderFloat("initial_radius", &param<float>("initial_radius"), 0, 1000);
      ImGui::SliderFloat("denoise_preserve", &param<float>("denoise_preserve"), 0, 3.1415);
      ImGui::SliderFloat("denoise_planar", &param<float>("denoise_planar"), 0, 3.1415);
      ImGui::Checkbox("nan_for_initr", &param<bool>("nan_for_initr"));
    }
    void process();
  };

  class ComputeNormalsNode:public Node {
    public:
    masb::normals_parameters params;
    float interval = 2;
    using Node::Node;
    void init() {
      add_input("points", typeid(PointCollection));
      add_output("normals", typeid(vec3f));
    }
    void gui(){
      ImGui::SliderInt("K", &params.k, 1, 100);
    }
    void process();
  };

  class SegmentMakerNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("sources", typeid(PointCollection));
      add_input("directions", typeid(vec3f));
      add_output("segments", typeid(SegmentCollection));
      
      add_param("length", (float) 1.0);
    }
    void gui(){
      ImGui::SliderFloat("Length multiplyer", &param<float>("length"), 0.01, 10);
    }
    void process();
  };

  class TestPointsNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_output("points", typeid(PointCollection));
      add_output("normals", typeid(vec3f));
      add_output("values", typeid(vec1f));
      add_param("grid", (int) 10);
    }
    void gui(){
      ImGui::InputInt("grid", &param<int>("grid"));
    }
    void process() {
      PointCollection points;
      vec3f normals;
      vec1f values;
      auto& N = param<int>("grid");
      for(int i = 0; i<N; ++i) {
        for(int j = 0; j<N; ++j) {
          points.push_back({float(i),float(j),0});
          normals.push_back({0,0,1});
          values.push_back(0);

          points.push_back({0,float(j),float(i)});
          normals.push_back({1,0,0});
          values.push_back(42);
        }
      }
      output("normals").set(normals);
      output("points").set(points);
      output("values").set(values);
    }
  };

  class RegionGrowMedialAxisNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("ma_coords", typeid(PointCollection));
      add_input("ma_bisector", typeid(vec3f));
      add_input("ma_sepangle", typeid(vec1f));
      add_input("ma_radii", typeid(vec1f));
      add_output("segment_ids", typeid(vec1i));

      add_param("shape_count", (int) 15);
      add_param("min_count", (int) 10);
      add_param("bisector_angle", (float) 5);
      add_param("separation_angle", (float) 5);
      add_param("ball_overlap", (float) 1.2);
      add_param("k", (int) 10);
      add_param("method", (int) 0);
    }
    void gui(){
      ImGui::SliderInt("k", &param<int>("k"), 0, 100);
      ImGui::SliderInt("min_count", &param<int>("min_count"), 1, 1000);
      ImGui::Separator();
      ImGui::Combo("method", &param<int>("method"), "bisector\0sepangle\0balloverlap\0count\0\0");
      switch (param<int>("method")) {
        case 0: {
          ImGui::SliderFloat("bisector_angle", 
          &param<float>("bisector_angle"), 0, 180); break;
        } case 1: {
          ImGui::SliderFloat("separation_angle", 
          &param<float>("separation_angle"), 0, 180); break;
        } case 2: {
          ImGui::SliderFloat("ball_overlap", 
          &param<float>("ball_overlap"), 0, 10); break;
        } case 3: {
          ImGui::SliderInt("shape_count", 
          &param<int>("shape_count"), 1, 1000); break;
        } default: break;
      };
    }
    void process();
  };
}