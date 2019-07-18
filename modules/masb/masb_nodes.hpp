#include <geoflow/geoflow.hpp>

#include <compute_ma_processing.h>
#include <compute_normals_processing.h>

namespace geoflow::nodes::mat {

  class ComputeMedialAxisNode:public Node {
    public:
    masb::ma_parameters params;
    float interval = 2;
    double zero=0,pi=3.14;
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

      add_param("initial_radius", ParamBoundedFloat(params.initial_radius, 0, 1000, "Initial radius");
      add_param("denoise_preserve", ParamBoundedDouble(params.denoise_preserve, 0, pi, "Denoise preserve");
      add_param("denoise_planar", ParamBoundedDouble(params.denoise_planar, 0, pi, "Denoise planar");
      add_param("nan_for_initr", ParamBool(params.nan_for_initr, "NaN for initR"));
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
      
      add_param("k", ParamBoundedFloat(params.k, 1, 100, "k"));
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
    }
    void process();
  };

  class TestPointsNode:public Node {
    int grid=10;
    public:
    using Node::Node;
    void init() {
      add_output("points", typeid(PointCollection));
      add_output("normals", typeid(vec3f));
      add_output("values", typeid(vec1f));
      
      add_param("grid", ParamInt(grid, "Grid size"));
    }
    void process() {
      PointCollection points;
      vec3f normals;
      vec1f values;
      auto& N = grid;
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
    int shape_count = 15;
    int min_count = 10;
    float bisector_angle = 5;
    float separation_angle = 5;
    float ball_overlap = 1.2;
    int k = 10;
    int method = 0;
    public:
    using Node::Node;
    void init() {
      add_input("ma_coords", typeid(PointCollection));
      add_input("ma_bisector", typeid(vec3f));
      add_input("ma_sepangle", typeid(vec1f));
      add_input("ma_radii", typeid(vec1f));
      add_output("segment_ids", typeid(vec1i));

      add_param("shape_count", ParamInt(shape_count, "shape_count"));
      add_param("min_count", ParamInt(min_count, "min_count"));
      add_param("bisector_angle", ParamBoundedFloat(bisector_angle, 0, 180, "bisector_angle"));
      add_param("separation_angle", ParamBoundedFloat(separation_angle, 0, 180, "separation_angle"));
      add_param("ball_overlap", ParamBoundedFloat(ball_overlap, 0,10, "ball_overlap"));
      add_param("k", ParamInt(k, "k"));
      add_param("method", ParamInt(method, "method"));
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