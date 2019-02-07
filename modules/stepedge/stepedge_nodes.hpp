#pragma once

#include <geoflow/core/geoflow.hpp>

#include "point_edge.h"

namespace geoflow::nodes::stepedge {

  class AlphaShapeNode:public Node {
    public:
    using Node::Node;
    void init() {
      // add_input("points", TT_any);
      add_input("points", TT_any);
      add_output("alpha_rings", TT_linear_ring_collection);
      add_output("edge_points", TT_point_collection);
      add_output("alpha_edges", TT_line_string_collection);

      add_param("thres_alpha", (float) 0.7);
      add_param("extract_alpha_rings", (bool) false);
    }

    void gui(){
      ImGui::InputFloat("Alpha", &param<float>("thres_alpha"), 0.01, 1);
      ImGui::Checkbox("extract_alpha_rings", &param<bool>("extract_alpha_rings"));
    }
    void process();
  };

  class PolygonExtruderNode:public Node {

    public:
    using Node::Node;
    void init() {
      add_input("polygons", TT_linear_ring_collection);
      add_input("point_clouds", TT_point_collection_list);
      add_output("polygons_extruded", TT_linear_ring_collection);
      add_output("height", TT_vec1f);
    }

    void process();
  };

  class Arr2LinearRingsNode:public Node {

    public:
    using Node::Node;
    void init() {
      add_input("arrangement", TT_any);
      add_output("linear_rings", TT_linear_ring_collection);
      add_output("attributes", TT_attribute_map_f);
    }

    void process();
  };

  class ExtruderNode:public Node {
    bool do_walls=true, do_roofs=true;
    public:
    using Node::Node;
    void init() {
      add_input("arrangement", TT_any);
      add_output("cell_id_vec1i", TT_vec1i);
      add_output("rms_errors", TT_vec1f);
      add_output("max_errors", TT_vec1f);
      add_output("segment_coverages", TT_vec1f);
      add_output("triangles", TT_triangle_collection);
      add_output("normals_vec3f", TT_vec3f);
      add_output("labels_vec1i", TT_vec1i); // 0==ground, 1==roof, 2==outerwall, 3==innerwall
    }

    void gui() {
      ImGui::Checkbox("Do walls", &do_walls);
      ImGui::Checkbox("Do roofs", &do_roofs);
    }
    void process();
  };

  class ProcessArrangementNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("arrangement", TT_any);
      add_input("points", TT_any);
      add_output("arrangement", TT_any);
      add_param("step_height_threshold", (float) 1.0);
      add_param("zrange_threshold", (float) 0.2);
      add_param("merge_segid", (bool) true);
      add_param("merge_zrange", (bool) false);
      add_param("merge_step_height", (bool) false);
      add_param("merge_unsegmented", (bool) false);
      add_param("merge_dangling_egdes", (bool) false);
    }

    void gui() {
      ImGui::DragFloat("Min step height", &param<float>("step_height_threshold"), 0.1);
      ImGui::DragFloat("zrange_threshold", &param<float>("zrange_threshold"), 0.1);
      ImGui::Checkbox("merge_segid", &param<bool>("merge_segid"));
      ImGui::Checkbox("merge_zrange", &param<bool>("merge_zrange"));
      ImGui::Checkbox("merge_step_height", &param<bool>("merge_step_height"));
      ImGui::Checkbox("merge_unsegmented", &param<bool>("merge_unsegmented"));
      ImGui::Checkbox("merge_dangling_egdes", &param<bool>("merge_dangling_egdes"));
    }
    void process();
  };

  class BuildArrangementNode:public Node {

    public:
    bool remove_unsupported=false;

    using Node::Node;
    void init() {
      add_input("edge_segments", TT_line_string_collection);
      add_input("footprint", TT_linear_ring);
      add_output("arrangement", TT_any);
      add_output("arr_segments", TT_line_string_collection);
    }
    void gui() {
      ImGui::Checkbox("Remove unsupported edges", &remove_unsupported);
    }
    void process();
  };

  class DetectLinesNode:public Node {
    config c;

    public:
    bool use_linear_neighboorhood=false;

    using Node::Node;
    void init() {
      add_input("edge_points", {TT_point_collection, TT_linear_ring_collection});
      add_output("edge_segments", TT_line_string_collection);
    }

    void gui(){
      ImGui::InputFloat("Dist thres", &c.linedetect_dist_threshold, 0.01, 1);
      ImGui::InputInt("Segment cnt min", &c.linedetect_min_segment_count);
      ImGui::InputInt("K", &c.linedetect_k);
      ImGui::Checkbox("Use linear neighbourhood for ring input", &use_linear_neighboorhood);
    }
    void process();
  };

  class ClassifyEdgePointsNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_output("edge_points", TT_any);
      add_output("edge_points_vec3f", TT_vec3f);
      add_input("points", TT_any);

      add_param("classify_jump_count_min", (int) 1);
      add_param("classify_jump_count_max", (int) 5);
      add_param("classify_line_dist", (float) 0.005);
      add_param("classify_jump_ele", (float) 1.0);
    }

    void gui(){
      ImGui::InputInt("Jump cnt min", &param<int>("classify_jump_count_min"));
      ImGui::InputInt("Jump cnt max", &param<int>("classify_jump_count_max"));
      ImGui::InputFloat("Line dist", &param<float>("classify_line_dist"), 0.01, 1);
      ImGui::InputFloat("Elevation jump", &param<float>("classify_jump_ele"), 0.01, 1);
    }

    void process();
  };

  class ComputeMetricsNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_output("points", TT_any);
      add_input("points", TT_point_collection); // change to Feature
      add_output("plane_id", TT_vec1i);
      add_output("is_wall", TT_vec1i);
      add_output("is_horizontal", TT_vec1i);
      add_output("line_dist", TT_vec1f);
      add_output("jump_count", TT_vec1f);
      add_output("jump_ele", TT_vec1f);
      add_output("points_c", TT_point_collection);

      add_param("metrics_normal_k", (int) 10);
      add_param("metrics_plane_min_points", (int) 25);
      add_param("metrics_plane_epsilon", (float) 0.2);
      add_param("metrics_plane_normal_threshold", (float) 0.75);
      add_param("metrics_is_horizontal_threshold", (float) 0.9);
      add_param("metrics_is_wall_threshold", (float) 0.3);
      add_param("metrics_k_linefit", (int) 15);
      add_param("metrics_k_jumpcnt_elediff", (int) 10);
    }

    void gui(){
      ImGui::InputInt("K estimate normal ", &param<int>("metrics_normal_k"));
      ImGui::InputInt("Plane min points", &param<int>("metrics_plane_min_points"));
      ImGui::InputFloat("Plane epsilon", &param<float>("metrics_plane_epsilon"), 0.01, 1);
      ImGui::InputFloat("Plane normal thres", &param<float>("metrics_plane_normal_threshold"), 0.01, 1);
      ImGui::InputFloat("Wall angle thres", &param<float>("metrics_is_wall_threshold"), 0.01, 1);
      ImGui::InputFloat("Is horizontal", &param<float>("metrics_is_horizontal_threshold"), 0.01, 1);
      ImGui::InputInt("K linefit", &param<int>("metrics_k_linefit"));
      ImGui::InputInt("K jumpedge", &param<int>("metrics_k_jumpcnt_elediff"));
    }

    void process();
  };

  class LASInPolygonsNode:public Node {
    std::vector<PointCollection> point_clouds;
    LinearRingCollection polygons;

    public:
    int footprint_id=0;
    // char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
    // char las_filepath[256] = "";
    using Node::Node;
    void init() {
      add_input("polygons", TT_linear_ring_collection);
      add_output("point_clouds", TT_point_collection_list);
      add_output("points", TT_point_collection);
      add_output("footprint", TT_linear_ring);

      add_param("las_filepath", (std::string) "");
    }

    void gui() {
      ImGui::InputText("LAS file path", &param<std::string>("las_filepath"));
      if (ImGui::SliderInt("#", &footprint_id, 0, polygons.size()-1)) {
        // if(run_on_change) {
        //   manager.run(*this);
        // } else {
        if (footprint_id < polygons.size() && footprint_id >= 0) {
          notify_children();
          output("points").set(point_clouds[footprint_id]);
          output("point_clouds").set(point_clouds);
          output("footprint").set(polygons[footprint_id]);
          propagate_outputs();
        } else { footprint_id = polygons.size()-1; }
      }
    }
    void process();
  };

  struct LineCluster {
    //                source_id, target_id, is_footprint
    typedef std::tuple<size_t, size_t, bool> SegmentTuple; 
    Vector_2 ref_vec; // direction of reference line for this cluster (exact_exact arrangent traits)
    Point_2 ref_point; // reference point on reference line
    vec1f vertices; // stored as distances from ref_point in direction of ref_vec
    std::vector<SegmentTuple> segments;
  };

  struct ValueCluster {
    Vector_2 ref_vec;
    Vector_2 ref_point;
    std::vector<size_t> idx;
  };

  class RegulariseLinesNode:public Node {
    static constexpr double pi = 3.14159265358979323846;

    public:
    using Node::Node;
    void init() {
      add_input("edge_segments", TT_line_string_collection);
      add_input("footprint", TT_linear_ring);
      add_output("edges_out", TT_line_string_collection);
      add_output("merged_edges_out", TT_line_string_collection);
      add_output("cluster_labels", TT_vec1i);
      // add_output("footprint_labels", TT_vec1i);
      // add_output("line_clusters", TT_any); // ie a LineCluster
      // add_output("tmp_vec3f", TT_vec3f);
      add_param("dist_threshold", (float) 0.5);
      add_param("angle_threshold", (float) 0.1);
    }

    void gui(){
      ImGui::DragFloat("Distance threshold", &param<float>("dist_threshold"), 0.1, 0);
      ImGui::DragFloat("Angle threshold", &param<float>("angle_threshold"), 0.01, 0.01, pi);
    }
    void process();
  };

  class LOD13GeneratorNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("point_clouds", TT_point_collection_list);
      add_input("polygons", TT_linear_ring_collection);
      add_output("decomposed_footprints", TT_linear_ring_collection);
      add_output("attributes", TT_attribute_map_f);

      add_param("step_height_threshold", (float) 1.0);
      add_param("zrange_threshold", (float) 0.2);
      add_param("merge_segid", (bool) true);
      add_param("merge_zrange", (bool) false);
      add_param("merge_step_height", (bool) false);
      add_param("merge_unsegmented", (bool) false);
      add_param("merge_dangling_egdes", (bool) false);
    }

    void gui(){
      ImGui::InputFloat("Step height", &param<float>("step_height_threshold"), 0.1, 1);
      ImGui::DragFloat("zrange_threshold", &param<float>("zrange_threshold"), 0.1);
      ImGui::Checkbox("merge_segid", &param<bool>("merge_segid"));
      ImGui::Checkbox("merge_zrange", &param<bool>("merge_zrange"));
      ImGui::Checkbox("merge_step_height", &param<bool>("merge_step_height"));
      ImGui::Checkbox("merge_unsegmented", &param<bool>("merge_unsegmented"));
      ImGui::Checkbox("merge_dangling_egdes", &param<bool>("merge_dangling_egdes"));
    }
    void process();
  };

}