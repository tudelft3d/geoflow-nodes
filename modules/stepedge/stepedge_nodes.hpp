#pragma once

#include <geoflow/core/geoflow.hpp>

#include "point_edge.h"

namespace geoflow::nodes::stepedge {

  class AlphaShapeNode:public Node {
    public:
    using Node::Node;
    void init() {
      // add_input("points", TT_any);
      add_input("pts_per_roofplane", TT_any);
      add_output("alpha_rings", TT_linear_ring_collection);
      add_output("edge_points", TT_point_collection);
      add_output("alpha_edges", TT_line_string_collection);
      add_output("alpha_triangles", TT_triangle_collection);
      add_output("segment_ids", TT_vec1i);
      add_output("boundary_points", TT_point_collection);

      add_param("thres_alpha", (float) 0.15);
      add_param("optimal_alpha", (bool) true);
      add_param("optimal_only_if_needed", (bool) true);
    }

    void gui(){
      ImGui::InputFloat("Alpha", &param<float>("thres_alpha"), 0.01, 1);
      ImGui::Checkbox("optimal_alpha", &param<bool>("optimal_alpha"));
      if (param<bool>("optimal_alpha"))
        ImGui::Checkbox("Only if needed", &param<bool>("optimal_only_if_needed"));
    }
    void process();
  };

  class Ring2SegmentsNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("rings", TT_linear_ring_collection);
      add_output("edge_segments", TT_segment_collection);
      add_output("ring_idx", TT_any);
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
      add_output("plane_id", TT_vec1i);
      add_output("rms_errors", TT_vec1f);
      add_output("max_errors", TT_vec1f);
      add_output("elevations", TT_vec1f);
      add_output("segment_coverages", TT_vec1f);
      add_output("triangles", TT_triangle_collection);
      add_output("normals_vec3f", TT_vec3f);
      add_output("labels_vec1i", TT_vec1i); // 0==ground, 1==roof, 2==outerwall, 3==innerwall

      add_param("in_footprint", (bool) false);
    }

    void gui() {
      ImGui::Checkbox("Do walls", &do_walls);
      ImGui::Checkbox("Do roofs", &do_roofs);
      ImGui::Checkbox("In footprint", &param<bool>("in_footprint"));
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
      add_param("merge_step_height", (bool) true);
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
      add_input("edge_segments", {TT_line_string_collection, TT_linear_ring_collection});
      add_input("footprint", TT_linear_ring);
      add_output("arrangement", TT_any);
      add_output("arr_segments", TT_line_string_collection);
    }
    void gui() {
      ImGui::Checkbox("Remove unsupported edges", &remove_unsupported);
    }
    void process();
  };

  class BuildArrFromRingsExactNode:public Node {

    public:
    // bool remove_unsupported=false;

    using Node::Node;
    void init() {
      add_input("rings", TT_any);
      add_input("pts_per_roofplane", TT_any);
      add_input("footprint", TT_any);
      add_output("arrangement", TT_any);
      add_output("arr_segments", TT_line_string_collection);

      add_param("z_percentile", (float) 0.9);
      add_param("flood_to_unsegmented", (bool) true);
      add_param("dissolve_edges", (bool) true);
      add_param("dissolve_stepedges", (bool) true);
      add_param("step_height_threshold", (float) 1.0);
    }
    void gui() {
      ImGui::SliderFloat("Elevation percentile", &param<float>("z_percentile"), 0, 1);
      ImGui::Checkbox("Flood to unsegmented", &param<bool>("flood_to_unsegmented"));
      ImGui::Checkbox("Dissolve edges", &param<bool>("dissolve_edges"));
      ImGui::Checkbox("Dissolve stepedges", &param<bool>("dissolve_stepedges"));
      ImGui::SliderFloat("step_height_threshold", &param<float>("step_height_threshold"), 0, 100);
    }
    void process();
  };

  class BuildArrFromRingsNode:public Node {

    public:
    // bool remove_unsupported=false;

    using Node::Node;
    void init() {
      add_input("rings", TT_linear_ring_collection);
      add_input("pts_per_roofplane", TT_any);
      add_input("footprint", TT_linear_ring);
      add_output("arrangement", TT_any);
      add_output("arr_segments", TT_line_string_collection);

      add_param("z_percentile", (float) 0.9);
      add_param("flood_to_unsegmented", (bool) true);
      add_param("dissolve_edges", (bool) true);
      add_param("dissolve_stepedges", (bool) true);
      add_param("step_height_threshold", (float) 1.0);
    }
    void gui() {
      ImGui::SliderFloat("Elevation percentile", &param<float>("z_percentile"), 0, 1);
      ImGui::Checkbox("Flood to unsegmented", &param<bool>("flood_to_unsegmented"));
      ImGui::Checkbox("Dissolve edges", &param<bool>("dissolve_edges"));
      ImGui::Checkbox("Dissolve stepedges", &param<bool>("dissolve_stepedges"));
      ImGui::SliderFloat("step_height_threshold", &param<float>("step_height_threshold"), 0, 100);
    }
    void process();
  };
  class LinearRingtoRingsNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("linear_ring", TT_linear_ring);
      add_output("linear_rings", TT_linear_ring_collection);
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
      add_output("edge_segments", TT_segment_collection);
      add_output("ring_idx", TT_any);
      add_output("ring_id", TT_vec1i);
      add_output("ring_order", TT_vec1i);
      add_output("is_start", TT_vec1i);
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

  class DetectPlanesNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("points", TT_point_collection);
      add_output("plane_id", TT_vec1i);
      add_output("is_wall", TT_vec1i);
      add_output("is_horizontal", TT_vec1i);
      
      add_output("pts_per_roofplane", TT_any);

      add_output("class", TT_int);
      add_output("classf", TT_float);
      add_output("horiz_roofplane_cnt", TT_float);
      add_output("slant_roofplane_cnt", TT_float);

      add_param("metrics_normal_k", (int) 10);
      add_param("metrics_plane_min_points", (int) 50);
      add_param("metrics_plane_epsilon", (float) 0.15);
      add_param("metrics_plane_normal_threshold", (float) 0.75);
      add_param("metrics_is_horizontal_threshold", (float) 0.96);
      add_param("metrics_is_wall_threshold", (float) 0.3);
    }

    void gui(){
      ImGui::InputInt("K estimate normal ", &param<int>("metrics_normal_k"));
      ImGui::InputInt("Plane min points", &param<int>("metrics_plane_min_points"));
      ImGui::InputFloat("Plane epsilon", &param<float>("metrics_plane_epsilon"), 0.01, 1);
      ImGui::InputFloat("Plane normal thres", &param<float>("metrics_plane_normal_threshold"), 0.01, 1);
      ImGui::InputFloat("Wall angle thres", &param<float>("metrics_is_wall_threshold"), 0.01, 1);
      ImGui::InputFloat("Is horizontal", &param<float>("metrics_is_horizontal_threshold"), 0.01, 1);
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
      add_param("metrics_plane_min_points", (int) 50);
      add_param("metrics_plane_epsilon", (float) 0.15);
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
    public:
    using Node::Node;
    void init() {
      add_input("polygons", TT_linear_ring_collection);
      add_output("point_clouds", TT_point_collection_list);

      add_param("las_filepath", (std::string) "");
    }

    void gui() {
      ImGui::InputText("LAS file path", &param<std::string>("las_filepath"));
    }
    void process();
  };
  
  class BuildingSelectorNode:public Node {
    public:
    int building_id=0, polygon_count=0;
    using Node::Node;
    void init() {
      add_input("point_clouds", TT_point_collection_list);
      add_input("polygons", TT_linear_ring_collection);
      add_output("point_cloud", TT_point_collection);
      add_output("polygon", TT_linear_ring);

      // add_param("building_id", (int) 0);
    }

    void gui() {
      if (ImGui::SliderInt("#", &building_id, 0, polygon_count-1)) {
        // if(run_on_change) {
        //   manager.run(*this);
        // } else {
        if (building_id < polygon_count && building_id >= 0) {
          // notify_children();
          // output("points").set(point_clouds[building_id]);
          // output("point_clouds").set(point_clouds);
          // output("footprint").set(polygons[building_id]);
          // propagate_outputs();
        } else { building_id = polygon_count-1; }
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
      add_input("edge_segments", TT_segment_collection);
      add_input("footprint", TT_linear_ring);
      add_output("edges_out", TT_line_string_collection);
      add_output("merged_edges_out", TT_line_string_collection);
      add_output("cluster_labels", TT_vec1i);
      // add_output("footprint_labels", TT_vec1i);
      // add_output("line_clusters", TT_any); // ie a LineCluster
      // add_output("tmp_vec3f", TT_vec3f);
      add_param("dist_threshold", (float) 0.5);
      add_param("angle_threshold", (float) 0.2);
    }

    void gui(){
      ImGui::DragFloat("Distance threshold", &param<float>("dist_threshold"), 0.1, 0);
      ImGui::DragFloat("Angle threshold", &param<float>("angle_threshold"), 0.01, 0.01, pi);
    }
    void process();
  };

  class RegulariseRingsNode:public Node {

    public:
    using Node::Node;
    void init() {
      add_input("edge_segments", TT_segment_collection);
      add_input("ring_idx", TT_any);
      // add_input("ring_id", TT_vec1i);
      // add_input("ring_order", TT_vec1i);
      // add_input("edge_segments", TT_segment_collection);
      add_input("footprint", TT_linear_ring);
      add_output("edges_out", TT_segment_collection);
      add_output("rings_out", TT_linear_ring_collection);
      add_output("footprint_out", TT_linear_ring);
      add_output("exact_rings_out", TT_any);
      add_output("exact_footprint_out", TT_any);
      // add_output("footprint_labels", TT_vec1i);
      // add_output("line_clusters", TT_any); // ie a LineCluster
      // add_output("tmp_vec3f", TT_vec3f);
      add_param("dist_threshold", (float) 0.5);
      add_param("angle_threshold", (float) 0.1);
      add_param("snap_threshold", (float) 1.0);
    }

    void gui(){
      ImGui::DragFloat("Distance threshold", &param<float>("dist_threshold"), 0.1, 0);
      ImGui::DragFloat("Angle threshold", &param<float>("angle_threshold"), 0.01, 0.01, 3.1415);
      ImGui::DragFloat("Snap threshold", &param<float>("snap_threshold"), 0.01, 0.01, 10);
    }
    void process();
  };


  class PrintResultNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("in", {TT_float});
    }
    void gui() {
      ImGui::Text("Result: %f", input("in").get<float>());
    }
    void process(){};
  };


  class SimplifyPolygonNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("polygons", {TT_linear_ring_collection, TT_linear_ring});
      add_output("polygons_simp", TT_linear_ring_collection);
      add_output("polygon_simp", TT_linear_ring);

      add_param("threshold_stop_cost", (float) 0.01);
    }
    void gui() {
      if(ImGui::DragFloat("stop cost", &param<float>("threshold_stop_cost"),0.01, 0,1000)) {
        manager.run(*this);
      }
    }
    void process();
  };

}