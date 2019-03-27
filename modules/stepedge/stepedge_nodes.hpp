#pragma once

#include <geoflow/core/geoflow.hpp>

#include "point_edge.h"
#include "line_regulariser.hpp"

namespace geoflow::nodes::stepedge {

  class AlphaShapeNode:public Node {
    public:
    using Node::Node;
    void init() {
      // add_input("points", TT_any);
      add_input("pts_per_roofplane", typeid(std::unordered_map<int, std::vector<Point>>));
      add_output("alpha_rings", typeid(LinearRingCollection));
      add_output("edge_points", typeid(PointCollection));
      add_output("alpha_edges", typeid(LineStringCollection));
      add_output("alpha_triangles", typeid(TriangleCollection));
      add_output("segment_ids", typeid(vec1i));
      add_output("boundary_points", typeid(PointCollection));

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
      add_input("rings", typeid(LinearRingCollection));
      add_output("edge_segments", typeid(SegmentCollection));
      add_output("ring_idx", typeid(std::vector<std::vector<size_t>>));
    }
    void process();
  };

  class PointCloudMeanZNode:public Node {

    public:
    using Node::Node;
    void init() {
      add_input("point_clouds", typeid(std::vector<PointCollection>));
      add_output("height", typeid(vec1f));
    }

    void process();
  };

  class PolygonExtruderNode:public Node {

    public:
    using Node::Node;
    void init() {
      add_input("polygons", typeid(LinearRingCollection));
      add_input("heights", typeid(vec1f));
      add_output("rings_3d", typeid(LinearRingCollection));
      add_output("ring_types", typeid(vec1i));
    }

    void process();
  };

  class Arr2LinearRingsNode:public Node {

    public:
    using Node::Node;
    void init() {
      add_input("arrangement", typeid(Arrangement_2));
      add_output("linear_rings", typeid(LinearRingCollection));
      add_output("attributes", typeid(AttributeMap));
      add_param("only_in_footprint", (bool) true);
    }
    void gui() {
      ImGui::Checkbox("Only in footprint", &param<bool>("only_in_footprint"));
    }
    void process();
  };

  class ExtruderNode:public Node {
    bool do_walls=true, do_roofs=true;
    public:
    using Node::Node;
    void init() {
      add_input("arrangement", typeid(Arrangement_2));
      add_output("cell_id_vec1i", typeid(vec1i));
      add_output("plane_id", typeid(vec1i));
      add_output("rms_errors", typeid(vec1f));
      add_output("max_errors", typeid(vec1f));
      add_output("elevations", typeid(vec1f));
      add_output("segment_coverages", typeid(vec1f));
      add_output("triangles", typeid(TriangleCollection));
      add_output("normals_vec3f", typeid(vec3f));
      add_output("labels_vec1i", typeid(vec1i)); // 0==ground, 1==roof, 2==outerwall, 3==innerwall

      add_param("in_footprint", (bool) false);
    }

    void gui() {
      ImGui::Checkbox("Do walls", &do_walls);
      ImGui::Checkbox("Do roofs", &do_roofs);
      ImGui::Checkbox("In footprint", &param<bool>("in_footprint"));
    }
    void process();
  };

    class PolygonGrowerNode:public Node {

    public:
    using Node::Node;
    void init() {
      add_input("rings", typeid(LinearRingCollection));
      add_output("rings", typeid(LinearRingCollection));

      add_param("extension", (float) 0.1);
    }

    void gui() {
      ImGui::DragFloat("Extension length", &param<float>("extension"));
    }
    void process();
  };

  class ProcessArrangementNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("arrangement", typeid(Arrangement_2));
      add_input("points", typeid(PNL_vector));
      add_output("arrangement", typeid(Arrangement_2));
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
      add_input("edge_segments", {typeid(LineStringCollection), typeid(LinearRingCollection)});
      add_input("footprint", typeid(LinearRing));
      add_output("arrangement", typeid(Arrangement_2));
      add_output("arr_segments", typeid(LineStringCollection));
    }
    void gui() {
      ImGui::Checkbox("Remove unsupported edges", &remove_unsupported);
    }
    void process();
  };

  class BuildArrFromRingsExactNode:public Node {

    public:
    bool arr_is_valid=false;
    int vcount, ecount;

    using Node::Node;
    void init() {
      add_input("rings", typeid(std::vector<linereg::Polygon_2>));
      add_input("pts_per_roofplane", typeid(std::unordered_map<int, std::vector<Point>>));
      add_input("footprint", {typeid(linereg::Polygon_2), typeid(LinearRing)});
      add_output("noseg_area_a", typeid(float));
      add_output("noseg_area_r", typeid(float));
      add_output("arrangement", typeid(Arrangement_2));
      add_output("arr_segments", typeid(LineStringCollection));
      add_output("snap_to_e", typeid(SegmentCollection));
      add_output("snap_to_v", typeid(PointCollection));
      add_output("snap_v", typeid(PointCollection));
      add_output("snap_fp_to_e", typeid(SegmentCollection));
      add_output("snap_fp_to_v", typeid(PointCollection));
      add_output("snap_fp_v", typeid(PointCollection));

      add_param("extrude_unsegmented", (bool) true);
      add_param("extrude_mindensity", (float) 5);
      add_param("z_percentile", (float) 0.9);
      add_param("rel_area_thres", (float) 0.1);
      add_param("flood_to_unsegmented", (bool) true);
      add_param("dissolve_edges", (bool) true);
      add_param("dissolve_stepedges", (bool) true);
      add_param("step_height_threshold", (float) 1.0);
      add_param("snap_clean", (bool) true);
      add_param("snap_clean_fp", (bool) false);
      add_param("snap_detect_only", (bool) false);
      add_param("snap_dist", (float) 1.0);
    }
    void gui() {
      ImGui::SliderFloat("Elevation percentile", &param<float>("z_percentile"), 0, 1);
      ImGui::SliderFloat("Preserve split ring area", &param<float>("rel_area_thres"), 0.01, 1);
      ImGui::Checkbox("Snap", &param<bool>("snap_clean"));
      ImGui::Checkbox("Snap fp", &param<bool>("snap_clean_fp"));
      ImGui::Checkbox("Snap detect only", &param<bool>("snap_detect_only"));
      ImGui::SliderFloat("Snap distance", &param<float>("snap_dist"), 0.01, 5);
      ImGui::Checkbox("Extrude unsegmented", &param<bool>("extrude_unsegmented"));
      ImGui::SliderFloat("Extrude min density", &param<float>("extrude_mindensity"), 1, 20);
      ImGui::Checkbox("Flood to unsegmented", &param<bool>("flood_to_unsegmented"));
      ImGui::Checkbox("Dissolve edges", &param<bool>("dissolve_edges"));
      ImGui::Checkbox("Dissolve stepedges", &param<bool>("dissolve_stepedges"));
      ImGui::SliderFloat("step_height_threshold", &param<float>("step_height_threshold"), 0, 100);
      ImGui::Text("Arrangement valid? %s", arr_is_valid? "yes" : "no");
      ImGui::Text("vcount: %d, ecount: %d", vcount, ecount);
    }
    void arr_snapclean(Arrangement_2& arr);
    void arr_snapclean_from_fp(Arrangement_2& arr);
    void arr_process(Arrangement_2& arr);
    void arr_assign_pts_to_unsegmented(Arrangement_2& arr, std::vector<Point>& points);
    void process();
  };

  // class BuildArrFromRingsNode:public Node {

  //   public:
  //   // bool remove_unsupported=false;
  //   bool arr_is_valid=false;

  //   using Node::Node;
  //   void init() {
  //     add_input("rings", typeid(LinearRingCollection));
  //     add_input("pts_per_roofplane", TT_any);
  //     add_input("footprint", typeid(LinearRing));
  //     add_output("arrangement", typeid(Arrangement_2));
  //     add_output("arr_segments", typeid(LineStringCollection));

  //     add_param("z_percentile", (float) 0.9);
  //     add_param("flood_to_unsegmented", (bool) true);
  //     add_param("dissolve_edges", (bool) true);
  //     add_param("dissolve_stepedges", (bool) true);
  //     add_param("step_height_threshold", (float) 1.0);
  //   }
  //   void gui() {
  //     ImGui::SliderFloat("Elevation percentile", &param<float>("z_percentile"), 0, 1);
  //     ImGui::Checkbox("Flood to unsegmented", &param<bool>("flood_to_unsegmented"));
  //     ImGui::Checkbox("Dissolve edges", &param<bool>("dissolve_edges"));
  //     ImGui::Checkbox("Dissolve stepedges", &param<bool>("dissolve_stepedges"));
  //     ImGui::SliderFloat("step_height_threshold", &param<float>("step_height_threshold"), 0, 100);
  //     ImGui::Text("Arrangement is valid? %d", arr_is_valid);
  //   }
  //   void process(){};
  // };
  class LinearRingtoRingsNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("linear_ring", typeid(LinearRing));
      add_output("linear_rings", typeid(LinearRingCollection));
    }
    void process();
  };

  class DetectLinesNode:public Node {
    public:
    typedef std::pair<size_t,size_t> IDPair;
    struct Cmp {
      bool operator()(const IDPair& lhs, const IDPair& rhs) const { 
          return lhs.first < rhs.first; 
      }
    };
    typedef std::map<IDPair, size_t, Cmp> RingSegMap;

    using Node::Node;
    void init() {
      add_input("edge_points", {typeid(PointCollection), typeid(LinearRingCollection)});
      add_output("edge_segments", typeid(SegmentCollection));
      add_output("lines3d", typeid(SegmentCollection));
      add_output("ring_edges", typeid(SegmentCollection));
      add_output("ring_idx", typeid(std::vector<std::vector<size_t>>));
      add_output("ring_id", typeid(vec1i));
      add_output("ring_order", typeid(vec1i));
      add_output("is_start", typeid(vec1i));

      add_param("linear_knn", (bool) false);
      add_param("dist_thres", (float) 0.4);
      add_param("min_cnt_upper", (int) 10);
      add_param("min_cnt_lower", (int) 5);
      add_param("k", (int) 10);
      add_param("snap_threshold", (float) 1);
      add_param("line_extend", (float) 0.05);
      add_param("perform_chaining", (bool) true);
      add_param("remove_overlap", (bool) true);
    }

    void gui(){
      ImGui::InputFloat("Dist thres", &param<float>("dist_thres"), 0.01, 1);
      ImGui::DragIntRange2("Minimum segment count", &param<int>("min_cnt_lower"), &param<int>("min_cnt_upper"), 1, 0);
      ImGui::InputInt("K", &param<int>("k"));
      ImGui::Checkbox("Use linear neighbourhood for ring input", &param<bool>("linear_knn"));
      ImGui::InputFloat("Extend lines", &param<float>("line_extend"), 0.01, 1);
      ImGui::Checkbox("Perform chaining", &param<bool>("perform_chaining"));
      ImGui::InputFloat("Chain snap thres", &param<float>("snap_threshold"), 0.01, 1);
      ImGui::Checkbox("Remove overlap", &param<bool>("remove_overlap"));
    }
    inline void detect_lines_ring_m1(linedect::LineDetector& LD, SegmentCollection& segments_out);
    inline size_t detect_lines_ring_m2(linedect::LineDetector& LD, SegmentCollection& segments_out);
    inline void detect_lines(linedect::LineDetector& LD);
    void process();
  };

  class ClassifyEdgePointsNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_output("edge_points", typeid(std::vector<linedect::Point>));
      add_output("edge_points_vec3f", typeid(vec3f));
      add_input("points", typeid(PNL_vector));

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
      add_input("points", typeid(PointCollection));
      add_output("plane_id", typeid(vec1i));
      add_output("is_wall", typeid(vec1i));
      add_output("is_horizontal", typeid(vec1i));
      
      add_output("pts_per_roofplane", typeid(std::unordered_map<int, std::vector<Point>>));

      add_output("roof_pt_cnt", typeid(int));
      add_output("class", typeid(int));
      add_output("classf", typeid(float));
      add_output("horiz_roofplane_cnt", typeid(float));
      add_output("slant_roofplane_cnt", typeid(float));

      add_param("only_horizontal", (bool) true);
      add_param("horiz_min_count", (float) 0.95);
      add_param("metrics_normal_k", (int) 10);
      add_param("metrics_plane_min_points", (int) 20);
      add_param("metrics_plane_epsilon", (float) 0.2);
      add_param("metrics_plane_normal_threshold", (float) 0.75);
      add_param("metrics_is_horizontal_threshold", (float) 0.97);
      add_param("metrics_is_wall_threshold", (float) 0.3);
      add_param("n_refit", (int) 5);
    }

    void gui(){
      ImGui::InputInt("Refit every n points", &param<int>("n_refit"));
      ImGui::InputInt("K estimate normal ", &param<int>("metrics_normal_k"));
      ImGui::InputInt("Plane min points", &param<int>("metrics_plane_min_points"));
      ImGui::InputFloat("Plane epsilon", &param<float>("metrics_plane_epsilon"), 0.01, 1);
      ImGui::InputFloat("Plane normal thres", &param<float>("metrics_plane_normal_threshold"), 0.01, 1);
      ImGui::InputFloat("Wall angle thres", &param<float>("metrics_is_wall_threshold"), 0.01, 1);
      ImGui::InputFloat("Is horizontal", &param<float>("metrics_is_horizontal_threshold"), 0.01, 1);
      ImGui::Checkbox("Output only horizontal planes", &param<bool>("only_horizontal"));
      if(param<bool>("only_horizontal"))
        ImGui::InputFloat("Min horiz point count", &param<float>("horiz_min_count"), 0.01, 1);
    }

    void process();
  };

  class ComputeMetricsNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_output("points", typeid(PNL_vector));
      add_input("points", typeid(PointCollection)); // change to Feature
      add_output("plane_id", typeid(vec1i));
      add_output("is_wall", typeid(vec1i));
      add_output("is_horizontal", typeid(vec1i));
      add_output("line_dist", typeid(vec1f));
      add_output("jump_count", typeid(vec1f));
      add_output("jump_ele", typeid(vec1f));
      add_output("points_c", typeid(PointCollection));

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
      add_input("polygons", typeid(LinearRingCollection));
      add_output("point_clouds", typeid(std::vector<PointCollection>));

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
      add_input("point_clouds", typeid(std::vector<PointCollection>));
      add_input("polygons", typeid(LinearRingCollection));
      add_output("point_cloud", typeid(PointCollection));
      add_output("polygon", typeid(LinearRing));

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
      add_input("edge_segments", typeid(SegmentCollection));
      add_input("footprint", typeid(LinearRing));
      add_output("edges_out", typeid(LineStringCollection));
      add_output("merged_edges_out", typeid(LineStringCollection));
      add_output("cluster_labels", typeid(vec1i));
      // add_output("footprint_labels", typeid(vec1i));
      // add_output("line_clusters", TT_any); // ie a LineCluster
      // add_output("tmp_vec3f", typeid(vec3f));
      add_param("dist_threshold", (float) 0.5);
      add_param("angle_threshold", (float) 0.15);
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
      add_input("edge_segments", typeid(SegmentCollection));
      add_input("ring_idx", typeid(std::vector<std::vector<size_t>>));
      // add_input("ring_id", typeid(vec1i));
      // add_input("ring_order", typeid(vec1i));
      // add_input("edge_segments", typeid(SegmentCollection));
      add_input("footprint", typeid(LinearRing));
      add_output("edges_out", typeid(SegmentCollection));
      add_output("priorities", typeid(vec1i));
      // add_output("rings_out", typeid(LinearRingCollection));
      // add_output("footprint_out", typeid(LinearRing));
      add_output("rings_out", typeid(LinearRingCollection));
      add_output("exact_rings_out", typeid(std::vector<linereg::Polygon_2>));
      add_output("exact_footprint_out", typeid(linereg::Polygon_2));
      // add_output("footprint_labels", typeid(vec1i));
      // add_output("line_clusters", TT_any); // ie a LineCluster
      // add_output("tmp_vec3f", typeid(vec3f));
      add_param("dist_threshold", (float) 0.5);
      add_param("angle_threshold", (float) 0.15);
      add_param("snap_threshold", (float) 1.0);
      add_param("weighted_avg", (bool) false);
      add_param("angle_per_distcluster", (bool) false);
      add_param("regularise_fp", (bool) false);
      add_param("fp_offset", (float) 0.01);
    }

    void gui(){
      ImGui::DragFloat("Distance threshold", &param<float>("dist_threshold"), 0.1, 0);
      ImGui::DragFloat("Angle threshold", &param<float>("angle_threshold"), 0.01, 0.01, 3.1415);
      ImGui::DragFloat("Snap threshold", &param<float>("snap_threshold"), 0.01, 0.01, 10);
      ImGui::Checkbox("weighted_avg", &param<bool>("weighted_avg"));
      ImGui::Checkbox("angle_per_distcluster", &param<bool>("angle_per_distcluster"));
      ImGui::Checkbox("regularise_fp", &param<bool>("regularise_fp"));
      ImGui::DragFloat("fp_offset", &param<float>("fp_offset"), 0.01, 0.01, 10);
    }
    void process();
  };


  class PrintResultNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("in", {typeid(float)});
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
      add_input("polygons", {typeid(LinearRingCollection), typeid(LinearRing)});
      add_output("polygons_simp", typeid(LinearRingCollection));
      add_output("polygon_simp", typeid(LinearRing));

      add_param("threshold_stop_cost", (float) 0.005);
    }
    void gui() {
      if(ImGui::DragFloat("stop cost", &param<float>("threshold_stop_cost"),0.01, 0,1000)) {
        manager.run(*this);
      }
    }
    void process();
  };

}