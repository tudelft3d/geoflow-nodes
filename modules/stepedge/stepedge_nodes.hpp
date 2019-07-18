#pragma once

#include <geoflow/geoflow.hpp>

#include "point_edge.h"
#include "line_regulariser.hpp"

namespace geoflow::nodes::stepedge {

  class AlphaShapeNode:public Node {
    float thres_alpha = 0.15;
    bool optimal_alpha = true;
    bool optimal_only_if_needed = true;
    public:
    using Node::Node;
    void init() {
      // add_input("points", TT_any);
      add_input("pts_per_roofplane", typeid(std::unordered_map<int, std::pair<Plane, std::vector<Point>>> ));
      add_output("alpha_rings", typeid(LinearRingCollection));
      add_output("edge_points", typeid(PointCollection));
      add_output("alpha_edges", typeid(LineStringCollection));
      add_output("alpha_triangles", typeid(TriangleCollection));
      add_output("segment_ids", typeid(vec1i));
      add_output("boundary_points", typeid(PointCollection));
      add_output("roofplane_ids", typeid(vec1i));

      add_param("thres_alpha", ParamFloat(thres_alpha, "thres_alpha"));
      add_param("optimal_alpha", ParamBool(optimal_alpha, "optimal_alpha"));
      add_param("optimal_only_if_needed", ParamBool(optimal_only_if_needed, "optimal_only_if_needed"));
    }

    void before_gui(){
      auto param = std::get<ParamBool>(parameters.at("optimal_only_if_needed"));
      param.set_visible(optimal_alpha);
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
    bool only_in_footprint = true;
    public:
    using Node::Node;
    void init() {
      add_input("arrangement", typeid(Arrangement_2));
      add_output("linear_rings", typeid(LinearRingCollection));
      add_output("attributes", typeid(AttributeMap));
      add_param("only_in_footprint", ParamBool(only_in_footprint, "Only in footprint"));
    }
    void process();
  };

  class ExtruderNode:public Node {
    bool do_walls=true, do_roofs=true;
    bool in_footprint = false;
    bool LoD2 = false;
    float base_elevation = 0;
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

      add_param("do_walls", ParamBool(do_walls, "Do walls"));
      add_param("do_roofs", ParamBool(do_roofs, "Do roofs"));
      add_param("in_footprint", ParamBool(in_footprint, "in_footprint"));
      add_param("LoD2", ParamBool(LoD2, "LoD2"));
      add_param("base_elevation", ParamFloat(base_elevation, "Base elevation"));
    }
    void process();
  };

  class PolygonGrowerNode:public Node {
    float extension = 0.1;
    public:
    using Node::Node;
    void init() {
      add_input("rings", typeid(LinearRingCollection));
      add_output("rings", typeid(LinearRingCollection));

      add_param("extension", ParamFloat(extension, "Extension length"));
    }
    void process();
  };

  class ProcessArrangementNode:public Node {
    float step_height_threshold = 1.0;
    float zrange_threshold = 0.2;
    bool merge_segid = true;
    bool merge_zrange = false;
    bool merge_step_height = true;
    bool merge_unsegmented = false;
    bool merge_dangling_egdes = false;
    public:
    using Node::Node;
    void init() {
      add_input("arrangement", typeid(Arrangement_2));
      add_input("points", typeid(PNL_vector));
      add_output("arrangement", typeid(Arrangement_2));

      add_param("step_height_threshold", ParamFloat(step_height_threshold, "step_height_threshold"));
      add_param("zrange_threshold", ParamFloat(zrange_threshold, "zrange_threshold"));
      add_param("merge_segid", ParamBool(merge_segid, "merge_segid"));
      add_param("merge_zrange", ParamBool(merge_zrange, "merge_zrange"));
      add_param("merge_step_height", ParamBool(merge_step_height, "merge_step_height"));
      add_param("merge_unsegmented", ParamBool(merge_unsegmented, "merge_unsegmented"));
      add_param("merge_dangling_egdes", ParamBool(merge_dangling_egdes, "merge_dangling_egdes"));
    }
    void process();
  };

  class BuildArrangementNode:public Node {
    bool remove_unsupported=false;
    public:
    using Node::Node;
    void init() {
      add_input("edge_segments", {typeid(LineStringCollection), typeid(LinearRingCollection)});
      add_input("footprint", typeid(LinearRing));
      add_output("arrangement", typeid(Arrangement_2));
      add_output("arr_segments", typeid(LineStringCollection));

      add_param("remove_unsupported", ParamBool(remove_unsupported, "Remove unsupported edges"));
    }
    void process();
  };

  class BuildArrFromRingsExactNode:public Node {
    bool extrude_unsegmented = true;
    float extrude_mindensity = 5;
    float z_percentile = 0.9;
    float rel_area_thres = 0.1;
    bool flood_to_unsegmented = true;
    bool dissolve_edges = true;
    bool dissolve_stepedges = true;
    float step_height_threshold = 1.0;
    bool snap_clean = true;
    bool snap_clean_fp = false;
    bool snap_detect_only = false;
    float snap_dist = 1.0;

    public:
    bool arr_is_valid=false;
    int vcount, ecount;

    using Node::Node;
    void init() {
      add_input("rings", typeid(std::unordered_map<size_t, linereg::Polygon_2>));
      add_input("pts_per_roofplane", typeid(std::unordered_map<int, std::pair<Plane, std::vector<Point>>> ));
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

      add_param("extrude_unsegmented", ParamBool(extrude_unsegmented, "extrude_unsegmented"));
      add_param("extrude_mindensity", ParamBoundedFloat(extrude_mindensity, 1, 20, "Extrude min density"));
      add_param("z_percentile", ParamBoundedFloat(z_percentile, 0, 1, "Elevation percentile"));
      add_param("rel_area_thres", ParamBoundedFloat(rel_area_thres, 0.01, 1, "Preserve split ring area"));
      add_param("flood_to_unsegmented", ParamBool(flood_to_unsegmented, "flood_to_unsegmented"));
      add_param("dissolve_edges", ParamBool(dissolve_edges, "dissolve_edges"));
      add_param("dissolve_stepedges", ParamBool(dissolve_stepedges, "dissolve_stepedges"));
      add_param("step_height_threshold", ParamBoundedFloat(step_height_threshold, 0, 100, "step_height_threshold"));
      add_param("snap_clean", ParamBool(snap_clean, "Snap"));
      add_param("snap_clean_fp", ParamBool(snap_clean_fp, "Snap fp"));
      add_param("snap_detect_only", ParamBool(snap_detect_only, "snap_detect_only"));
      add_param("snap_dist", ParamBoundedFloat(snap_dist, 0.01, 5, "Snap distance"));
    }
    // std::string info() {
    //   ImGui::Text("Arrangement valid? %s", arr_is_valid? "yes" : "no");
    //   ImGui::Text("vcount: %d, ecount: %d", vcount, ecount);
    // }
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
    bool linear_knn = false;
    float dist_thres = 0.4;
    std::pair<int,int> min_cnt_range = {5,10};
    int k = 10;
    float snap_threshold = 1;
    float line_extend = 0.05;
    bool perform_chaining = true;
    bool remove_overlap = true;
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
      add_input("roofplane_ids", typeid(vec1i));
      add_output("edge_segments", typeid(SegmentCollection));
      add_output("lines3d", typeid(SegmentCollection));
      add_output("ring_edges", typeid(SegmentCollection));
      add_output("ring_idx", typeid(std::unordered_map<size_t,std::vector<size_t>>));
      add_output("ring_id", typeid(vec1i));
      add_output("ring_order", typeid(vec1i));
      add_output("is_start", typeid(vec1i));

      add_param("linear_knn", ParamBool(linear_knn, "Use linear neighbourhood for ring input"));
      add_param("dist_thres", ParamFloat(dist_thres, "dist_thres"));
      add_param("min_cnt_range", ParamIntRange(min_cnt_range, "Minimum segment count"));
      add_param("k", ParamInt(k, "k"));
      add_param("snap_threshold", ParamFloat(snap_threshold, "Chain snap thres"));
      add_param("line_extend", ParamFloat(line_extend, "Extend lines"));
      add_param("perform_chaining", ParamBool(perform_chaining, "Perform chaining"));
      add_param("remove_overlap", ParamBool(remove_overlap, "Remove overlap"));
    }
    inline void detect_lines_ring_m1(linedect::LineDetector& LD, SegmentCollection& segments_out);
    inline size_t detect_lines_ring_m2(linedect::LineDetector& LD, SegmentCollection& segments_out);
    inline void detect_lines(linedect::LineDetector& LD);
    void process();
  };

  // class ClassifyEdgePointsNode:public Node {
  //   public:
  //   using Node::Node;
  //   void init() {
  //     add_output("edge_points", typeid(std::vector<linedect::Point>));
  //     add_output("edge_points_vec3f", typeid(vec3f));
  //     add_input("points", typeid(PNL_vector));

  //     add_param("classify_jump_count_min", (int) 1);
  //     add_param("classify_jump_count_max", (int) 5);
  //     add_param("classify_line_dist", (float) 0.005);
  //     add_param("classify_jump_ele", (float) 1.0);
  //   }

  //   void gui(){
  //     ImGui::InputInt("Jump cnt min", &param<int>("classify_jump_count_min"));
  //     ImGui::InputInt("Jump cnt max", &param<int>("classify_jump_count_max"));
  //     ImGui::InputFloat("Line dist", &param<float>("classify_line_dist"), 0.01, 1);
  //     ImGui::InputFloat("Elevation jump", &param<float>("classify_jump_ele"), 0.01, 1);
  //   }

  //   void process();
  // };

  class DetectPlanesNode:public Node {
    bool only_horizontal = true;
    float horiz_min_count = 0.95;
    int metrics_normal_k = 10;
    int metrics_plane_min_points = 20;
    float metrics_plane_epsilon = 0.2;
    float metrics_plane_normal_threshold = 0.75;
    float metrics_is_horizontal_threshold = 0.97;
    float metrics_is_wall_threshold = 0.3;
    int n_refit = 5;
    public:
    using Node::Node;
    void init() {
      add_input("points", typeid(PointCollection));
      add_output("plane_id", typeid(vec1i));
      add_output("is_wall", typeid(vec1i));
      add_output("is_horizontal", typeid(vec1i));
      
      add_output("pts_per_roofplane", typeid(std::unordered_map<int, std::pair<Plane, std::vector<Point>>> ));

      add_output("roof_pt_cnt", typeid(int));
      add_output("class", typeid(int));
      add_output("classf", typeid(float));
      add_output("horiz_roofplane_cnt", typeid(float));
      add_output("slant_roofplane_cnt", typeid(float));

      add_param("only_horizontal", ParamBool(only_horizontal, "Output only horizontal planes"));
      add_param("horiz_min_count", ParamFloat(horiz_min_count, "Min horiz point count"));
      add_param("metrics_normal_k", ParamInt(metrics_normal_k, "K estimate normal"));
      add_param("metrics_plane_min_points", ParamInt(metrics_plane_min_points, "Plane min points"));
      add_param("metrics_plane_epsilon", ParamFloat(metrics_plane_epsilon, "Plane epsilon"));
      add_param("metrics_plane_normal_threshold", ParamFloat(metrics_plane_normal_threshold, "Plane normal thres"));
      add_param("metrics_is_horizontal_threshold", ParamFloat(metrics_is_horizontal_threshold, "Is horizontal"));
      add_param("metrics_is_wall_threshold", ParamFloat(metrics_is_wall_threshold, "Wall angle thres"));
      add_param("n_refit", ParamInt(n_refit, "Refit every n points"));
    }
    void before_gui(){
      auto param_count = std::get<ParamFloat>(parameters.at("horiz_min_count"));
      auto param_ishoriz = std::get<ParamFloat>(parameters.at("metrics_is_horizontal_threshold"));
      param_count.set_visible(only_horizontal);
      param_ishoriz.set_visible(only_horizontal);
    }
    void process();
  };

  class ComputeMetricsNode:public Node {
    int metrics_normal_k = 10;
    int metrics_plane_min_points = 50;
    float metrics_plane_epsilon = 0.15;
    float metrics_plane_normal_threshold = 0.75;
    float metrics_is_horizontal_threshold = 0.9;
    float metrics_is_wall_threshold = 0.3;
    int metrics_k_linefit = 15;
    int metrics_k_jumpcnt_elediff = 10;
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

      add_param("metrics_normal_k", ParamInt(metrics_normal_k, "K estimate normal"));
      add_param("metrics_plane_min_points", ParamInt(metrics_plane_min_points, "Plane min points"));
      add_param("metrics_plane_epsilon", ParamFloat(metrics_plane_epsilon, "Plane epsilon"));
      add_param("metrics_plane_normal_threshold", ParamFloat(metrics_plane_normal_threshold, "Plane normal thres"));
      add_param("metrics_is_horizontal_threshold", ParamFloat(metrics_is_horizontal_threshold, "Is horizontal"));
      add_param("metrics_is_wall_threshold", ParamFloat(metrics_is_wall_threshold, "Wall angle thres"));
      add_param("metrics_k_linefit", ParamInt(metrics_k_linefit, "K linefit"));
      add_param("metrics_k_jumpcnt_elediff", ParamInt(metrics_k_jumpcnt_elediff, "K jumpedge"));
    }
    void process();
  };

  class LASInPolygonsNode:public Node {
    std::string filepath = "";
    public:
    using Node::Node;
    void init() {
      add_input("polygons", typeid(LinearRingCollection));
      add_output("point_clouds", typeid(std::vector<PointCollection>));

      add_param("las_filepath", ParamPath(filepath, "LAS filepath"));
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

      add_param("building_id", ParamBoundedInt(building_id, 0, polygon_count-1, "building_id"));
    }
    void on_connect_input(InputTerminal& it) {
      if (it.get_name() == "polygons") {
        auto& polygons = input("polygons").get<LinearRingCollection&>();
        polygon_count = polygons.size();
        auto param = std::get<ParamBoundedInt>(parameters.at("building_id"));
        param.set_bounds(0, polygon_count-1);
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
    float dist_threshold = 0.5;
    float angle_threshold = 0.15;

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
      add_param("dist_threshold", ParamFloat(dist_threshold, "Distance threshold"));
      add_param("angle_threshold", ParamBoundedFloat(angle_threshold, 0.01, pi, "Angle threshold"));
    }
    void process();
  };

  class RegulariseRingsNode:public Node {
    float dist_threshold = 0.5;
    float angle_threshold = 0.15;
    float snap_threshold = 1.0;
    bool weighted_avg = false;
    bool angle_per_distcluster = false;
    bool regularise_fp = false;
    float fp_offset = 0.01;

    public:
    using Node::Node;
    void init() {
      add_input("edge_segments", typeid(SegmentCollection));
      add_input("ring_idx", typeid(std::unordered_map<size_t,std::vector<size_t>>));
      // add_input("ring_id", typeid(vec1i));
      // add_input("ring_order", typeid(vec1i));
      // add_input("edge_segments", typeid(SegmentCollection));
      add_input("footprint", typeid(LinearRing));
      add_output("edges_out", typeid(SegmentCollection));
      add_output("priorities", typeid(vec1i));
      // add_output("rings_out", typeid(LinearRingCollection));
      // add_output("footprint_out", typeid(LinearRing));
      add_output("rings_out", typeid(LinearRingCollection));
      add_output("plane_id", typeid(vec1i));
      add_output("exact_rings_out", typeid(std::unordered_map<size_t, linereg::Polygon_2>));
      add_output("exact_footprint_out", typeid(linereg::Polygon_2));
      // add_output("footprint_labels", typeid(vec1i));
      // add_output("line_clusters", TT_any); // ie a LineCluster
      // add_output("tmp_vec3f", typeid(vec3f));
      add_param("dist_threshold", ParamFloat(dist_threshold, "Distance threshold"));
      add_param("angle_threshold", ParamBoundedFloat(angle_threshold, 0.01, 3.1415, "Angle threshold"));
      add_param("snap_threshold", ParamBoundedFloat(snap_threshold, 0.01, 10, "Snap threshold"));
      add_param("weighted_avg", ParamBool(weighted_avg, "weighted_avg"));
      add_param("angle_per_distcluster", ParamBool(angle_per_distcluster, "angle_per_distcluster"));
      add_param("regularise_fp", ParamBool(regularise_fp, "regularise_fp"));
      add_param("fp_offset", ParamBoundedFloat(fp_offset, 0.01, 10, "fp_offset"));
    }
    void process();
  };


  class PrintResultNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("in", {typeid(float)});
    }
    std::string info() {
      return "Result: " + std::to_string(input("in").get<float>());
    }
    void process(){};
  };


  class SimplifyPolygonNode:public Node {
    float threshold_stop_cost = 0.005;
    public:
    using Node::Node;
    void init() {
      add_input("polygons", {typeid(LinearRingCollection), typeid(LinearRing)});
      add_output("polygons_simp", typeid(LinearRingCollection));
      add_output("polygon_simp", typeid(LinearRing));

      add_param("threshold_stop_cost", ParamBoundedFloat(threshold_stop_cost, 0, 1000, "threshold_stop_cost"));
    }
    void on_change_parameter(std::string name, ParameterVariant& param){
      if (name == "threshold_stop_cost") {
        manager.run(*this);
      }
    }
    void process();
  };

}