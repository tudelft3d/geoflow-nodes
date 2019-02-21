#include "stepedge_nodes.hpp"

namespace geoflow::nodes::stepedge {

  void create_lod13chart(NodeManager& N, bool direct_alpharing, bool only_classify) {
    NodeRegister R("Nodes");
    R.register_node<AlphaShapeNode>("AlphaShape");
    R.register_node<SimplifyPolygonNode>("SimplifyPolygon");
    R.register_node<DetectPlanesNode>("DetectPlanes");
    R.register_node<DetectLinesNode>("DetectLines");
    R.register_node<RegulariseRingsNode>("RegulariseRings");
    R.register_node<BuildArrFromRingsExactNode>("BuildArrFromRings");
    // R.register_node<BuildArrFromRingsNode>("BuildArrFromRings");
    // R.register_node<ProcessArrangementNode>("ProcessArrangement");
    R.register_node<Arr2LinearRingsNode>("Arr2LinearRings");
    R.register_node<Ring2SegmentsNode>("Ring2Segments");

    auto DetectPlanes_node = N.create_node(R, "DetectPlanes", {300,75});
    N.name_node(DetectPlanes_node, "DetectPlanes_node");
    auto AlphaShape_node = N.create_node(R, "AlphaShape", {600,75});
    N.name_node(AlphaShape_node, "AlphaShape_node");
    auto DetectLines_node = N.create_node(R, "DetectLines", {900,-75});
    N.name_node(DetectLines_node, "DetectLines_node");
    auto RegulariseRings_node = N.create_node(R, "RegulariseRings", {1200,25});
    N.name_node(RegulariseRings_node, "RegulariseRings_node");
    auto BuildArrFromRings_node = N.create_node(R, "BuildArrFromRings", {1550,-125});
    N.name_node(BuildArrFromRings_node, "BuildArrFromRings_node");
    // auto ProcessArrangement_node = N.create_node(R, "ProcessArrangement");
    auto Arr2LinearRings_node = N.create_node(R, "Arr2LinearRings", {1550,225});
    N.name_node(Arr2LinearRings_node, "Arr2LinearRings_node");
    auto SimplifyPolygon_node = N.create_node(R, "SimplifyPolygon", {900,150});
    N.name_node(SimplifyPolygon_node, "SimplifyPolygon_node");
    // auto SimplifyPolygon_node_postfp = N.create_node(R, "SimplifyPolygon", {1200,-125});
    // N.name_node(SimplifyPolygon_node_postfp, "SimplifyPolygon_node_postfp");
    // auto SimplifyPolygon_node_postr = N.create_node(R, "SimplifyPolygon", {1200,175});
    // N.name_node(SimplifyPolygon_node_postr, "SimplifyPolygon_node_postr");
    auto Ring2Segments_node = N.create_node(R, "Ring2Segments", {900,50});
    N.name_node(Ring2Segments_node, "Ring2Segments_node");

    connect(DetectPlanes_node, AlphaShape_node, "pts_per_roofplane", "pts_per_roofplane");
    connect(DetectPlanes_node, BuildArrFromRings_node, "pts_per_roofplane", "pts_per_roofplane");
    
    if (direct_alpharing) {
      SimplifyPolygon_node->set_param("threshold_stop_cost", float(0.18));
      connect(AlphaShape_node, SimplifyPolygon_node, "alpha_rings", "polygons");
      connect(SimplifyPolygon_node, Ring2Segments_node, "polygons_simp", "rings");
      connect(Ring2Segments_node, RegulariseRings_node, "edge_segments", "edge_segments");
      connect(Ring2Segments_node, RegulariseRings_node, "ring_idx", "ring_idx");
      connect(RegulariseRings_node, BuildArrFromRings_node, "exact_rings_out", "rings");
      connect(RegulariseRings_node, BuildArrFromRings_node, "exact_footprint_out", "footprint");
      // connect(RegulariseRings_node, SimplifyPolygon_node_postr, "rings_out", "polygons");
      // connect(RegulariseRings_node, SimplifyPolygon_node_postfp, "footprint_out", "polygons");
      // connect(SimplifyPolygon_node_postr, BuildArrFromRings_node, "polygons_simp", "rings");
      // connect(SimplifyPolygon_node_postfp, BuildArrFromRings_node, "polygon_simp", "footprint");
    } else {
      connect(AlphaShape_node, DetectLines_node, "alpha_rings", "edge_points");
      connect(DetectLines_node, RegulariseRings_node, "edge_segments", "edge_segments");
      connect(DetectLines_node, RegulariseRings_node, "ring_idx", "ring_idx");
      connect(RegulariseRings_node, BuildArrFromRings_node, "rings_out", "rings");
      connect(RegulariseRings_node, SimplifyPolygon_node, "footprint_out", "polygons");
      connect(SimplifyPolygon_node, BuildArrFromRings_node, "polygon_simp", "footprint");
    }
    connect(BuildArrFromRings_node, Arr2LinearRings_node, "arrangement", "arrangement");
  }

  class LOD13GeneratorNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("point_clouds", TT_point_collection_list);
      add_input("polygons", TT_linear_ring_collection);
      add_output("decomposed_footprints", TT_linear_ring_collection);
      add_output("attributes", TT_attribute_map_f);
      add_output("building_class", TT_attribute_map_f);

      add_param("step_height_threshold", (float) 2.0);
      add_param("only_classify", (bool) false);
      add_param("direct_alpharing", (bool) true);
      add_param("z_percentile", (float) 0.9);
      add_param("flood_to_unsegmented", (bool) true);
      add_param("dissolve_edges", (bool) true);
      add_param("dissolve_stepedges", (bool) true);
      // add_param("zrange_threshold", (float) 0.2);
      // add_param("merge_segid", (bool) true);
      // add_param("merge_zrange", (bool) false);
      // add_param("merge_step_height", (bool) true);
      // add_param("merge_unsegmented", (bool) false);
      // add_param("merge_dangling_egdes", (bool) false);
    }

    void gui(){
      ImGui::Checkbox("only_classify", &param<bool>("only_classify"));
      ImGui::Checkbox("direct_alpharing", &param<bool>("direct_alpharing"));
      
      ImGui::SliderFloat("Elevation percentile", &param<float>("z_percentile"), 0, 1);
      ImGui::Checkbox("Flood to unsegmented", &param<bool>("flood_to_unsegmented"));
      ImGui::Checkbox("Dissolve edges", &param<bool>("dissolve_edges"));
      ImGui::Checkbox("Dissolve stepedges", &param<bool>("dissolve_stepedges"));
      ImGui::SliderFloat("step_height_threshold", &param<float>("step_height_threshold"), 0, 100);
    }

    void process(){
      auto point_clouds = input("point_clouds").get<std::vector<PointCollection>>();
      auto polygons = input("polygons").get<LinearRingCollection>();
      
      // for each pair of polygon and point_cloud
        //create nodes and connections
        //run the thing
      if (point_clouds.size()!=polygons.size()) return;

      LinearRingCollection all_cells;
      AttributeMap all_attributes, building_class;
      
      for(int i=0; i<point_clouds.size(); i++) {
        // std::cout << "b id: " << i << "\n";
        auto& points = point_clouds[i];
        auto& polygon = polygons[i];
        
        NodeManager N;
        create_lod13chart(N, param<bool>("direct_alpharing"), param<bool>("only_classify"));

        // config and run
        // this should copy all parameters from this LOD13Generator node to the ProcessArrangement node
        N.nodes["BuildArrFromRings_node"]->set_params( dump_params() );

        N.nodes["DetectPlanes_node"]->input("points").set(points);
        N.nodes["RegulariseRings_node"]->input("footprint").set(polygon);
        // N.nodes["SimplifyPolygon_node_postfp"]->input("polygons").set(polygon);

        N.run(N.nodes["DetectPlanes_node"]);

        auto classf = N.nodes["DetectPlanes_node"]->output("classf").get<float>();
        auto horiz = N.nodes["DetectPlanes_node"]->output("horiz_roofplane_cnt").get<float>();
        auto slant = N.nodes["DetectPlanes_node"]->output("slant_roofplane_cnt").get<float>();
        building_class["bclass"].push_back(classf);
        building_class["horiz"].push_back(horiz);
        building_class["slant"].push_back(slant);
        // note: the following will crash if the flowchart specified above is stopped halfway for some reason (eg missing output/connection)

        auto cells = N.nodes["Arr2LinearRings_node"]->output("linear_rings").get<LinearRingCollection>();
        auto attributes = N.nodes["Arr2LinearRings_node"]->output("attributes").get<AttributeMap>();

        for (int i=0; i<cells.size(); i++) {
          // if(polygons_feature.attr["height"][i]!=0) { //FIXME this is a hack!!
          all_cells.push_back(cells[i]);
          all_attributes["height"].push_back(attributes["height"][i]);
          all_attributes["rms_error"].push_back(attributes["rms_error"][i]);
          all_attributes["max_error"].push_back(attributes["max_error"][i]);
          all_attributes["count"].push_back(attributes["count"][i]);
          all_attributes["coverage"].push_back(attributes["coverage"][i]);
          all_attributes["bclass"].push_back(classf);
        }
      }
      output("decomposed_footprints").set(all_cells);
      output("attributes").set(all_attributes);
      output("building_class").set(building_class);
    }
  };

  class LOD10GeneratorNode:public Node {
    public:
    using Node::Node;
    void init() {
      add_input("point_clouds", TT_point_collection_list);
      add_output("attributes", TT_attribute_map_f);

      add_param("z_percentile", (float) 0.9);
    }

    void gui(){
      ImGui::SliderFloat("Elevation percentile", &param<float>("z_percentile"), 0, 1);
    }

    void process(){
      auto point_clouds = input("point_clouds").get<std::vector<PointCollection>>();

      AttributeMap all_attributes;

      NodeRegister R("Nodes");
      R.register_node<DetectPlanesNode>("DetectPlanes");
      
      for(int i=0; i<point_clouds.size(); i++) {
        // std::cout << "b id: " << i << "\n";
        auto& point_cloud = point_clouds[i];
        
        NodeManager N;
        auto detect_planes_node = N.create_node(R, "DetectPlanes");
        detect_planes_node->input("points").set(point_cloud);

        N.run(detect_planes_node);

        auto classf = detect_planes_node->output("classf").get<float>();
        auto horiz = detect_planes_node->output("horiz_roofplane_cnt").get<float>();
        auto slant = detect_planes_node->output("slant_roofplane_cnt").get<float>();
        all_attributes["bclass"].push_back(classf);
        // all_attributes["horiz"].push_back(horiz);
        // all_attributes["slant"].push_back(slant);
        // note: the following will crash if the flowchart specified above is stopped halfway for some reason (eg missing output/connection)

        auto roofplanepts_per_fp = detect_planes_node->output("pts_per_roofplane").get<std::unordered_map<int, std::vector<Point>>>();

        std::vector<Point> points;
        for (auto& kv : roofplanepts_per_fp) {
          points.insert(points.end(), kv.second.begin(), kv.second.end());
        }
        if(points.size() == 0) {
          all_attributes["height"].push_back(0);
          all_attributes["rms_error"].push_back(0);
        } else {
          // if(polygons_feature.attr["height"][i]!=0) { //FIXME this is a hack!!
          std::sort(points.begin(), points.end(), [](linedect::Point& p1, linedect::Point& p2) {
            return p1.z() < p2.z();
          });
          auto elevation_id = int(param<float>("z_percentile")*float(points.size()));
          // std::cout << "id: " << elevation_id << ", size: " << points.size() << "\n";
          double elevation = points[elevation_id].z();
          double square_sum = 0;
          for (auto& p : points) {
            float d = elevation - p.z();
            square_sum += d*d;
          }
          all_attributes["rms_error"].push_back(CGAL::sqrt(square_sum/points.size()));
          all_attributes["height"].push_back(elevation);
        }

      }
      output("attributes").set(all_attributes);
    }
  };

  NodeRegister create_register() {
    auto R = NodeRegister("Step edge");
    R.register_node<AlphaShapeNode>("AlphaShape");
    R.register_node<PolygonExtruderNode>("PolygonExtruder");
    R.register_node<Arr2LinearRingsNode>("Arr2LinearRings");
    R.register_node<ExtruderNode>("Extruder");
    R.register_node<ProcessArrangementNode>("ProcessArrangement");
    R.register_node<LinearRingtoRingsNode>("LinearRingtoRings");
    R.register_node<BuildArrangementNode>("BuildArrangement");
    R.register_node<BuildArrFromRingsNode>("BuildArrFromRings");
    R.register_node<BuildArrFromRingsExactNode>("BuildArrFromRingsExact");
    R.register_node<DetectLinesNode>("DetectLines");
    R.register_node<DetectPlanesNode>("DetectPlanes");
    R.register_node<ClassifyEdgePointsNode>("ClassifyEdgePoints");
    R.register_node<ComputeMetricsNode>("ComputeMetrics");
    R.register_node<LASInPolygonsNode>("LASInPolygons");
    R.register_node<BuildingSelectorNode>("BuildingSelector");
    R.register_node<RegulariseLinesNode>("RegulariseLines");
    R.register_node<RegulariseRingsNode>("RegulariseRings");
    R.register_node<SimplifyPolygonNode>("SimplifyPolygon");
    R.register_node<LOD10GeneratorNode>("LOD10Generator");
    R.register_node<LOD13GeneratorNode>("LOD13Generator");
    R.register_node<Ring2SegmentsNode>("Ring2Segments");
    R.register_node<PrintResultNode>("PrintResult");
    return R;
  }

}