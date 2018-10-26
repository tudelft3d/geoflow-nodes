#include <iostream>
#include <fstream>

#include "imgui.h"
#include "app_povi.h"
#include "nodes.h"
#include "../src/stepedge_nodes.hpp"
#include "../src/gdal_nodes.hpp"
#include "../src/las_nodes.hpp"
#include "../src/cgal_nodes.hpp"
#include <array>

class LOD13GeneratorNode:public Node {
  float step_threshold = 1.0;

  public:
  LOD13GeneratorNode(NodeManager& manager):Node(manager, "LOD13Generator") {
    add_input("point_clouds", TT_any);
    add_input("polygons", TT_any);
    add_output("decomposed_polygons", TT_any);
  }

  void gui(){
    ImGui::InputFloat("Step height", &step_threshold, 0.1, 1);
  }

  void process(){
    auto point_clouds = std::any_cast<Feature>(get_value("point_clouds"));
    auto polygons = std::any_cast<Feature>(get_value("polygons"));
    
    // for each pair of polygon and point_cloud
      //create nodes and connections
      //run the thing
    if (point_clouds.geom.size()!=polygons.geom.size()) return;

    Feature decomposed_polygons;
    decomposed_polygons.type=geoflow::line_loop;
    
    for(int i=0; i<point_clouds.geom.size(); i++) {
      auto& points_vec3f = point_clouds.geom[i];
      auto& polygon_vec3f = polygons.geom[i];

      NodeManager N = NodeManager();

      auto ComputeMetrics_node = std::make_shared<ComputeMetricsNode>(N);
      auto AlphaShape_node = std::make_shared<AlphaShapeNode>(N);
      auto DetectLines_node = std::make_shared<DetectLinesNode>(N);
      auto RegulariseLines_node = std::make_shared<RegulariseLinesNode>(N);
      auto BuildArrangement_node = std::make_shared<BuildArrangementNode>(N);
      auto ProcessArrangement_node = std::make_shared<ProcessArrangementNode>(N);
      auto Arr2Feature_node = std::make_shared<Arr2FeatureNode>(N);

      ComputeMetrics_node->inputTerminals["points_vec3f"]->push(points_vec3f);
      BuildArrangement_node->inputTerminals["footprint_vec3f"]->push(polygon_vec3f);
      RegulariseLines_node->inputTerminals["footprint_vec3f"]->push(polygon_vec3f);

      connect(*ComputeMetrics_node, *AlphaShape_node, "points", "points");
      connect(*ComputeMetrics_node, *ProcessArrangement_node, "points", "points");
      connect(*AlphaShape_node, *DetectLines_node, "edge_points", "edge_points");
      connect(*DetectLines_node, *RegulariseLines_node, "edge_segments", "edge_segments");
      connect(*RegulariseLines_node, *BuildArrangement_node, "edges_out", "edge_segments");
      connect(*BuildArrangement_node, *ProcessArrangement_node, "arrangement", "arrangement");
      connect(*ProcessArrangement_node, *Arr2Feature_node, "arrangement", "arrangement");

      // config and run
      ProcessArrangement_node->c.step_height_threshold = step_threshold;
      N.run(*ComputeMetrics_node);

      auto oTerm = Arr2Feature_node->outputTerminals["decomposed_footprint"];
      auto polygons_feature = std::any_cast<Feature>(oTerm->cdata);

      for (int i=0; i<polygons_feature.geom.size(); i++) {
        // if(polygons_feature.attr["height"][i]!=0) { //FIXME this is a hack!!
          decomposed_polygons.geom.push_back(polygons_feature.geom[i]);
          decomposed_polygons.attr["height"].push_back(polygons_feature.attr["height"][i]);
        // }
      }
    }
    set_value("decomposed_polygons", decomposed_polygons);
  }
};

// #include <boost/program_options.hpp>

static auto a = std::make_shared<poviApp>(1280, 800, "Step edge detector");
static geoflow::NodeManager N;
static ImGui::Nodes nodes_(N, *a);

void on_draw() {
    ImGui::Begin("Nodes");
        nodes_.ProcessNodes();
    ImGui::End();
}

int main(int ac, const char * av[])
{
    //‎⁨ /Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_gebouwen/bag.gpkg
    //‎⁨ /Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_puntenwolk/extend.las
    // /Users/ravi/surfdrive/Data/step-edge-detector/C_31HZ1_clip.LAZ
    // /Users/ravi/surfdrive/Data/step-edge-detector/ahn3.las
    // /Users/ravi/surfdrive/Data/step-edge-detector/rdam_sample_0.gpkg
    //viewer nodes
    N.register_node<TriangleNode>("Triangle");
    N.register_node<ColorMapperNode>("ColorMapper");
    N.register_node<Vec3SplitterNode>("Vec3Splitter");
    N.register_node<GradientMapperNode>("GradientMapper");
    
    //gdal nodes
    N.register_node<OGRLoaderNode>("OGRLoader");
    N.register_node<OGRWriterNode>("OGRWriter");

    N.register_node<LASLoaderNode>("LASLoader");
    N.register_node<CDTNode>("CDT");
    N.register_node<PointDistanceNode>("PointDistance");
    N.register_node<ComparePointDistanceNode>("ComparePointDistance");
    N.register_node<CSVLoaderNode>("CSVLoader");
    N.register_node<TinSimpNode>("TinSimp");
    
    N.register_node<LASInPolygonsNode>("LASInPolygons");
    N.register_node<LOD13GeneratorNode>("LOD13Generator");
    
    N.register_node<PolygonExtruderNode>("PolygonExtruder");

    a->draw_that(on_draw);

    ImGui::NodeStore ns;
    ns.push_back(std::make_tuple("OGRLoader", "TheOGRLoader", ImVec2(75,75)));
    ns.push_back(std::make_tuple("LASInPolygons", "TheLASInPolygons", ImVec2(300,75)));
    ns.push_back(std::make_tuple("LOD13Generator", "TheLOD13Generator", ImVec2(650,75)));
    ns.push_back(std::make_tuple("OGRWriter", "TheOGRWriter", ImVec2(1000,75)));
    nodes_.PreloadNodes(ns);

    ImGui::LinkStore ls;
    ls.push_back(std::make_tuple("TheOGRLoader", "TheLASInPolygons", "features", "polygons"));
    ls.push_back(std::make_tuple("TheOGRLoader", "TheLOD13Generator", "features", "polygons"));
    ls.push_back(std::make_tuple("TheLASInPolygons", "TheLOD13Generator", "point_clouds", "point_clouds"));
    ls.push_back(std::make_tuple("TheLOD13Generator", "TheOGRWriter", "decomposed_polygons", "features"));
    nodes_.PreloadLinks(ls);

    a->run();
}