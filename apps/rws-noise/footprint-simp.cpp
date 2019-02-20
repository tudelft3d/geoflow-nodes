#include <iostream>
#include <fstream>

#include "imgui.h"
#include "app_povi.h"
#include "nodes.h"
#include <stepedge_nodes.hpp>
#include <gdal_nodes.hpp>
#include <las_nodes.hpp>
#include <cgal_nodes.hpp>
#include <array>

// #include <boost/program_options.hpp>

static auto a = std::make_shared<poviApp>(1280, 800, "Step edge detector");
static geoflow::NodeManager N;
static ImGui::Nodes nodes_(N, *a);

void on_draw() {
  ImGui::Begin("Nodes");
  nodes_.ProcessNodes();
  ImGui::End();
}

int main(int ac, const char * av[]) {
  //viewer nodes
  N.register_node<TriangleNode>("Triangle");
  N.register_node<ColorMapperNode>("ColorMapper");
  N.register_node<Vec3SplitterNode>("Vec3Splitter");
  N.register_node<GradientMapperNode>("GradientMapper");

  //processing nodes
  N.register_node<SimplifyFootprintNode>("SimplifyFootprint");
  N.register_node<SimplifyLinesNode>("SimplifyLines");
  N.register_node<SimplifyLineNode>("SimplifyLine");
  N.register_node<ExtruderNode>("Extruder");
  N.register_node<ProcessArrangementNode>("ProcessArrangement");
  N.register_node<BuildArrangementNode>("BuildArrangement");
  N.register_node<DetectLinesNode>("DetectLines");
  N.register_node<ClassifyEdgePointsNode>("ClassifyEdgePoints");
  N.register_node<ComputeMetricsNode>("ComputeMetrics");
  //N.register_node<PointsInFootprintNode>("PointsInFootprint");
  N.register_node<RegulariseLinesNode>("RegulariseLines");
  N.register_node<AlphaShapeNode>("AlphaShape");

  //gdal nodes
  N.register_node<OGRLoaderNode>("OGRLoader");
  N.register_node<OGRWriterNode>("OGRWriter");

  N.register_node<LASLoaderNode>("LASLoader");
  N.register_node<CDTNode>("CDT");
  N.register_node<PointDistanceNode>("PointDistance");
  N.register_node<ComparePointDistanceNode>("ComparePointDistance");
  N.register_node<CSVLoaderNode>("CSVLoader");
  N.register_node<TinSimpNode>("TinSimp");

  a->draw_that(on_draw);

  ImGui::NodeStore ns;
  ns.push_back(std::make_tuple("OGRLoader", "TheOGRLoader", ImVec2(75, 75)));
  ns.push_back(std::make_tuple("SimplifyFootprint", "TheSimplifyFootprint", ImVec2(375, 75)));
  ns.push_back(std::make_tuple("OGRWriter", "TheOGRWriter", ImVec2(775, 75)));
  nodes_.PreloadNodes(ns);

  // ImGui::LinkStore ls;
  // ls.push_back(std::make_tuple("ThePointsInFootprint", "TheComputeMetrics", "points", "points"));
  // ls.push_back(std::make_tuple("ThePointsInFootprint", "TheBuildArrangement", "footprint", "footprint"));
  // ls.push_back(std::make_tuple("ThePointsInFootprint", "TheRegulariseLines", "footprint_vec3f", "footprint_vec3f"));
  // ls.push_back(std::make_tuple("TheComputeMetrics", "TheClassifyEdgePoints", "points", "points"));
  // ls.push_back(std::make_tuple("TheComputeMetrics", "TheProcessArrangement", "points", "points"));
  // ls.push_back(std::make_tuple("TheClassifyEdgePoints", "TheDetectLines", "edge_points", "edge_points"));
  // ls.push_back(std::make_tuple("TheDetectLines", "TheRegulariseLines", "edge_segments", "edge_segments"));
  // ls.push_back(std::make_tuple("TheRegulariseLines", "TheBuildArrangement", "edges_out", "edge_segments"));
  // ls.push_back(std::make_tuple("TheBuildArrangement", "TheProcessArrangement", "arrangement", "arrangement"));
  // ls.push_back(std::make_tuple("TheProcessArrangement", "TheExtruder", "arrangement", "arrangement"));
  // nodes_.PreloadLinks(ls);

  a->run();
}