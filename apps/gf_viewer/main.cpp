#include <iostream>
#include <fstream>
#include <array>

#include "imgui.h"
#include "app_povi.h"
#include "nodes.h"

#include <cgal_nodes.hpp>
#include <gdal_nodes.hpp>
#include <las_nodes.hpp>
#include <stepedge_nodes.hpp>
#include <masb_nodes.hpp>

static auto a = std::make_shared<poviApp>(1280, 800, "Geoflow nodes");
static geoflow::NodeManager N;
static ImGui::Nodes nodes_(N, *a);

void on_draw() {
    ImGui::Begin("Nodes");
        nodes_.ProcessNodes();
    ImGui::End();
}

int main(int ac, const char * av[])
{
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
    N.register_node<LASInPolygonsNode>("LASInPolygons");
    // N.register_node<PointsInFootprintNode>("PointsInFootprint");
    N.register_node<RegulariseLinesNode>("RegulariseLines");
    N.register_node<AlphaShapeNode>("AlphaShape");
    N.register_node<Arr2LinearRingsNode>("Arr2LinearRings");
    N.register_node<PolygonExtruderNode>("PolygonExtruder");
    
    //gdal nodes
    N.register_node<OGRLoaderNode>("OGRLoader");
    N.register_node<OGRWriterNode>("OGRWriter");
    N.register_node<LASLoaderNode>("LASLoader");
    N.register_node<CDTNode>("CDT");
    // N.register_node<PointDistanceNode>("PointDistance");
    // N.register_node<ComparePointDistanceNode>("ComparePointDistance");
    N.register_node<CSVLoaderNode>("CSVLoader");
    N.register_node<TinSimpNode>("TinSimp");
    N.register_node<SimplifyLine3DNode>("SimplifyLine3D");

    N.register_node<ComputeNormalsNode>("ComputeNormals");
    N.register_node<ComputeMedialAxisNode>("ComputeMedialAxis");

    a->draw_that(on_draw);

    // ImGui::NodeStore ns;
    // ns.push_back(std::make_tuple("OGRLoader", "TheOGRLoader", ImVec2(-275,75)));
    // ns.push_back(std::make_tuple("LASInPolygons", "TheLASInPolygons", ImVec2(75,75)));
    // ns.push_back(std::make_tuple("ComputeMetrics", "TheComputeMetrics", ImVec2(300,75)));
    // ns.push_back(std::make_tuple("AlphaShape", "TheAlphaShape", ImVec2(600,75)));
    // ns.push_back(std::make_tuple("DetectLines", "TheDetectLines", ImVec2(900,75)));
    // ns.push_back(std::make_tuple("RegulariseLines", "TheRegulariseLines", ImVec2(1200,175)));
    // ns.push_back(std::make_tuple("BuildArrangement", "TheBuildArrangement", ImVec2(1200,75)));
    // ns.push_back(std::make_tuple("ProcessArrangement", "TheProcessArrangement", ImVec2(1500,75)));
    // ns.push_back(std::make_tuple("Extruder", "TheExtruder", ImVec2(1800,75)));
    // nodes_.PreloadNodes(ns);

    // ImGui::LinkStore ls;
    // // ls.push_back(std::make_tuple("TheOGRLoader", "TheLASInPolygons", "linear_rings", "polygons"));
    // ls.push_back(std::make_tuple("TheLASInPolygons", "TheComputeMetrics", "points_vec3f", "points_vec3f"));
    // ls.push_back(std::make_tuple("TheLASInPolygons", "TheBuildArrangement", "footprint_vec3f", "footprint_vec3f"));
    // ls.push_back(std::make_tuple("TheLASInPolygons", "TheRegulariseLines", "footprint_vec3f", "footprint_vec3f"));
    // ls.push_back(std::make_tuple("TheComputeMetrics", "TheAlphaShape", "points", "points"));
    // ls.push_back(std::make_tuple("TheComputeMetrics", "TheProcessArrangement", "points", "points"));
    // ls.push_back(std::make_tuple("TheAlphaShape", "TheDetectLines", "edge_points", "edge_points"));
    // ls.push_back(std::make_tuple("TheDetectLines", "TheRegulariseLines", "edge_segments", "edge_segments"));
    // ls.push_back(std::make_tuple("TheRegulariseLines", "TheBuildArrangement", "edges_out", "edge_segments"));
    // ls.push_back(std::make_tuple("TheBuildArrangement", "TheProcessArrangement", "arrangement", "arrangement"));
    // ls.push_back(std::make_tuple("TheProcessArrangement", "TheExtruder", "arrangement", "arrangement"));
    // nodes_.PreloadLinks(ls);

    a->run();
}