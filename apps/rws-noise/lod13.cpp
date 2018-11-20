#include <iostream>
#include <fstream>

#include "imgui.h"
#include "app_povi.h"
#include "nodes.h"
#include <stepedge_nodes.hpp>
#include <gdal_nodes.hpp>
#include <las_nodes.hpp>
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