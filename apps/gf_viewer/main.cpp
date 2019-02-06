#include <iostream>
#include <fstream>
#include <array>

#include "imgui.h"
#include <geoflow/gui/flowchart.hpp>

#include <geoflow/core/geoflow.hpp>
#include <cgal_register.hpp>
#include <gdal_register.hpp>
#include <las_register.hpp>
#include <stepedge_register.hpp>
#include <masb_register.hpp>

namespace gfn = geoflow::nodes;

int main(int ac, const char * av[])
{
    // register nodes from various modules
    NodeRegister stepedge = gfn::stepedge::create_register();
    NodeRegister cgal = gfn::cgal::create_register();
    NodeRegister gdal = gfn::gdal::create_register();
    NodeRegister las = gfn::las::create_register();
    NodeRegister mat = gfn::mat::create_register();

    // create some nodes and connections
    NodeManager N;

    NodeHandle OGRLoader = N.create_node(gdal, "OGRLoader", {-275,75});
    NodeHandle LASInPolygons = N.create_node(stepedge, "LASInPolygons", {75,75});
    NodeHandle ComputeMetrics = N.create_node(stepedge, "ComputeMetrics", {300,75});
    NodeHandle AlphaShape = N.create_node(stepedge, "AlphaShape", {600,75});
    NodeHandle DetectLines = N.create_node(stepedge, "DetectLines", {900,75});
    NodeHandle RegulariseLines = N.create_node(stepedge, "RegulariseLines", {1200,175});
    NodeHandle BuildArrangement = N.create_node(stepedge, "BuildArrangement", {1200,75});
    NodeHandle ProcessArrangement = N.create_node(stepedge, "ProcessArrangement", {1500,75});
    NodeHandle Extruder = N.create_node(stepedge, "Extruder", {1800,75});

    connect(LASInPolygons, ComputeMetrics, "points", "points");
    connect(LASInPolygons, BuildArrangement, "footprint", "footprint");
    connect(LASInPolygons, RegulariseLines, "footprint", "footprint");
    connect(ComputeMetrics, AlphaShape, "points", "points");
    connect(ComputeMetrics, ProcessArrangement, "points", "points");
    connect(AlphaShape, DetectLines, "edge_points", "edge_points");
    connect(DetectLines, RegulariseLines, "edge_segments", "edge_segments");
    connect(RegulariseLines, BuildArrangement, "edges_out", "edge_segments");
    connect(BuildArrangement, ProcessArrangement, "arrangement", "arrangement");
    connect(ProcessArrangement, Extruder, "arrangement", "arrangement");

    LASInPolygons->set_param(
        "las_filepath", (std::string)"/Users/ravi/surfdrive/Data/step-edge-detector/ahn3.las");
    OGRLoader->set_param(
        "filepath", (std::string) "/Users/ravi/surfdrive/Data/step-edge-detector/rdam_sample_0.gpkg");

    // launch the GUI
    launch_flowchart(N, {stepedge, cgal, gdal, las, mat});
}