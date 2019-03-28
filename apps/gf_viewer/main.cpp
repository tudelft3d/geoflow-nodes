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
#include <utility_nodes.hpp>

namespace gfn = geoflow::nodes;

int main(int ac, const char * av[])
{
    // /Users/ravi/surfdrive/Data/step-edge-detector/ahn3.las
    // /Users/ravi/surfdrive/Data/step-edge-detector/rdam_sample_0.gpkg
    // std::string las_path = "/Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_puntenwolk/extend.las";
    // std::string fp_path = "/Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_gebouwen/bag.gpkg";
    std::string las_path = "/Users/ravi/surfdrive/Data/step-edge-detector/ahn3.las";
    std::string fp_path = "/Users/ravi/surfdrive/Data/step-edge-detector/rdam_sample_1.gpkg";
    // register nodes from various modules
    auto stepedge = gfn::stepedge::create_register();
    auto cgal = gfn::cgal::create_register();
    auto gdal = gfn::gdal::create_register();
    auto las = gfn::las::create_register();
    auto mat = gfn::mat::create_register();
    auto utility = gfn::utility::create_register();

    // create some nodes and connections
    NodeManager N;

    gfn::stepedge::create_lod13chart(N, true);

    NodeHandle OGRLoader = N.create_node(gdal, "OGRLoader", {-275,75});
    NodeHandle PolygonSimp = N.create_node(stepedge, "SimplifyPolygon", {-275,175});
    NodeHandle LASInPolygons = N.create_node(stepedge, "LASInPolygons", {0,75});
    NodeHandle BuildingSelect = N.create_node(stepedge, "BuildingSelector", {0,175});
    // NodeHandle DetectPlanes = N.create_node(stepedge, "DetectPlanes", {300,75});
    // NodeHandle AlphaShape = N.create_node(stepedge, "AlphaShape", {600,75});
    // NodeHandle DetectLines = N.create_node(stepedge, "DetectLines", {900,75});
    // // NodeHandle RegulariseLines = N.create_node(stepedge, "RegulariseLines", {1200,175});
    // NodeHandle RegulariseRings = N.create_node(stepedge, "RegulariseRings", {1200,175});
    // NodeHandle BuildArrangement = N.create_node(stepedge, "BuildArrFromRings", {1200,75});
    // NodeHandle ProcessArrangement = N.create_node(stepedge, "ProcessArrangement", {1500,75});
    NodeHandle Extruder = N.create_node(stepedge, "Extruder", {1550,75});

    connect(OGRLoader, PolygonSimp, "linear_rings", "polygons");
    connect(PolygonSimp, BuildingSelect, "polygons_simp", "polygons");
    connect(PolygonSimp, LASInPolygons, "polygons_simp", "polygons");
    connect(LASInPolygons, BuildingSelect, "point_clouds", "point_clouds");
    connect(BuildingSelect, N.nodes["DetectPlanes_node"], "point_cloud", "points");
    connect(BuildingSelect, N.nodes["RegulariseRings_node"], "polygon", "footprint");
    // // connect(BuildingSelect, BuildArrangement, "polygon", "footprint");
    // connect(BuildingSelect, RegulariseRings, "polygon", "footprint");
    // connect(DetectPlanes, AlphaShape, "pts_per_roofplane", "pts_per_roofplane");
    // // connect(ComputeMetrics, ProcessArrangement, "points", "points");
    // connect(AlphaShape, DetectLines, "alpha_rings", "edge_points");
    // connect(DetectLines, RegulariseRings, "edge_segments", "edge_segments");
    // connect(DetectLines, RegulariseRings, "ring_idx", "ring_idx");
    // connect(RegulariseRings, BuildArrangement, "rings_out", "rings");
    // connect(RegulariseRings, BuildArrangement, "footprint_out", "footprint");
    // connect(DetectPlanes, BuildArrangement, "pts_per_roofplane", "pts_per_roofplane");
    // connect(BuildArrangement, ProcessArrangement, "arrangement", "arrangement");
    // connect(ProcessArrangement, Extruder, "arrangement", "arrangement");
    connect(N.nodes.at("BuildArrFromRings_node"), Extruder, "arrangement", "arrangement");

    LASInPolygons->set_param(
        "las_filepath", las_path);
    OGRLoader->set_param(
        "filepath", fp_path);

    // auto h_ogrload = N.create_node(gdal, "OGRLoader", {-275,300});
    // auto h_polex = N.create_node(stepedge, "PolygonExtruder", {-50,300});
    // auto h_rtri = N.create_node(utility, "RingTriangulator", {150,300});

    // h_ogrload->set_param("filepath", (std::string)"/Users/ravi/git/geoflow-nodes/build/out_lod13H.gpkg");

    // N.run(h_ogrload);

    // h_ogrload->output_group("attributes").term("height").connect(h_polex->input("heights"));
    // h_ogrload->output("linear_rings").connect(h_polex->input("polygons"));

    // launch the GUI
    launch_flowchart(N, {stepedge, cgal, gdal, las, mat, utility});
}