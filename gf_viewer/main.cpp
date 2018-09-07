#include <iostream>
#include <fstream>

#include "imgui.h"
#include "app_povi.h"
#include "nodes.h"
#include "nodes.hpp"
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
    //viewer nodes
    N.register_node<TriangleNode>("Triangle");
    N.register_node<ColorMapperNode>("ColorMapper");
    N.register_node<GradientMapperNode>("GradientMapper");

    //processing nodes
    N.register_node<ExtruderNode>("Extruder");
    N.register_node<ProcessArrangementNode>("ProcessArrangement");
    N.register_node<BuildArrangementNode>("BuildArrangement");
    N.register_node<DetectLinesNode>("DetectLines");
    N.register_node<ClassifyEdgePointsNode>("ClassifyEdgePoints");
    N.register_node<ComputeMetricsNode>("ComputeMetrics");
    N.register_node<PointsInFootprintNode>("PointsInFootprint");
    a->draw_that(on_draw);

    ImGui::NodeStore ns;
    ns.push_back(std::make_pair("PointsInFootprint", ImVec2(75,75)));
    ns.push_back(std::make_pair("ComputeMetrics", ImVec2(300,75)));
    ns.push_back(std::make_pair("ClassifyEdgePoints", ImVec2(600,75)));
    ns.push_back(std::make_pair("DetectLines", ImVec2(900,75)));
    ns.push_back(std::make_pair("BuildArrangement", ImVec2(1200,75)));
    ns.push_back(std::make_pair("ProcessArrangement", ImVec2(1500,75)));
    ns.push_back(std::make_pair("Extruder", ImVec2(1800,75)));
    nodes_.PreloadNodes(ns);

    a->run();
}