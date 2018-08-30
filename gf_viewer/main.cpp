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
    N.register_node<TriangleNode>("Triangle");
    N.register_node<BuildArrangementNode>("BuildArrangement");
    N.register_node<DetectLinesNode>("DetectLines");
    N.register_node<ClassifyEdgePointsNode>("ClassifyEdgePoints");
    N.register_node<ComputeMetricsNode>("ComputeMetrics");
    N.register_node<PointsInFootprintNode>("PointsInFootprint");
    a->draw_that(on_draw);
    a->run();
}