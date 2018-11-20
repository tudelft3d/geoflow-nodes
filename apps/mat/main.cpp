#include <iostream>
#include <fstream>

#include "imgui.h"
#include "app_povi.h"
#include "nodes.h"
#include <las_nodes.hpp>
#include <masb_nodes.hpp>
#include <array>

static auto a = std::make_shared<poviApp>(1280, 800, "MAT processor");
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
    N.register_node<LASLoaderNode>("LASLoader");
    
    N.register_node<ComputeMedialAxisNode>("ComputeMedialAxis");
    N.register_node<ComputeNormalsNode>("ComputeNormals");

    a->draw_that(on_draw);

    a->run();
}