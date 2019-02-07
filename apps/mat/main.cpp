#include <iostream>
#include <fstream>

#include "imgui.h"
#include <geoflow/gui/flowchart.hpp>

#include <geoflow/core/geoflow.hpp>
#include <las_register.hpp>
#include <cgal_register.hpp>
#include <masb_register.hpp>
#include <array>

namespace gfn = geoflow::nodes;

int main(int ac, const char * av[])
{
    geoflow::NodeManager N;

    // register nodes
    NodeRegister cgal = gfn::cgal::create_register();
    NodeRegister mat = gfn::mat::create_register();
    NodeRegister las = gfn::las::create_register();

    launch_flowchart(N, {cgal, las, mat});
}