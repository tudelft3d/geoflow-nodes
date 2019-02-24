#include <iostream>
#include <fstream>

#include <geoflow/gui/flowchart.hpp>

#include <geoflow/core/geoflow.hpp>
#include <cityjson_nodes.hpp>
#include <utility_nodes.hpp>
// #include <las_register.hpp>

namespace gfn = geoflow::nodes;

int main(int ac, const char * av[])
{
    geoflow::NodeManager N;

    // register nodes
    NodeRegister utility = gfn::utility::create_register();
    NodeRegister cityjson = gfn::cityjson::create_register();

    // create nodes
    auto cjreader = N.create_node(cityjson, "CityJSONReader", {200,100});
    auto ringtri = N.create_node(utility, "RingTriangulator", {600,100});

    // connect terminals
    connect(
        cjreader->output("faces"),
        ringtri->input("rings")
    );
    connect(
        cjreader->output("surface_types"),
        ringtri->input("values")
    );
    
    // Run the flowchart (no GUI required)
    // N.run(cjreader);

    // launch GUI
    launch_flowchart(N, {cityjson, utility});
}