#include <iostream>
#include <fstream>

#include "imgui.h"
#include "app_povi.h"
#include "nodes.h"
#include <gdal_nodes.hpp>
#include <las_nodes.hpp>
#include <cgal_nodes.hpp>
#include <array>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int ac, const char * av[]) {
    std::string lines_file_in;
    std::string lines_file_out;
    float simplification_threshold = 0.5;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("lines_file_in", po::value<std::string>(&lines_file_in), "Input lines")
      ("lines_file_out", po::value<std::string>(&lines_file_out), "Output lines")
      ("simplification_threshold", po::value<float>(&simplification_threshold), "Simplification threshold")
      ;
    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 1;
    }

    geoflow::NodeManager N;

    auto ogr_loader = std::make_shared<OGRLoaderNode>(N);
    auto simplify_footprint = std::make_shared<SimplifyFootprintNode>(N);
    auto ogr_writer = std::make_shared<OGRWriterNoAttributesNode>(N);

    std::strcpy(ogr_loader->filepath, lines_file_in.c_str());
    std::strcpy(ogr_writer->filepath, lines_file_out.c_str());
    simplify_footprint->threshold_stop_cost = simplification_threshold;

    // footprint simplification
    geoflow::connect(ogr_loader->outputs("linear_rings"), simplify_footprint->inputs("polygons"));;


    a->run();
}