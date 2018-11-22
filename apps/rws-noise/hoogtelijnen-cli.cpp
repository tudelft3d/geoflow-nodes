#include <iostream>
#include <fstream>

#include "imgui.h"
#include "app_povi.h"
#include "nodes.h"
#include <gdal_nodes.hpp>
// #include <las_nodes.hpp>
#include <cgal_nodes.hpp>
#include <array>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int ac, const char * av[])
{
    std::string lines_file_in;
    std::string lines_file_out;
    // std::string las_file = "cloud.las";
    float selection_threshold = 0.5;
    float simplification_threshold = 10;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    // ("las", po::value<std::string>(&las_file), "Point cloud ")
    ("lines_file_in", po::value<std::string>(&lines_file_in), "Input lines")
    ("lines_file_out", po::value<std::string>(&lines_file_out), "Output lines")
    ("selection_threshold", po::value<float>(&selection_threshold), "Selection threshold")
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
    auto tin_simp = std::make_shared<TinSimpNode>(N);
    auto simp_3d = std::make_shared<SimplifyLine3DNode>(N);
    auto ogr_writer = std::make_shared<OGRWriterNoAttributesNode>(N);

    std::strcpy(ogr_loader->filepath, lines_file_in.c_str());
    std::strcpy(ogr_writer->filepath, lines_file_out.c_str());

    geoflow::connect(ogr_loader->outputs("line_strings"), tin_simp->inputs("geometries"));
    geoflow::connect(tin_simp->outputs("selected_lines"), simp_3d->inputs("lines"));
    geoflow::connect(simp_3d->outputs("lines"), ogr_writer->inputs("geometries"));

    tin_simp->thres_error = selection_threshold;
    tin_simp->densify_interval = selection_threshold;
    simp_3d->area_threshold = simplification_threshold;
    N.run(*ogr_loader);
}