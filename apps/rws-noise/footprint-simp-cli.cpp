#include <iostream>
#include <fstream>

#include <gdal_nodes.hpp>
#include <cgal_nodes.hpp>
#include <array>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int ac, const char * av[]) {
    std::string footprint_file_in;
    std::string footprint_file_out;
    float simplification_threshold = 1;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("footprint_file_in", po::value<std::string>(&footprint_file_in), "Input lines")
      ("footprint_file_out", po::value<std::string>(&footprint_file_out), "Output lines")
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
    //auto simplify_footprint = std::make_shared<SimplifyFootprintNode>(N);
    //auto simplify_footprint = std::make_shared<SimplifyFootprintsCDTNode>(N);
    auto simplify_footprint = std::make_shared<SimplifyLinesBufferNode>(N);
    auto ogr_writer = std::make_shared<OGRWriterNoAttributesNode>(N);

    std::strcpy(ogr_loader->filepath, footprint_file_in.c_str());
    std::strcpy(ogr_writer->filepath, footprint_file_out.c_str());
    //simplify_footprint->threshold_stop_cost = simplification_threshold;
    simplify_footprint->threshold = simplification_threshold;

    // footprint simplification
    geoflow::connect(ogr_loader->outputs("linear_rings"), simplify_footprint->inputs("polygons"));;
    geoflow::connect(simplify_footprint->outputs("polygons_simp"), ogr_writer->inputs("geometries"));
    
    N.run(*ogr_loader);
}