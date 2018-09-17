#include <iostream>
#include <fstream>
#include <cstring>

#include "geoflow.hpp"
#include "gdal_nodes.hpp"
#include <array>

#include <boost/program_options.hpp>

using namespace geoflow;
namespace po = boost::program_options;

static NodeManager N;

int main(int ac, const char * av[])
{
    std::string path_lines_1 = "/Users/ravi/surfdrive/Data/step-edge-detector/small-test/3bg_v01_hoogtelijnen_.gpkg";
    std::string path_lines_2 = "/Users/ravi/surfdrive/Data/step-edge-detector/small-test/dgmr_hoogtelijnen_.gpkg";
    std::string path_las = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
    std::string path_out = "ComparePointDistanceNode.out";
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("las", po::value<std::string>(&path_las), "Point cloud ")
    ("lines_1", po::value<std::string>(&path_lines_1), "Lines 1")
    ("lines_2", po::value<std::string>(&path_lines_2), "Lines 2")
    ("csv_out", po::value<std::string>(&path_out), "Output csv ")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    NodeManager N = NodeManager();
    // N.register_node<OGRLoaderNode>("OGRLoader");
    // N.register_node<CDTNode>("CDT");
    // N.register_node<PointDistanceNode>("PointDistance");
    // N.register_node<ComparePointDistanceNode>("ComparePointDistance");
    auto ogr_loader_1 = std::make_shared<OGRLoaderNode>(N);
    std::strcpy(ogr_loader_1->filepath, path_lines_1.c_str());
    auto ogr_loader_2 = std::make_shared<OGRLoaderNode>(N);
    std::strcpy(ogr_loader_2->filepath, path_lines_2.c_str());
    auto cdt_1 = std::make_shared<CDTNode>(N);
    auto cdt_2 = std::make_shared<CDTNode>(N);
    auto comp_dist = std::make_shared<ComparePointDistanceNode>(N);
    std::strcpy(comp_dist->las_filepath, path_las.c_str());
    std::strcpy(comp_dist->log_filepath, path_out.c_str());

    connect(*ogr_loader_1, *cdt_1, "lines_vec3f", "lines_vec3f");
    connect(*ogr_loader_2, *cdt_2, "lines_vec3f", "lines_vec3f");
    connect(*cdt_1, *comp_dist, "triangles_vec3f", "triangles1_vec3f");
    connect(*cdt_2, *comp_dist, "triangles_vec3f", "triangles2_vec3f");
    N.run(*ogr_loader_1);
    N.run(*ogr_loader_2);
    // connect(*number, *adder, "result", "in1");
    // connect(*number, *adder, "result", "in2");
    // connect(*adder, *adder2, "result", "in1");
    // connect(*adder, *adder2, "result", "in2");
    // bool success = N.run(*number);
    // if (success){
    // try{
    // std::cout << "Result: " << std::any_cast<float>(adder2->outputTerminals["result"]->cdata) << "\n";
    // } catch(const std::bad_any_cast& e) {
    // std::cout << "Oops... " << e.what() << '\n';
    // }
    // } else {
    // std::cout << "No result, missing inputs\n";
    // }
    //gdal nodes
    return 0;
}