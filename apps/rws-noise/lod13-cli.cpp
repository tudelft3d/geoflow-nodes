#include <iostream>
#include <fstream>

#include <stepedge_nodes.hpp>
#include <gdal_nodes.hpp>
#include <las_nodes.hpp>
#include <array>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int ac, const char * av[])
{
    //‎⁨ /Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_gebouwen/bag.gpkg
    //‎⁨ /Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_puntenwolk/extend.las
    // /Users/ravi/surfdrive/Data/step-edge-detector/C_31HZ1_clip.LAZ
    // /Users/ravi/surfdrive/Data/step-edge-detector/ahn3.las
    // /Users/ravi/surfdrive/Data/step-edge-detector/rdam_sample_0.gpkg

    std::string footprints_file = "/Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_gebouwen/bgt_singleparts.gpkg";
    std::string las_file = "/Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_puntenwolk/extend.las";
    std::string decomposed_footprints_file = "out.shp";
    float step_threshold = 1.0;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("las", po::value<std::string>(&las_file), "Point cloud ")
    ("footprints", po::value<std::string>(&footprints_file), "Footprints")
    ("step_threshold", po::value<float>(&step_threshold), "Step threshold")
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
    auto las_in_poly = std::make_shared<LASInPolygonsNode>(N);
    auto lod13generator = std::make_shared<LOD13GeneratorNode>(N);
    auto ogr_writer = std::make_shared<OGRWriterNode>(N);

    std::strcpy(ogr_loader->filepath, footprints_file.c_str());
    std::strcpy(las_in_poly->las_filepath, las_file.c_str());

    geoflow::connect(ogr_loader->outputs("features"), las_in_poly->inputs("polygons"));
    geoflow::connect(ogr_loader->outputs("features"), lod13generator->inputs("polygons"));
    geoflow::connect(las_in_poly->outputs("point_clouds"), lod13generator->inputs("point_clouds"));
    geoflow::connect(lod13generator->outputs("decomposed_footprints"), ogr_writer->inputs("geometries"));
    geoflow::connect(lod13generator->outputs("attributes"), ogr_writer->inputs("attributes"));

    std::string out_file = "out_" + std::to_string(step_threshold) + ".shp";
    std::strcpy(ogr_writer->filepath, out_file.c_str());
    lod13generator->step_threshold = step_threshold;
    N.run(*ogr_loader);

    // FIXME: accept multiple stepedge_thresholds from cli argument...
    // bool init = false;
    // for (float step_threshold : {1.0, 2.0, 3.0, 4.0}) {
    //   std::string out_file = "out_" + std::to_string(step_threshold) + ".shp";
    //   std::strcpy(ogr_writer->filepath, out_file.c_str());
    //   lod13generator->step_threshold = step_threshold;
    //   if(!init){
    //     N.run(*ogr_loader);
    //     init=true;
    //   } else 
    //     N.run(*lod13generator);
    // }
}