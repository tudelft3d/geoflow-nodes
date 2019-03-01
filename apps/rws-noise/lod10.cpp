#include <iostream>
#include <fstream>

#include <stepedge_register.hpp>
#include <gdal_register.hpp>
// #include <geoflow/gui/flowchart.hpp>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace gfn = geoflow::nodes;

int main(int ac, const char * av[])
{
    std::string footprints_file("/Users/ravi/surfdrive/Data/step-edge-detector/rdam_sample_1.gpkg");
//    std::string footprints_file("/Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_gebouwen/bag.gpkg");
    std::string las_file("/Users/ravi/surfdrive/Data/step-edge-detector/ahn3.las");
//    std::string las_file("/Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_puntenwolk/extend.las");
    std::string footprints_out_file("out.shp");
    bool gui = false;
    float percentile = 0.9;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("gui", po::bool_switch(&gui), "launch gui") 
    ("las", po::value<std::string>(&las_file), "Point cloud ")
    ("footprints", po::value<std::string>(&footprints_file), "Input footprints")
    ("output", po::value<std::string>(&footprints_out_file), "Output footprints")
    ("percentile", po::value<float>(&percentile), "Percentile")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    geoflow::NodeManager N;
    auto stepedge = gfn::stepedge::create_register();
    auto gdal = gfn::gdal::create_register();

    auto ogr_loader = N.create_node(gdal, "OGRLoader", {75,75});
    // auto footprint_simp = N.create_node(stepedge, "SimplifyPolygon", {75,175});
    auto las_in_poly = N.create_node(stepedge, "LASInPolygons", {300,75});
    auto lod10generator = N.create_node(stepedge, "LOD10Generator", {650,75});
    auto ogr_writer = N.create_node(gdal, "OGRWriter", {1000,75});

    ogr_loader->set_param("filepath", footprints_file);
    las_in_poly->set_param("las_filepath", las_file);

    lod10generator->set_param("z_percentile", percentile);
    
    ogr_writer->set_param("filepath", footprints_out_file);

    geoflow::connect(
        ogr_loader->output("linear_rings"), 
        las_in_poly->input("polygons")
    );
    geoflow::connect(
        las_in_poly->output("point_clouds"), 
        lod10generator->input("point_clouds")
    );
    geoflow::connect(
        ogr_loader->output("linear_rings"), 
        ogr_writer->input("geometries")
    );
    ogr_loader->output_group("attributes").connect(
        ogr_writer->input_group("attributes")
    );
    lod10generator->output_group("attributes").connect(
        ogr_writer->input_group("attributes")
    );
    
    
    // if (gui)
    //     geoflow::launch_flowchart(N, {stepedge, gdal});
    // else
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