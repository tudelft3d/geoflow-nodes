#include <iostream>
#include <fstream>

#include <stepedge_register.hpp>
#include <gdal_register.hpp>
#include <cgal_register.hpp>
#include <geoflow/gui/flowchart.hpp>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace gfn = geoflow::nodes;

//‎⁨ /Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_gebouwen/bag.gpkg
//‎⁨ /Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_puntenwolk/extend.las
// /Users/ravi/surfdrive/Data/step-edge-detector/C_31HZ1_clip.LAZ
// /Users/ravi/surfdrive/Data/step-edge-detector/ahn3.las
// /Users/ravi/surfdrive/Data/step-edge-detector/rdam_sample_0.gpkg
//viewer nodes

int main(int ac, const char * av[])
{   
    std::string footprints_file;
    std::string las_file;
    std::string decomposed_footprints_file = "out.shp";
    float step_threshold = 1.0;
    bool gui = false;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("gui", po::bool_switch(&gui), "launch gui") 
    ("las", po::value<std::string>(&las_file), "Point cloud ")
    ("footprints", po::value<std::string>(&footprints_file), "Footprints")
    ("output", po::value<std::string>(&decomposed_footprints_file), "Decomposed footprints")
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
    auto stepedge = gfn::stepedge::create_register();
    auto gdal = gfn::gdal::create_register();
    auto cgal = gfn::cgal::create_register();

    auto ogr_loader = N.create_node(gdal, "OGRLoader", {75,75});
    auto las_in_poly = N.create_node(stepedge, "LASInPolygons", {300,75});
    auto lod13generator = N.create_node(stepedge, "LOD13Generator", {650,75});
    auto ogr_writer = N.create_node(gdal, "OGRWriter", {1000,75});

    ogr_loader->set_param("filepath", footprints_file);
    las_in_poly->set_param("las_filepath", las_file);

    geoflow::connect(ogr_loader->output("linear_rings"), las_in_poly->input("polygons"));
    geoflow::connect(ogr_loader->output("linear_rings"), lod13generator->input("polygons"));
    geoflow::connect(las_in_poly->output("point_clouds"), lod13generator->input("point_clouds"));
    geoflow::connect(lod13generator->output("decomposed_footprints"), ogr_writer->input("geometries"));
    geoflow::connect(lod13generator->output("attributes"), ogr_writer->input("attributes"));

    ogr_writer->set_param("filepath", decomposed_footprints_file);
    lod13generator->set_param("step_height_threshold", step_threshold);
    
    if (gui)
        geoflow::launch_flowchart(N, {stepedge, gdal, cgal});
    else
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