#include <iostream>
#include <fstream>

#include <stepedge_register.hpp>
#include <gdal_register.hpp>
#include <cgal_register.hpp>
// #include <geoflow/gui/flowchart.hpp>

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
    //  std::string footprints_file("/Users/ravi/surfdrive/Data/step-edge-detector/rdam_sample_1.gpkg");
   std::string footprints_file("/Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_gebouwen/bag.gpkg");
    //  std::string las_file("/Users/ravi/surfdrive/Data/step-edge-detector/ahn3.las");
   std::string las_file("/Users/ravi/surfdrive/Data/step-edge-detector/nieuwegein_puntenwolk/extend.las");
    std::string footprints_classes_file("out_bclass.shp");
    std::string decomposed_footprints_file("out_lod13.shp");
    float step_threshold = 1.0;
    float percentile = 0.9;
    bool regularise_footprint = false;
    bool use_linedetector = false;
    bool presimp_fp = false;
    // bool gui = false;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    // ("gui", po::bool_switch(&gui), "launch gui") 
    ("use_linedetector", po::bool_switch(&use_linedetector), "use line detector") 
    ("regularise_footprint", po::bool_switch(&regularise_footprint), "regularise footprints") 
    ("presimp_fp", po::bool_switch(&presimp_fp), "Simplify input polygons with Douglas Peucker 10 cm") 
    ("las", po::value<std::string>(&las_file), "Point cloud ")
    ("footprints", po::value<std::string>(&footprints_file), "Footprints")
    ("output", po::value<std::string>(&decomposed_footprints_file), "Decomposed footprints")
    ("step_threshold", po::value<float>(&step_threshold), "Step threshold")
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
    auto cgal = gfn::cgal::create_register();

    auto ogr_loader = N.create_node(gdal, "OGRLoader", {75,75});
    auto footprint_simp = N.create_node(stepedge, "SimplifyPolygon", {75,175});
    auto las_in_poly = N.create_node(stepedge, "LASInPolygons", {300,75});
    auto lod13generator = N.create_node(stepedge, "LOD13Generator", {650,75});
    auto ogr_writer = N.create_node(gdal, "OGRWriter", {1000,75});

    ogr_loader->set_param("filepath", footprints_file);
    las_in_poly->set_param("las_filepath", las_file);

    lod13generator->set_param("step_height_threshold", step_threshold);
    lod13generator->set_param("z_percentile", percentile);
    lod13generator->set_param("use_only_hplanes", true);
    lod13generator->set_param("regularise_footprint", regularise_footprint);
    lod13generator->set_param("use_linedetector", use_linedetector);

    ogr_writer->set_param("filepath", decomposed_footprints_file);

    ogr_loader->output_group("attributes").connect(
      lod13generator->input_group("attributes")
    );
    if (presimp_fp) {
        geoflow::connect(
            ogr_loader->output("linear_rings"),
            footprint_simp->input("polygons")
        );
        geoflow::connect(
            footprint_simp->output("polygons_simp"),
            las_in_poly->input("polygons")
        );
        geoflow::connect(
            footprint_simp->output("polygons_simp"),
            lod13generator->input("polygons")
        );
    } else {
        geoflow::connect(
            ogr_loader->output("linear_rings"),
            las_in_poly->input("polygons")
        );
        geoflow::connect(
            ogr_loader->output("linear_rings"),
            lod13generator->input("polygons")
        );
    }
    geoflow::connect(
        las_in_poly->output("point_clouds"),
        lod13generator->input("point_clouds")
    );
    geoflow::connect(
        lod13generator->output("decomposed_footprints"),
        ogr_writer->input("geometries")
    );
    lod13generator->output_group("attributes").connect(
      ogr_writer->input_group("attributes")
    );
  
    // if (gui)
    //     geoflow::launch_flowchart(N, {stepedge, gdal, cgal});
    // else
        N.run(*ogr_loader);
}
