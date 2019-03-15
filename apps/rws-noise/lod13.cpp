#include <iostream>
#include <fstream>

#include <stepedge_register.hpp>
#include <gdal_register.hpp>
#include <cgal_register.hpp>
#ifdef GF_BUILD_GUI
    #include <geoflow/gui/flowchart.hpp>
#endif

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
    std::string decomposed_footprints_H_file("out_lod13H.gpkg");
    // std::string decomposed_footprints_A_file("out_lod13A.gpkg");
    std::string decomposed_footprints_10_file("out_lod10.gpkg");
    float step_threshold = 1.0;
    float percentile = 0.9;
    bool regularise_footprint = false;
    bool no_linedetector = false;
    bool presimp_fp = true;
    bool gui = false;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    #ifdef GF_BUILD_GUI
        ("gui", po::bool_switch(&gui), "launch gui")
    #endif
    ("no_linedetector", po::bool_switch(&no_linedetector), "don't use line detector") 
    ("regularise_footprint", po::bool_switch(&regularise_footprint), "regularise footprints") 
    // ("presimp_fp", po::bool_switch(&presimp_fp), "Simplify input polygons with Douglas Peucker 10 cm") 
    ("las", po::value<std::string>(&las_file), "Point cloud ")
    ("footprints", po::value<std::string>(&footprints_file), "Footprints")
    // ("outputA", po::value<std::string>(&decomposed_footprints_A_file), "Decomposed footprints LoD1.3A")
    ("outputH", po::value<std::string>(&decomposed_footprints_H_file), "Decomposed footprints LoD1.3H")
    ("output10", po::value<std::string>(&decomposed_footprints_10_file), "Footprints LoD1.0")
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
    auto lod13generatorH = N.create_node(stepedge, "LOD13Generator", {650,75});
    // auto lod13generatorA = N.create_node(stepedge, "LOD13Generator", {650,175});
    auto lod10generator = N.create_node(stepedge, "LOD10Generator", {650,275});
    auto ogr_writerH = N.create_node(gdal, "OGRWriter", {1000,75});
    // auto ogr_writerA = N.create_node(gdal, "OGRWriter", {1000,175});
    auto ogr_writer_lod10 = N.create_node(gdal, "OGRWriter", {1000,175});

    ogr_loader->set_param("filepath", footprints_file);
    las_in_poly->set_param("las_filepath", las_file);

    lod13generatorH->set_param("step_height_threshold", step_threshold);
    lod13generatorH->set_param("z_percentile", percentile);
    lod13generatorH->set_param("use_only_hplanes", true);
    lod13generatorH->set_param("regularise_footprint", regularise_footprint);
    lod13generatorH->set_param("use_linedetector", !no_linedetector);
    
    // lod13generatorA->set_param("step_height_threshold", step_threshold);
    // lod13generatorA->set_param("z_percentile", percentile);
    // lod13generatorA->set_param("use_only_hplanes", false);
    // lod13generatorA->set_param("regularise_footprint", regularise_footprint);
    // lod13generatorA->set_param("use_linedetector", !no_linedetector);

    lod10generator->set_param("z_percentile", percentile);

    ogr_writerH->set_param("filepath", decomposed_footprints_H_file);
    // ogr_writerA->set_param("filepath", decomposed_footprints_A_file);
    ogr_writer_lod10->set_param("filepath", decomposed_footprints_10_file);

    ogr_loader->output_group("attributes").connect(
      lod13generatorH->input_group("attributes")
    );
    // ogr_loader->output_group("attributes").connect(
    //   lod13generatorA->input_group("attributes")
    // );
    ogr_loader->output_group("attributes").connect(
        ogr_writer_lod10->input_group("attributes")
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
            lod13generatorH->input("polygons")
        );
        // geoflow::connect(
        //     footprint_simp->output("polygons_simp"),
        //     lod13generatorA->input("polygons")
        // );
    } else {
        geoflow::connect(
            ogr_loader->output("linear_rings"),
            las_in_poly->input("polygons")
        );
        geoflow::connect(
            ogr_loader->output("linear_rings"),
            lod13generatorH->input("polygons")
        );
        // geoflow::connect(
        //     ogr_loader->output("linear_rings"),
        //     lod13generatorA->input("polygons")
        // );
    }
    geoflow::connect(
        las_in_poly->output("point_clouds"),
        lod13generatorH->input("point_clouds")
    );
    geoflow::connect(
        lod13generatorH->output("decomposed_footprints"),
        ogr_writerH->input("geometries")
    );
    lod13generatorH->output_group("attributes").connect(
      ogr_writerH->input_group("attributes")
    );
    // geoflow::connect(
    //     las_in_poly->output("point_clouds"),
    //     lod13generatorA->input("point_clouds")
    // );
    // geoflow::connect(
    //     lod13generatorA->output("decomposed_footprints"),
    //     ogr_writerA->input("geometries")
    // );
    // lod13generatorA->output_group("attributes").connect(
    //   ogr_writerA->input_group("attributes")
    // );
    geoflow::connect(
        ogr_loader->output("linear_rings"),
        ogr_writer_lod10->input("geometries")
    );
    geoflow::connect(
        las_in_poly->output("point_clouds"),
        lod10generator->input("point_clouds")
    );
    lod10generator->output_group("attributes").connect(
        ogr_writer_lod10->input_group("attributes")
    );
  
    #ifdef GF_BUILD_GUI
        if (gui)
            geoflow::launch_flowchart(N, {stepedge, gdal, cgal});
        else
    #endif
        N.run(*ogr_loader);
}
