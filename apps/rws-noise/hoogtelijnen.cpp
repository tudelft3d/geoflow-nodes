#include <iostream>
#include <fstream>

#include <general_register.hpp>
#include <gdal_register.hpp>
#include <cgal_register.hpp>
#include <las_register.hpp>
#include <geoflow/gui/flowchart.hpp>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace gfn = geoflow::nodes;

int main(int ac, const char * av[])
{
    std::string lines_file_in;
    std::string lines_file_out;
    std::string las_file;
    float selection_threshold = 0.5;
    float simplification_threshold = 10;
    int pointthinning = 10;
    float iso_interval = 1.0;
    bool gui = false;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("gui", po::bool_switch(&gui), "launch gui")
    ("las", po::value<std::string>(&las_file), "Point cloud ")
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

    auto general = gfn::general::create_register();
    auto las = gfn::las::create_register();
    auto cgal = gfn::cgal::create_register();
    auto gdal = gfn::gdal::create_register();
    geoflow::NodeManager N;

    //auto ogr_loader = N.create_node(gdal, "OGRLoader");
    //auto tin_creator_lines = N.create_node(cgal, "CDT");
    //auto height_difference_calc = N.create_node(cgal, "PointDistance");
    //auto tin_creator_difference = N.create_node(cgal, "CDT");
    //auto iso_lines = N.create_node(cgal, "IsoLine");
    //auto line_merger = N.create_node(gdal, "GEOSMergeLines");
    //auto add_height_to_lines = N.create_node(cgal, "LineHeight");
    //auto tin_simp_lines = N.create_node(cgal, "TinSimp");
    //auto tin_simp_iso = N.create_node(cgal, "TinSimp");
    //auto simplify_lines_cdt = N.create_node(cgal, "SimplifyLines");
    //auto ogr_writer = N.create_node(gdal, "OGRWriterNoAttributes");

    //ogr_loader->set_param("filepath", lines_file_in);
    //ogr_writer->set_param("filepath", lines_file_out);
    //
    //tin_creator_lines->set_params({
    //    {"create_triangles", true}
    //  });
    //height_difference_calc->set_params({
    //    {"filepath", las_file},
    //    {"thin_nth", pointthinning},
    //    {"overwritez", true}
    //});
    //add_height_to_lines->set_params({
    //    {"filepath", las_file},
    //    {"thin_nth", pointthinning}
    //});
    //tin_simp_lines->set_params({
    //    {"thres_error", selection_threshold},
    //    {"densify_interval", selection_threshold}
    //});
    //tin_simp_iso->set_params({
    //    {"thres_error", selection_threshold},
    //    {"densify_interval", selection_threshold}
    //});
    //simplify_lines_cdt->set_params({
    //    {"threshold_stop_cost", simplification_threshold}
    //});

    //// iso line generation
    //geoflow::connect(ogr_loader->output("line_strings"), tin_creator_lines->input("geometries"));;
    //// calculate height difference tin-points
    //geoflow::connect(tin_creator_lines->output("triangles"), height_difference_calc->input("triangles"));
    //// make tin from height differences
    //geoflow::connect(height_difference_calc->output("points"), tin_creator_difference->input("geometries"));
    //// make iso lines
    //geoflow::connect(height_difference_calc->output("distance_min"), iso_lines->input("min"));
    //geoflow::connect(height_difference_calc->output("distance_max"), iso_lines->input("max"));
    //geoflow::connect(tin_creator_difference->output("cgal_cdt"), iso_lines->input("cgal_cdt"));
    //// merge iso line segments
    //geoflow::connect(iso_lines->output("lines"), line_merger->input("lines"));
    //// add height to lines
    //geoflow::connect(line_merger->output("lines"), add_height_to_lines->input("lines"));
    //// remove lines using tinsimp
    //geoflow::connect(ogr_loader->output("line_strings"), tin_simp_lines->input("geometries"));
    //geoflow::connect(add_height_to_lines->output("lines"), tin_simp_iso->input("geometries"));
    //// simplify lines using CDT
    //geoflow::connect(tin_simp_lines->output("selected_lines"), simplify_lines_cdt->input("lines"));
    //geoflow::connect(tin_simp_iso->output("selected_lines"), simplify_lines_cdt->input("lines2"));
    //// write lines
    //geoflow::connect(simplify_lines_cdt->output("lines"), ogr_writer->input("geometries"));

    // Create nodes
    auto las_loader = N.create_node(las, "LASLoader");
    auto tin_creator_lidar = N.create_node(cgal, "TinSimp");
    auto ogr_loader = N.create_node(gdal, "OGRLoader");
  // TEST: line height from CDT calculator -> add Bounding Box!!! see build_initial_tin()
  auto line_height_adder_input = N.create_node(cgal, "LineHeightCDT"); 
    auto tin_creator_lines = N.create_node(cgal, "CDT");
  // TEST: CDT vertex - CDT distance calculator
  auto height_difference_calc = N.create_node(cgal, "CDTDistance");
    auto tin_creator_difference = N.create_node(cgal, "TinSimp");
    auto iso_lines = N.create_node(cgal, "IsoLine");
    auto line_merger = N.create_node(gdal, "GEOSMergeLines");
  // TEST: filter lines based on length
  auto line_string_filter = N.create_node(general, "LineStringFilter");
  // lineheight from CDT calculator
  auto line_height_adder_iso = N.create_node(cgal, "LineHeightCDT"); 
  // TEST: merge height lines + ISO lines
  auto geometry_merger = N.create_node(general, "MergeGeometries");
    auto tin_simp_lines = N.create_node(cgal, "TinSimp");
    auto simplify_lines_cdt = N.create_node(cgal, "SimplifyLines");
    auto ogr_writer = N.create_node(gdal, "OGRWriterNoAttributes");

    // Setup nodes
    ogr_loader->set_param("filepath", lines_file_in);
    ogr_writer->set_param("filepath", lines_file_out);

    las_loader->set_params({
        {"filepath", las_file},
        {"thin_nth", pointthinning}
    }); 
    tin_creator_lidar->set_params({
        {"thres_error", selection_threshold},
        {"create_triangles", false}
    });
    line_height_adder_input->set_params({
        {"densify_interval", selection_threshold}
    });
    tin_creator_lines->set_params({
        {"create_triangles", false}
    });
    tin_creator_difference->set_params({
        {"thres_error", selection_threshold},
        {"create_triangles", false}
    });
    iso_lines->set_params({
        {"interval", iso_interval}
    });
    line_height_adder_iso->set_params({
        {"densify_interval", selection_threshold}
    });
    tin_simp_lines->set_params({
        {"thres_error", selection_threshold},
        {"densify_interval", selection_threshold}
    });
    simplify_lines_cdt->set_params({
        {"threshold_stop_cost", simplification_threshold}
    });

    // Connect nodes
    // Create TIN from lidar using tinsimp
    geoflow::connect(las_loader->output("points"), tin_creator_lidar->input("points"));
    // Add heights to input lines from TIN
    geoflow::connect(tin_creator_lidar->output("cgal_cdt"), line_height_adder_input->input("cgal_cdt"));
    geoflow::connect(ogr_loader->output("line_strings"), line_height_adder_input->input("line_strings"));
    // Create TIN from height lines
    geoflow::connect(line_height_adder_input->output("line_strings"), tin_creator_lines->input("geometries"));
    // Calculate height difference of vertices in lidar TIN with height lines TIN
    geoflow::connect(tin_creator_lines->output("cgal_cdt"), height_difference_calc->input("cgal_cdt_base"));
    geoflow::connect(tin_creator_lidar->output("cgal_cdt"), height_difference_calc->input("cgal_cdt_target"));
    // Create TIN from height differences
    geoflow::connect(height_difference_calc->output("points"), tin_creator_difference->input("geometries"));
    // Create ISO lines of difference TIN and merge line segments
    geoflow::connect(height_difference_calc->output("distance_min"), iso_lines->input("min"));
    geoflow::connect(height_difference_calc->output("distance_max"), iso_lines->input("max"));
    geoflow::connect(tin_creator_difference->output("cgal_cdt"), iso_lines->input("cgal_cdt"));
    geoflow::connect(iso_lines->output("lines"), line_merger->input("lines"));
    // Add heights to ISO lines based on lidar TIN (also densify?)
    geoflow::connect(line_merger->output("lines"), line_height_adder_iso->input("lines"));
    // merge height lines + ISO lines
    geoflow::connect(line_height_adder_input->output("lines"), geometry_merger->input("geometries1"));
    geoflow::connect(line_height_adder_iso->output("lines"), geometry_merger->input("geometries2"));
    // Filter height lines + ISO lines using tinsimp
    geoflow::connect(geometry_merger->output("geometries"), tin_simp_lines->input("geometries"));
    // Simplify height lines + ISO lines using CDT Visvalingam
    geoflow::connect(tin_simp_lines->output("selected_lines"), simplify_lines_cdt->input("lines"));
    // Filter lines < 10m
    geoflow::connect(simplify_lines_cdt->output("lines"), line_string_filter->input("line_strings"));
    // Write lines
    geoflow::connect(line_string_filter->output("line_strings"), ogr_writer->input("geometries"));

    if (gui)
        geoflow::launch_flowchart(N, {cgal, gdal});
    else {
      N.run(*tin_creator_lidar);
      N.run(*ogr_loader);
    }
}