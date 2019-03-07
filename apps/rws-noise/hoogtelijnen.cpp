#include <iostream>
#include <fstream>
#include <exception>

#include <general_register.hpp>
#include <gdal_register.hpp>
#include <cgal_register.hpp>
#include <las_register.hpp>

#ifdef GF_BUILD_GUI
    #include <geoflow/gui/flowchart.hpp>
#endif

#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace gfn = geoflow::nodes;

int main(int ac, const char * av[])
{
    std::string lines_file_in;
    std::string lines_file_out_all;
    std::string lines_file_out_bgtonly;
    std::string las_file;
    float selection_threshold = 0.5;
    float simplification_threshold = 10;
    float line_densification = 2.0;
    int pointthinning = 1;
    float iso_interval = 1.0;
    float iso_exclude_begin = -1.0;
    float iso_exclude_end = 1.0;
    float min_line_length = 10;
    bool gui = false;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    #ifdef GF_BUILD_GUI
        ("gui", po::bool_switch(&gui), "launch gui")
    #endif
    ("las", po::value<std::string>(&las_file), "Point cloud ")
    ("lines_file_in", po::value<std::string>(&lines_file_in), "Input lines")
    ("lines_file_out", po::value<std::string>(&lines_file_out_all), "All output lines")
    ("lines_file_out_bgtonly", po::value<std::string>(&lines_file_out_bgtonly), "Output lines bgt only")
    ("selection_threshold", po::value<float>(&selection_threshold), "Selection threshold")
    ("simplification_threshold", po::value<float>(&simplification_threshold), "Simplification threshold (area of triangle)")
    ("line_densification", po::value<float>(&line_densification), "Line densification distance")
    ("iso_interval", po::value<float>(&iso_interval), "Iso line interval")
    ("iso_exclude_begin", po::value<float>(&iso_exclude_begin), "Iso line exclude begin")
    ("iso_exclude_end", po::value<float>(&iso_exclude_end), "Iso line exclude end")
    ("min_line_length", po::value<float>(&min_line_length), "Minimum line length")
    ("pointthinning", po::value<int>(&pointthinning), "Point thinning")
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
    //auto las_loader = N.create_node(las, "LASGroundLoader", { -200, 300 });
    //auto tin_creator_lidar = N.create_node(cgal, "TinSimp", { 30, 300 });
    auto tin_creator_lidar = N.create_node(cgal, "TinSimpLASReader", { 30, 300 });

    auto ogr_loader = N.create_node(gdal, "OGRLoader", { -200, 50 });
    auto line_height_adder_input = N.create_node(cgal, "LineHeightCDT", { 30, 50 }); 
    auto tin_creator_lines = N.create_node(cgal, "CDT", { 275, 50 });

    auto height_difference_calc = N.create_node(cgal, "CDTDistance", { 550, 150 });

    auto tin_creator_difference = N.create_node(cgal, "TinSimp", { 900, 0 });
    auto iso_lines = N.create_node(cgal, "IsoLine", { 900, 140 });
    auto line_merger = N.create_node(gdal, "GEOSMergeLines", { 900, 260 });
    auto line_height_adder_iso = N.create_node(cgal, "LineHeightCDT", { 900, 340 }); 

    auto line_string_merger = N.create_node(general, "MergeLinestrings", { 1250, 0 });
    auto tin_simp_lines = N.create_node(cgal, "TinSimp", { 1250, 110 });
    auto simplify_lines_cdt = N.create_node(cgal, "SimplifyLines", { 1250, 215 });
    auto line_string_filter = N.create_node(general, "LineStringFilter", { 1250, 285 });
    auto ogr_writer = N.create_node(gdal, "OGRWriterNoAttributes", { 1250, 350 });
    auto ogr_writer_bgtonly = N.create_node(gdal, "OGRWriterNoAttributes", { 1250, 550 });

    // Setup nodes
    ogr_loader->set_param("filepath", lines_file_in);
    ogr_writer->set_param("filepath", lines_file_out_all);
    ogr_writer_bgtonly->set_param("filepath", lines_file_out_bgtonly);

    //las_loader->set_params({
    //    {"filepath", las_file},
    //    {"thin_nth", pointthinning}
    //}); 
    //tin_creator_lidar->set_params({
    //    {"thres_error", selection_threshold},
    //    {"create_triangles", false}
    //});
    tin_creator_lidar->set_params({
        {"filepath", las_file},
        {"thin_nth", pointthinning},
        {"thres_error", selection_threshold},
        {"create_triangles", false}
      });
    line_height_adder_input->set_params({
        {"densify_interval", line_densification}
    });
    tin_creator_lines->set_params({
        {"create_triangles", false}
    });
    tin_creator_difference->set_params({
        {"thres_error", selection_threshold},
        {"create_triangles", false}
    });
    iso_lines->set_params({
        {"interval", iso_interval},
        {"exclude_begin", iso_exclude_begin},
        {"exclude_end", iso_exclude_end}
    });
    line_height_adder_iso->set_params({
        {"densify_interval", line_densification}
    });
    tin_simp_lines->set_params({
        {"thres_error", selection_threshold},
        {"densify_interval", line_densification}
    });
    simplify_lines_cdt->set_params({
        {"threshold_stop_cost", simplification_threshold}
    });
    line_string_filter->set_params({
        {"filter_length", min_line_length}
    });

    try {
        // Connect nodes
        // Create TIN from lidar using tinsimp
        //geoflow::connect(las_loader->output("points"), tin_creator_lidar->input("geometries"));
        // Add heights to input lines from TIN
        geoflow::connect(tin_creator_lidar->output("cgal_cdt"), line_height_adder_input->input("cgal_cdt"));
        geoflow::connect(ogr_loader->output("line_strings"), line_height_adder_input->input("lines"));
        // Create TIN from height lines
        geoflow::connect(line_height_adder_input->output("lines"), tin_creator_lines->input("geometries"));
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
        geoflow::connect(tin_creator_lidar->output("cgal_cdt"), line_height_adder_iso->input("cgal_cdt"));
        geoflow::connect(line_merger->output("lines"), line_height_adder_iso->input("lines"));
        // merge height lines + ISO lines
        geoflow::connect(line_height_adder_input->output("lines"), line_string_merger->input("lines1"));
        geoflow::connect(line_height_adder_iso->output("lines"), line_string_merger->input("lines2"));
        // Filter height lines + ISO lines using tinsimp
        geoflow::connect(line_string_merger->output("lines"), tin_simp_lines->input("geometries"));
        // Simplify height lines + ISO lines using CDT Visvalingam
        geoflow::connect(tin_simp_lines->output("selected_lines"), simplify_lines_cdt->input("lines"));
        // Filter lines < 10m
        geoflow::connect(simplify_lines_cdt->output("lines"), line_string_filter->input("line_strings"));
        // Write lines
        geoflow::connect(line_string_filter->output("line_strings"), ogr_writer->input("geometries"));


        geoflow::connect(line_height_adder_input->output("lines"), ogr_writer_bgtonly->input("geometries"));

        #ifdef GF_BUILD_GUI
            if (gui)
                geoflow::launch_flowchart(N, {cgal, gdal});
            else {
                //N.run(*las_loader);
                N.run(*tin_creator_lidar);
                N.run(*ogr_loader);
            }
        #else
            //N.run(*las_loader);
            N.run(*tin_creator_lidar);
            N.run(*ogr_loader);
        #endif
    } catch (const std::exception e) {
      std::cout << e.what() << std::endl;
    }
}