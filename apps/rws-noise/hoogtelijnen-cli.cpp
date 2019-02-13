#include <iostream>
#include <fstream>

#include <gdal_nodes.hpp>
#include <las_nodes.hpp>
#include <cgal_nodes.hpp>
#include <array>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int ac, const char * av[])
{
    std::string lines_file_in;
    std::string lines_file_out;
    std::string las_file;
    float selection_threshold = 0.5;
    float simplification_threshold = 10;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
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

    geoflow::NodeManager N;

    auto ogr_loader = std::make_shared<OGRLoaderNode>(N);
    auto tin_creator_lines = std::make_shared<CDTNode>(N);
    auto height_difference_calc = std::make_shared<PointDistanceNode>(N);
    auto tin_creator_difference = std::make_shared<CDTNode>(N);
    auto iso_lines = std::make_shared<IsoLineNode>(N);
    auto line_merger = std::make_shared<GEOSMergeLinesNode>(N);
    auto add_height_to_lines = std::make_shared<LineHeightNode>(N);
    auto tin_simp_lines = std::make_shared<TinSimpNode>(N);
    auto tin_simp_iso = std::make_shared<TinSimpNode>(N);
    auto simplify_lines_cdt = std::make_shared<SimplifyLinesNode>(N);
    auto ogr_writer = std::make_shared<OGRWriterNoAttributesNode>(N);

    std::strcpy(ogr_loader->filepath, lines_file_in.c_str());
    std::strcpy(height_difference_calc->filepath, las_file.c_str());
    std::strcpy(add_height_to_lines->filepath, las_file.c_str());
    std::strcpy(ogr_writer->filepath, lines_file_out.c_str());
    
    tin_creator_lines->create_triangles = true;
    height_difference_calc->thin_nth = 10;
    height_difference_calc->overwritez = true;
    add_height_to_lines->thin_nth = 10;
    tin_simp_lines->thres_error = selection_threshold;
    tin_simp_lines->densify_interval = selection_threshold;
    tin_simp_iso->thres_error = selection_threshold;
    tin_simp_iso->densify_interval = selection_threshold;
    simplify_lines_cdt->threshold_stop_cost = simplification_threshold;

    // iso line generation
    geoflow::connect(ogr_loader->outputs("line_strings"), tin_creator_lines->inputs("geometries"));;
    // calculate height difference tin-points
    geoflow::connect(tin_creator_lines->outputs("triangles"), height_difference_calc->inputs("triangles"));
    // make tin from height differences
    geoflow::connect(height_difference_calc->outputs("points"), tin_creator_difference->inputs("geometries"));
    // make iso lines
    geoflow::connect(height_difference_calc->outputs("distance_min"), iso_lines->inputs("min"));
    geoflow::connect(height_difference_calc->outputs("distance_max"), iso_lines->inputs("max"));
    geoflow::connect(tin_creator_difference->outputs("cgal_cdt"), iso_lines->inputs("cgal_cdt"));
    // merge iso line segments
    geoflow::connect(iso_lines->outputs("lines"), line_merger->inputs("lines"));
    // add height to lines
    geoflow::connect(line_merger->outputs("lines"), add_height_to_lines->inputs("lines"));
    // remove lines using tinsimp
    geoflow::connect(ogr_loader->outputs("line_strings"), tin_simp_lines->inputs("geometries"));
    geoflow::connect(add_height_to_lines->outputs("lines"), tin_simp_iso->inputs("geometries"));
    // simplify lines using CDT
    geoflow::connect(tin_simp_lines->outputs("selected_lines"), simplify_lines_cdt->inputs("lines"));
    geoflow::connect(tin_simp_iso->outputs("selected_lines"), simplify_lines_cdt->inputs("lines2"));
    // write lines
    geoflow::connect(simplify_lines_cdt->outputs("lines"), ogr_writer->inputs("geometries"));

    N.run(*ogr_loader);
}