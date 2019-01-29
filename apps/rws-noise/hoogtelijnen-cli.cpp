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
    auto tin_simp = std::make_shared<TinSimpNode>(N);
    auto simp_3d = std::make_shared<SimplifyLine3DNode>(N);
    auto ogr_writer = std::make_shared<OGRWriterNoAttributesNode>(N);
    
    // iso line generation
    auto tin_creator_lines = std::make_shared<CDTNode>(N);
    auto height_difference_calc = std::make_shared<PointDistanceNode>(N);
    auto tin_creator_difference = std::make_shared<CDTNode>(N);
    auto iso_lines = std::make_shared<IsoLineSlicerNode>(N);
    auto add_height_to_lines = std::make_shared<LineHeightNode>(N);
    auto simplify_lines = std::make_shared<SimplifyLine3DNode>(N);
    auto ogr_writer_1 = std::make_shared<OGRWriterNoAttributesNode>(N);
    //auto ogr_writer_2 = std::make_shared<OGRWriterNode>(N);

    std::strcpy(height_difference_calc->filepath, las_file.c_str());
    std::strcpy(ogr_loader->filepath, lines_file_in.c_str());
    std::strcpy(ogr_writer_1->filepath, lines_file_out.c_str());
    std::strcpy(add_height_to_lines->filepath, las_file.c_str());
    //std::strcpy(ogr_writer_2->filepath, "D:\\Projects\\3D Geluid\\Hoogtelijnen\\test-iso-segmenten.shp");

    height_difference_calc->thin_nth = 100;
    height_difference_calc->overwritez = true;
    add_height_to_lines->thin_nth = 100;

    //geoflow::connect(ogr_loader->outputs("line_strings"), tin_simp->inputs("geometries"));
    //geoflow::connect(tin_simp->outputs("selected_lines"), simp_3d->inputs("lines"));
    //geoflow::connect(simp_3d->outputs("lines"), ogr_writer->inputs("geometries"));

    // iso line generation
    geoflow::connect(ogr_loader->outputs("line_strings"), tin_creator_lines->inputs("geometries"));;
    // calculate height difference tin-points
    geoflow::connect(tin_creator_lines->outputs("triangles"), height_difference_calc->inputs("triangles"));
    // make tin from height differences
    geoflow::connect(height_difference_calc->outputs("points"), tin_creator_difference->inputs("geometries"));
    // make iso lines
    geoflow::connect(tin_creator_difference->outputs("cgal_cdt"), iso_lines->inputs("cgal_cdt"));
    // add height to lines
    geoflow::connect(iso_lines->outputs("lines"), add_height_to_lines->inputs("lines"));
    //simplify lines
    //geoflow::connect(add_height_to_lines->outputs("lines"), simplify_lines->inputs("lines"));

    // write lines
    geoflow::connect(add_height_to_lines->outputs("lines"), ogr_writer_1->inputs("geometries"));

    ////// create height map from height lines + iso lines difference with point cloud
    //// iso line generation
    //geoflow::connect(ogr_loader->outputs("line_strings"), tin_creator_lines->inputs("geometries"));;
    //// calculate height difference tin-points
    //geoflow::connect(tin_creator_lines->outputs("triangles"), height_difference_calc->inputs("triangles"));
    //// make tin from height differences
    //geoflow::connect(height_difference_calc->outputs("points"), tin_creator_difference->inputs("geometries"));
    //// make iso lines
    //geoflow::connect(tin_creator_difference->outputs("cgal_cdt"), iso_lines->inputs("cgal_cdt"));
    //// add height to lines
    //geoflow::connect(iso_lines->outputs("lines"), add_height_to_lines->inputs("lines"));



    //line simplification
    // https://doc.cgal.org/latest/Polyline_simplification_2/index.html#Subsection_PolylineSimplification_Simplifying_Several

    //tin_simp->thres_error = selection_threshold;
    //tin_simp->densify_interval = selection_threshold;
    //simp_3d->area_threshold = simplification_threshold;
    N.run(*ogr_loader);

    int stop = 0;
}