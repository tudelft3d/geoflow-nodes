#include <iostream>
#include <fstream>
#include <exception>

#include <general_register.hpp>
#include <gdal_register.hpp>
#include <cgal_register.hpp>

#ifdef GF_BUILD_GUI
    #include <geoflow/gui/flowchart.hpp>
#endif

#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace gfn = geoflow::nodes;

int main(int ac, const char * av[])
{
    std::string footprint_file_in;
    bool gui = false;
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    #ifdef GF_BUILD_GUI
        ("gui", po::bool_switch(&gui), "launch gui")
    #endif
    ("footprints_file_in", po::value<std::string>(&footprint_file_in), "Input footprints")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    auto general = gfn::general::create_register();
    auto cgal = gfn::cgal::create_register();
    auto gdal = gfn::gdal::create_register();
    geoflow::NodeManager N;

    auto ogr_loader = N.create_node(gdal, "OGRLoader", { 0, 0 });
    auto line_strings = N.create_node(general, "PolygonToLineString", { 0, 0 });
    auto line_equations = N.create_node(general, "CreateLineEquations", { 0, 0 });
    auto line_buffers = N.create_node(general, "CreateLineBuffers", { 0, 0 });

    // Setup nodes
    ogr_loader->set_param("filepath", footprint_file_in);

    try {
        geoflow::connect(ogr_loader->output("linear_rings"), line_strings->input("linear_rings"));
        geoflow::connect(line_strings->output("line_strings"), line_equations->input("line_strings"));
        geoflow::connect(line_equations->output("line_equations"), line_buffers->input("line_equations"));
        
        #ifdef GF_BUILD_GUI
            if (gui)
                geoflow::launch_flowchart(N, {general, cgal, gdal});
            else {
                N.run(*ogr_loader);
            }
        #else
            N.run(*ogr_loader);
        #endif
    } catch (const std::exception e) {
      std::cout << e.what() << std::endl;
    }
}