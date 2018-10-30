#include <iostream>
#include <fstream>

#include "imgui.h"
#include "app_povi.h"
#include "nodes.h"
#include "../src/stepedge_nodes.hpp"
#include "../src/gdal_nodes.hpp"
#include "../src/las_nodes.hpp"
#include "../src/cgal_nodes.hpp"
#include <array>

// #include <boost/program_options.hpp>

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

    geoflow::NodeManager N;

    auto ogr_loader = std::make_shared<OGRLoaderNode>(N);
    auto las_in_poly = std::make_shared<LASInPolygonsNode>(N);
    auto lod13generator = std::make_shared<LOD13GeneratorNode>(N);
    auto ogr_writer = std::make_shared<OGRWriterNode>(N);

    std::strcpy(ogr_loader->filepath, footprints_file.c_str());
    std::strcpy(las_in_poly->las_filepath, las_file.c_str());

    geoflow::connect(ogr_loader->outputTerminals["features"].get(), las_in_poly->inputTerminals["polygons"].get());
    geoflow::connect(ogr_loader->outputTerminals["features"].get(), lod13generator->inputTerminals["polygons"].get());
    geoflow::connect(las_in_poly->outputTerminals["point_clouds"].get(), lod13generator->inputTerminals["point_clouds"].get());
    geoflow::connect(lod13generator->outputTerminals["decomposed_polygons"].get(), ogr_writer->inputTerminals["features"].get());

    bool init = false;
    for (float step_threshold : {1.0, 2.0, 3.0, 4.0}) {
      std::string out_file = "out_" + std::to_string(step_threshold) + ".shp";
      std::strcpy(ogr_writer->filepath, out_file.c_str());
      lod13generator->step_threshold = step_threshold;
      if(!init){
        N.run(*ogr_loader);
        init=true;
      } else 
        N.run(*lod13generator);
    }
}