#include "imgui.h"
#include "geoflow.hpp"
#include <lasreader.hpp>

using namespace geoflow;

class LASLoaderNode:public Node {

  public:
  char filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
  int thin_nth = 20;
  bool use_thin = true;

  LASLoaderNode(NodeManager& manager):Node(manager) {
    add_output("geometry", TT_geometry);
    add_output("points", TT_vec3f);
    add_output("classification", TT_vec1i);
    add_output("intensity", TT_vec1f);
    GDALAllRegister();
  }

  void gui(){
    ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 1, 100);
  }

  void process(){
    gfGeometry3D geometry;
    geometry.type = geoflow::points;
    geometry.format = geoflow::simple;
    vec1i classification;
    vec1f intensity;

    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(filepath);
    LASreader* lasreader = lasreadopener.open();
    if (!lasreader)
      return;

    geometry.bounding_box.set(
      {float(lasreader->get_min_x()), float(lasreader->get_min_y()), float(lasreader->get_min_z())},
      {float(lasreader->get_max_x()), float(lasreader->get_max_y()), float(lasreader->get_max_z())}
    );

    size_t i=0;
    while (lasreader->read_point()) {

      if (i++ % thin_nth == 0) {
        classification.push_back(lasreader->point.get_classification());
        intensity.push_back(float(lasreader->point.get_intensity()));
        geometry.vertices.push_back({
          float(lasreader->point.get_x()), 
          float(lasreader->point.get_y()), 
          float(lasreader->point.get_z())}
        );
        if(i%100000000==0) std::cout << "Read " << i << " points...\n";
      }
    }
    lasreader->close();
    delete lasreader;

    set_value("points", geometry.vertices);
    set_value("geometry", geometry);
    set_value("classification", classification);
    set_value("intensity", intensity);
  }
};