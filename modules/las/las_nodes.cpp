#include "las_nodes.hpp"

using namespace geoflow;

void LASLoaderNode::process(){
  PointCollection points;
  vec1i classification;
  vec1f intensity;

  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(filepath);
  LASreader* lasreader = lasreadopener.open();
  if (!lasreader)
    return;

  // geometry.bounding_box.set(
  //   {float(lasreader->get_min_x()), float(lasreader->get_min_y()), float(lasreader->get_min_z())},
  //   {float(lasreader->get_max_x()), float(lasreader->get_max_y()), float(lasreader->get_max_z())}
  // );
  bool found_offset = manager.data_offset.has_value();

  size_t i=0;
  while (lasreader->read_point()) {
    if (!found_offset) {
      manager.data_offset = {lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z()};
      found_offset = true;
    }
    if (i++ % thin_nth == 0) {
      classification.push_back(lasreader->point.get_classification());
      intensity.push_back(float(lasreader->point.get_intensity()));
      points.push_back({
        float(lasreader->point.get_x() - (*manager.data_offset)[0]), 
        float(lasreader->point.get_y() - (*manager.data_offset)[1]), 
        float(lasreader->point.get_z() - (*manager.data_offset)[2])}
      );
      if(i%100000000==0) std::cout << "Read " << i << " points...\n";
    }
  }
  lasreader->close();
  delete lasreader;

  outputs("points").set(points);
  outputs("classification").set(classification);
  outputs("intensity").set(intensity);
}