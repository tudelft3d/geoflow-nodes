#include <lasreader.hpp>
#include <laswriter.hpp>

#include "las_nodes.hpp"

namespace geoflow::nodes::las {

void LASLoaderNode::process(){
  auto filepath = param<std::string>("filepath");
  auto thin_nth = param<int>("thin_nth");

  PointCollection points;
  vec1i classification;
  vec1f intensity;

  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(filepath.c_str());
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
    }
    if (i % 1000000 == 0) {
      std::cout << "Read " << i << " points...\n";
    }
  }
  lasreader->close();
  delete lasreader;

  output("points").set(points);
  output("classification").set(classification);
  output("intensity").set(intensity);
}

void LASGroundLoaderNode::process() {
  auto filepath = param<std::string>("filepath");
  auto thin_nth = param<int>("thin_nth");

  PointCollection points;
  vec1i classification;
  vec1f intensity;

  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(filepath.c_str());
  LASreader* lasreader = lasreadopener.open();
  if (!lasreader)
    return;

  // geometry.bounding_box.set(
  //   {float(lasreader->get_min_x()), float(lasreader->get_min_y()), float(lasreader->get_min_z())},
  //   {float(lasreader->get_max_x()), float(lasreader->get_max_y()), float(lasreader->get_max_z())}
  // );
  bool found_offset = manager.data_offset.has_value();

  size_t i = 0;
  while (lasreader->read_point()) {
    if (!found_offset) {
      manager.data_offset = { lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z() };
      found_offset = true;
    }

    if (lasreader->point.get_classification() == 2) {
      if (i++ % thin_nth == 0) {
        points.push_back({
          float(lasreader->point.get_x() - (*manager.data_offset)[0]),
          float(lasreader->point.get_y() - (*manager.data_offset)[1]),
          float(lasreader->point.get_z() - (*manager.data_offset)[2]) }
        );
      }
      if (i % 1000000 == 0) {
        std::cout << "Read " << i << " points...\n";
      }
    }
  }
  lasreader->close();
  delete lasreader;

  output("points").set(points);
}

void LASWriterNode::write_point_cloud_collection(PointCollection& point_cloud, std::string path) {
  LASwriteOpener laswriteopener;
  laswriteopener.set_file_name(path.c_str());

  LASheader lasheader;
  lasheader.x_scale_factor = 0.01;
  lasheader.y_scale_factor = 0.01;
  lasheader.z_scale_factor = 0.01;
  lasheader.x_offset = 0.0;
  lasheader.y_offset = 0.0;
  lasheader.z_offset = 0.0;
  lasheader.point_data_format = 0;
  lasheader.point_data_record_length = 20;

  LASpoint laspoint;
  laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

  LASwriter* laswriter = laswriteopener.open(&lasheader);
  if (laswriter == 0)
  {
    std::cerr << "ERROR: could not open laswriter\n";
    return;
  }

  // bool found_offset = manager.data_offset.has_value();

  size_t i=0;
  for (auto& p : point_cloud) {
    laspoint.set_x(p[0] + (*manager.data_offset)[0]);
    laspoint.set_y(p[1] + (*manager.data_offset)[1]);
    laspoint.set_z(p[2] + (*manager.data_offset)[2]);

    laswriter->write_point(&laspoint);
    laswriter->update_inventory(&laspoint);
    
    if((++i)%100000000==0) std::cout << "Wrtten " << i << " points...\n";
  } 

  laswriter->update_header(&lasheader, TRUE);
  laswriter->close();
  delete laswriter;
}

void LASWriterNode::process(){
  auto filepath = param<std::string>("filepath");

  auto input_geom = input("point_clouds");

  if (input_geom.connected_type == typeid(PointCollection)) {
    auto point_cloud = input_geom.get<PointCollection>();
    write_point_cloud_collection(point_cloud, filepath);

  } else if (input_geom.connected_type == typeid(std::vector<PointCollection>)) {
    auto point_clouds = input_geom.get< std::vector<PointCollection> >();

    int i=0;
    for (auto point_cloud : point_clouds) {
      filepath += "." + std::to_string(i++) + ".las";
      write_point_cloud_collection(point_cloud, filepath);
    }
  }
}

}