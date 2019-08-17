#include "general_nodes.hpp"

#include <fstream>
#include <sstream>
#include <set>
#include <vector>
#include <map>

namespace geoflow::nodes::general {
  void MergeGeometriesNode::process() {
    auto geometries1 = input("geometries1");
    auto geometries2 = input("geometries2");


    if (geometries1.is_connected_type(typeid(PointCollection))) {
      auto mergedGeometries = geometries1.get<geoflow::PointCollection>();
      auto points2 = geometries2.get<geoflow::PointCollection>();
      for (auto& p : points2) {
        mergedGeometries.push_back(p);
      }
      output("geometries").set(mergedGeometries);
    }
    else if (geometries1.is_connected_type(typeid(SegmentCollection))) {
      auto mergedGeometries = geometries1.get<geoflow::SegmentCollection>();
      auto segments2 = geometries2.get<geoflow::SegmentCollection>();
      for (auto& s : segments2) {
        mergedGeometries.push_back(s);
      }
      output("geometries").set(mergedGeometries);
    }
    else if (geometries1.is_connected_type(typeid(LineStringCollection))) {
      auto mergedGeometries = geometries1.get<geoflow::LineStringCollection>();
      auto lines2 = geometries2.get<geoflow::LineStringCollection>();
      for (auto& l : lines2) {
        mergedGeometries.push_back(l);
      }
      output("geometries").set(mergedGeometries);
    }
    else if (geometries1.is_connected_type(typeid(LinearRingCollection))) {
      auto mergedGeometries = geometries1.get<geoflow::LinearRingCollection>();
      auto rings2 = geometries2.get<geoflow::LinearRingCollection>();
      for (auto& r : rings2) {
        mergedGeometries.push_back(r);
      }
      output("geometries").set(mergedGeometries);
    } else {
      throw new gfException("Geometry inputs must be of same type\n");
    }
  }
  
  void MergeLinestringsNode::process() {
    auto geometries1 = input("lines1");
    auto geometries2 = input("lines2");

    std::cout << "Merging two sets of line strings\n";

    auto mergedLines = geometries1.get<geoflow::LineStringCollection>();
    auto lines2 = geometries2.get<geoflow::LineStringCollection>();
    for (auto& l : lines2) {
      mergedLines.push_back(l);
    }
    output("lines").set(mergedLines);
  }

  void LineStringFilterNode::process() {
    auto line_strings = input("line_strings").get<geoflow::LineStringCollection>();

    std::cout << "Filtering " << line_strings.size() << " lines\n";
    LineStringCollection out_line_strings;
    if (filter_length > 0.0) {
      for (auto& l : line_strings) {
        float length = 0.0;
        float length2 = 0.0;
        for (int i = 0; i < l.size() - 1; i++) {
          //float x1 = l[i][0];
          //float x2 = l[i + 1][0];
          //float y1 = l[i][1];
          //float y2 = l[i + 1][1];
          //length2 += sqrt(((x2 - x1)*(x2 - x1)) + ((y2 - y1) * (y2 - y1)));
          length += sqrt(((l[i + 1][0] - l[i][0])*(l[i + 1][0] - l[i][0])) + ((l[i + 1][1] - l[i][1]) * (l[i + 1][1] - l[i][1])));
        }
        if (length > filter_length) {
          out_line_strings.push_back(l);
        }
      }
    }
    std::cout << "Filtered " << line_strings.size() - out_line_strings.size() << " lines\n";

    output("line_strings").set(out_line_strings);
  }

  void OBJwriterNode::process() {

    auto& triangles = input("triangles").get<TriangleCollection&>();

    std::map<arr3f, size_t> vertex_map;
    std::vector<arr3f> vertex_vec;
    // std::vector<std::tuple<size_t,size_t,size_t>> face_indices;
    {
      size_t v_cntr=1;
      std::set<arr3f> vertex_set;
      for (auto& triangle : triangles) {
        for (auto& vertex : triangle) {
          auto [it, did_insert] = vertex_set.insert(vertex);
          if (did_insert) {
            vertex_map[vertex] = v_cntr++;
            vertex_vec.push_back(vertex);
          }
        }
      }
    }
    std::ofstream ofs;
    ofs.open(filepath);
    // ofs << mtllib << std::endl;
    ofs << std::fixed << std::setprecision(3);
    for (auto& v : vertex_vec) {
        ofs << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }
    // ofs << "o " << uuid << std::endl;
    // ofs << material << std::endl;
    for (auto& triangle : triangles) {
      ofs << "f " << vertex_map[triangle[0]] << " " << vertex_map[triangle[1]] << " " << vertex_map[triangle[2]] << std::endl;
    }
    ofs.close();
  }
}