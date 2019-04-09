#include "general_nodes.hpp"

namespace geoflow::nodes::general {
  Buffer::Buffer(Line &l) {
    a = l.a;
    b = l.b;
    bmin = l.b;
    bmax = l.b;
    id = l.id;
    length = l.length;
    parallelLines.push_back(l);
  }

  void MergeGeometriesNode::process() {
    auto geometries1 = input("geometries1");
    auto geometries2 = input("geometries2");

    if (geometries1.connected_type != geometries2.connected_type) {
      throw new Exception("Geometry inputs must be of same type\n");
    }

    if (geometries1.connected_type == typeid(PointCollection)) {
      auto mergedGeometries = geometries1.get<geoflow::PointCollection>();
      auto points2 = geometries2.get<geoflow::PointCollection>();
      for (auto& p : points2) {
        mergedGeometries.push_back(p);
      }
      output("geometries").set(mergedGeometries);
    }
    else if (geometries1.connected_type == typeid(SegmentCollection)) {
      auto mergedGeometries = geometries1.get<geoflow::SegmentCollection>();
      auto segments2 = geometries2.get<geoflow::SegmentCollection>();
      for (auto& s : segments2) {
        mergedGeometries.push_back(s);
      }
      output("geometries").set(mergedGeometries);
    }
    else if (geometries1.connected_type == typeid(LineStringCollection)) {
      auto mergedGeometries = geometries1.get<geoflow::LineStringCollection>();
      auto lines2 = geometries2.get<geoflow::LineStringCollection>();
      for (auto& l : lines2) {
        mergedGeometries.push_back(l);
      }
      output("geometries").set(mergedGeometries);
    }
    else if (geometries1.connected_type == typeid(LinearRingCollection)) {
      auto mergedGeometries = geometries1.get<geoflow::LinearRingCollection>();
      auto rings2 = geometries2.get<geoflow::LinearRingCollection>();
      for (auto& r : rings2) {
        mergedGeometries.push_back(r);
      }
      output("geometries").set(mergedGeometries);
    }
  }
  
  void MergeLinestringsNode::process() {
    auto geometries1 = input("lines1");
    auto geometries2 = input("lines2");

    std::cout << "Merging two sets of linestring strings\n";

    auto mergedLines = geometries1.get<geoflow::LineStringCollection>();
    auto lines2 = geometries2.get<geoflow::LineStringCollection>();
    for (auto& l : lines2) {
      mergedLines.push_back(l);
    }
    output("lines").set(mergedLines);
  }

  void LineStringFilterNode::process() {
    auto line_strings = input("line_strings").get<geoflow::LineStringCollection>();

    auto filter_length = param<float>("filter_length");

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

  void PolygonToLineStringNode::process() {
    auto polygons = input("linear_rings").get<geoflow::LinearRingCollection>();
    
    LineStringCollection out_line_strings;
    // for now only convert first polygon to linestrings
    int p = 0;
    LinearRing polygon = polygons[p];
    
    for (int i = 1; i < polygon.size(); i++) {
      out_line_strings.push_back(LineString({ polygon[i - 1], polygon[i] }));
    }

    output("line_strings").set(out_line_strings);
  }

  void CreateLineEquationsNode::process() {
    auto line_strings = input("line_strings").get<geoflow::LineStringCollection>();
    std::vector<Line> equations;

    for (int i = 0; i < line_strings.size(); i++) {
      vec3f linestring = line_strings[i];
      Line line;
      line.p1 = linestring[0];
      line.p2 = linestring[1];

      // calculate a
      if (fabs(line.p2[0] - line.p1[0]) < std::numeric_limits<double>::epsilon()) {
        if (line.p2[1] < line.p1[1]) line.a = 9999.0;	//Line is oriented downwards
        else line.a = -9999;							            //Line is oriented upwards
      }
      else {
        line.a = (line.p2[1] - line.p1[1]) / (line.p2[0] - line.p1[0]);
      }
      // calculate b
      line.b = line.p1[1] - line.a * line.p1[0];

      // calculate length
      line.length = sqrt(pow(line.p2[0] - line.p1[0], 2) + pow(line.p2[1] - line.p1[1], 2));
      equations.push_back(line);
    }
    output("line_equations").set(equations);
  }

  bool compareLength(Line l1, Line l2) {
    return (l1.length < l2.length);
  }

  double minBBuffer(double A, Buffer buffer) {
    double tempB = 0, minB = 999999;

    for (std::vector<Line>::iterator line = buffer.parallelLines.begin(); line != buffer.parallelLines.end(); ++line) {
        tempB = line->p1[1] - A * line->p1[0];
        if (tempB < minB) minB = tempB;
        tempB = line->p2[1] - A * line->p2[0];
        if (tempB < minB) minB = tempB;
    }
    return minB;
  }

  double maxBBuffer(double A, Buffer buffer) {
    double tempB = 0, maxB = -999999;

    for (std::vector<Line>::iterator line = buffer.parallelLines.begin(); line != buffer.parallelLines.end(); ++line) {
      tempB = line->p1[1] - A * line->p1[0];
      if (tempB > maxB) maxB = tempB;
      tempB = line->p2[1] - A * line->p2[0];
      if (tempB > maxB) maxB = tempB;
    }
    return maxB;
  }

  bool joinableBuffers(Buffer &buffer1, Buffer &buffer2, double epsilon, double alpha) {
    double angle = abs(atan(buffer1.a) - atan(buffer2.a));
    alpha = alpha / (180 / PI); //From degrees to radians

    if (angle < alpha) {
      double weight1 = buffer1.length / (buffer1.length + buffer2.length);
      double weight2 = buffer2.length / (buffer1.length + buffer2.length);

      //Update Ax+B
      double tempA = tan(atan(buffer1.a) * weight1 + atan(buffer2.a) * weight2);

      double minB = std::min(minBBuffer(tempA, buffer1) , minBBuffer(tempA, buffer2));
      double maxB = std::max(maxBBuffer(tempA, buffer1), maxBBuffer(tempA, buffer2));

      if (abs((maxB - minB) * cos(atan(tempA))) < epsilon) {
        return true;
      }
    }
    return false;
  }

  void joinBuffers(Buffer &buffer1, Buffer &buffer2) {
    // copy lines to other buffer
    buffer1.parallelLines.insert(buffer1.parallelLines.end(),
      buffer2.parallelLines.begin(),
      buffer2.parallelLines.end());
    buffer1.nonParallelLines.insert(buffer1.nonParallelLines.end(),
      buffer2.nonParallelLines.begin(),
      buffer2.nonParallelLines.end());

    // calculate line weights
    double weight1 = buffer1.length / (buffer1.length + buffer2.length);
    double weight2 = buffer2.length / (buffer1.length + buffer2.length);

    //Update Ax+B
    buffer1.a = tan(atan(buffer1.a) * weight1 + atan(buffer2.a) * weight2);
    buffer1.bmin = minBBuffer(buffer1.a, buffer1);
    buffer1.bmax = maxBBuffer(buffer1.a, buffer1);
    
    // update length
    buffer1.length = buffer1.length + buffer2.length;
  }

  LineString createBufferLineString(double a, double b, float x1, float x2) {
    LineString bufferline;
    float y1 = a * x1 + b;
    float y2 = a * x2 + b;

    bufferline.push_back({ x1, y1 });
    bufferline.push_back({ x2, y2 });
    return bufferline;
  }

  void CreateLineBuffersNode::process() {
    auto line_equations = input("line_equations").get<std::vector<Line>>();
    auto alpha = param<float>("alpha");
    auto epsilon = param<float>("epsilon");
    std::vector<Buffer> buffers;
    
    // sort lines on importance
    std::sort(line_equations.begin(), line_equations.end(), compareLength);
    
    // create buffers from line equations
    for (auto &line_equation : line_equations) {
      buffers.push_back(Buffer(line_equation));
    }

    // join buffers
    int i = 0;
    //Test if buffers can be joined based on alpha and epsilon
    while (i < buffers.size() - 1) {
      for (int b = i + 1; b < buffers.size() - 1;) {
        if (joinableBuffers(buffers[i], buffers[b], epsilon, alpha)) {
          joinBuffers(buffers[i], buffers[b]);
          buffers.erase(buffers.begin() + b);
        }
        else b++;
      }
      i++;
    }

    LineStringCollection bufferlines;
    for (auto buffer : buffers) {
      float minx = 999999, maxx = -999999;
      for (auto l : buffer.parallelLines) {
        minx = std::min(minx, std::min(l.p1[0], l.p2[0]));
        maxx = std::max(maxx, std::max(l.p1[0], l.p2[0]));
      }
      bufferlines.push_back(createBufferLineString(buffer.a, buffer.bmin, minx, maxx));
    }

    output("line_equations").set(buffers);
    output("line_strings").set(buffers);
  }
}