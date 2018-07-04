#include "region_growing.h"
#include <iostream>

using namespace std;
using namespace linedect;

int main() {
  std::vector<Point> points;
  for (int i=0; i<100 ;i++){
    points.push_back(Point(i,0,0));
  }
  for (int i=0; i<100 ;i++){
    points.push_back(Point(0,i,0));
  }
  for (int i=0; i<100 ;i++){
    points.push_back(Point(0,0,10+i));
  }
  auto LD = LineDetector(points);
  LD.dist_thres = 1;
  LD.detect();
  
  for (auto s : LD.segment_shapes){
    cout << s.first << " | " << s.second.direction() << endl; 
    for (auto pid : LD.get_point_indices(s.first))
      cout << points[pid] << endl;
  }
  return 0;
}
