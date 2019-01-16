#include <queue>
#include <stack>
#include "region_growing.h"

using namespace std;
using namespace linedect;

LineDetector::LineDetector(vector<Point> &points) {
  auto size = points.size();
  indexed_points.reserve(size);
  size_t i=0;
  for(auto p: points)
    indexed_points.push_back(std::make_pair(p,i++));
  point_segment_idx.resize(size, 0);
  // point_seed_flags.resize(size, true);
  tree.insert(indexed_points.begin(), indexed_points.end());
}

vector<size_t> LineDetector::get_point_indices(size_t shape_id) {
  vector<size_t> result;
  for (auto pi:indexed_points){
    auto idx = pi.second;
    if (point_segment_idx[idx] == shape_id)
      result.push_back(idx);
  }
  return result;
}

inline Line LineDetector::fit_line(Neighbor_search search_result){
  vector<Point> neighbor_points;
  for (auto neighbor: search_result)
    neighbor_points.push_back(neighbor.first.first);
  Line line;
  linear_least_squares_fitting_3(neighbor_points.begin(),neighbor_points.end(),line,CGAL::Dimension_tag<0>());
  return line;
}

void LineDetector::detect(){
  // seed generation
  typedef pair<size_t,double> index_dist_pair;
  auto cmp = [](index_dist_pair left, index_dist_pair right) {return left.second < right.second;};
  priority_queue<index_dist_pair, vector<index_dist_pair>, decltype(cmp)> pq(cmp);

  size_t i=0;
  for(auto pi : indexed_points){
    auto p = pi.first;
    Neighbor_search search(tree, p, N);
    auto line = fit_line(search);
    auto line_dist = CGAL::squared_distance(line, p);
    pq.push(index_dist_pair(i++, line_dist));
  }

  // region growing from seed points
  while(pq.size()>0){
    auto idx = pq.top().first; pq.pop();
    // if (point_seed_flags[idx]){
    if (point_segment_idx[idx]==0){
      grow_region(idx);
      region_counter++;
    }
  }
}

inline bool LineDetector::valid_candidate(Line &line, Point &p) {
  return CGAL::squared_distance(line, p) < dist_thres;
}

void LineDetector::grow_region(size_t seed_idx){
  auto p = indexed_points[seed_idx];
  Neighbor_search search_init(tree, p.first, N);
  auto line = fit_line(search_init);
  segment_shapes[region_counter] = line;

  vector<Point> points_in_region;
  vector<size_t> idx_in_region;
  stack<size_t> candidates;
  candidates.push(seed_idx);
  point_segment_idx[seed_idx] = region_counter;
  points_in_region.push_back(p.first); 
  idx_in_region.push_back(p.second); 
  
  while (candidates.size()>0){
    auto seed_id = candidates.top(); candidates.pop();
    auto cp = indexed_points[seed_id].first;
    Neighbor_search search(tree, cp, N);
    for (auto neighbor: search){
      auto n_id = neighbor.first.second;
      if (point_segment_idx[n_id]!=0)
        continue;
      // point_seed_flags[n_id] = false; // this point can no longer be used as seed
      if (valid_candidate(segment_shapes[region_counter], neighbor.first.first)){
        point_segment_idx[n_id] = region_counter;
        candidates.push(n_id);
        points_in_region.push_back(neighbor.first.first);
        idx_in_region.push_back(n_id);
        Line line;
        linear_least_squares_fitting_3(points_in_region.begin(),points_in_region.end(),line,CGAL::Dimension_tag<0>());
        segment_shapes[region_counter] = line;
      }

    }
  }
  // undo region if it doesn't satisfy quality criteria
  if (points_in_region.size()<min_segment_count){
    segment_shapes.erase(region_counter);
    for (auto idx: idx_in_region)
      point_segment_idx[idx] = 0;
  }
}
