#pragma once

#include <unordered_map>
#include <deque>
#include <type_traits>

namespace regiongrower {

  using namespace std;

// typename std::enable_if<std::is_base_of<Implementation, T>::value, bool>::type 
  template <typename candidateDS, typename regionType> class RegionGrower {
    public:
    vector<size_t> region_ids;
    vector<regionType> regions;
    size_t min_segment_count=15;
    size_t cur_region_id=1;

    private:
    template <typename Tester> inline bool grow_one_region(candidateDS& cds, Tester& tester, size_t& seed_handle) {
      deque<size_t> candidates;
      vector<size_t> handles_in_region;
      regions.push_back(regionType());

      candidates.push_back(seed_handle);
      handles_in_region.push_back(seed_handle);
      region_ids[seed_handle] = cur_region_id;//regions.size();

      while (candidates.size()>0) {
        auto candidate = candidates.front(); candidates.pop_front();
        for (auto neighbour: cds.get_neighbours(candidate)) {
          if (region_ids[neighbour]!=0) continue;
          if (tester.is_valid(cds, candidate, neighbour, regions.back())) {
            candidates.push_back(neighbour);
            handles_in_region.push_back(neighbour);
            region_ids[neighbour] = cur_region_id;//regions.size();
          }
        }
      }
      // undo region if it doesn't satisfy quality criteria
      if (handles_in_region.size() < min_segment_count) {
        regions.erase(regions.end()-1);
        for (auto handle: handles_in_region)
          region_ids[handle] = 0;
        return false;
      } return true;
    };

    public:
    template <typename Tester> void grow_regions(candidateDS& cds, Tester& tester) {
      std::vector<size_t> new_regions;
      std::deque<size_t> seeds = cds.get_seeds();

      region_ids.resize(cds.size, 0);
      // first region means unsegmented
      regions.push_back(regionType());

      // region growing from seed points
      while (seeds.size()>0) {
        auto idx = seeds.front();
        seeds.erase(seeds.begin());
        if (region_ids[idx]==0) {
          grow_one_region(cds, tester, idx);
          ++cur_region_id;
        }
      }
    };
  };
}