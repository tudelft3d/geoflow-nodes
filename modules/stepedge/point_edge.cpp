#include "point_edge.h"
#include "region_growing_plane.h"
#include <CGAL/Arr_walk_along_line_point_location.h>
#include <unordered_map>
#include <boost/tuple/tuple.hpp>

Polygon_2 ring_to_cgal_polygon(geoflow::LinearRing& ring) {
  std::vector<Point_2> footprint_pts;
  for (auto p : ring) {
    footprint_pts.push_back(Point_2(p[0], p[1]));
  }
  return Polygon_2(footprint_pts.begin(), footprint_pts.end());
}

void pc_in_footprint(std::string las_filename, std::vector<bg::model::polygon<point_type>> &footprints, std::vector<PNL_vector> &points_vec) {
  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(las_filename.c_str());
  LASreader* lasreader = lasreadopener.open();

  for (auto footprint:footprints)
    points_vec.push_back(PNL_vector());

  while (lasreader->read_point()) {
    if (lasreader->point.get_classification() == 6){
      point_type p;
      p.set<0>(lasreader->point.get_x());
      p.set<1>(lasreader->point.get_y());
      // p.set<2>(lasreader->point.get_z());
      int i=0;
      for (auto footprint:footprints) {
        if (bg::within(p,footprint)){
          PNL pv;
          boost::get<0>(pv) = Point(lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z());
          points_vec[i].push_back(pv);
          break;
          // laswriter->write_point(&lasreader->point);
        }
        i++;
      }
    }
  }
  lasreader->close();
  delete lasreader;
}

void compute_metrics(PNL_vector &points, config c) {

  if(points.size()<c.metrics_plane_min_points) return;

  // Estimates normals
  // Note: pca_estimate_normals() requiresa range of points
  // as well as property maps to access each point's position and normal.
  // const int nb_neighbors = 5; // K-nearest neighbors = 3 rings
  CGAL::pca_estimate_normals<Concurrency_tag>
    (points, c.metrics_normal_k,
    CGAL::parameters::point_map(Point_map()).
    normal_map(Normal_map()));
  // Orients normals.
  // Note: mst_orient_normals() requires a range of points
  // as well as property maps to access each point's position and normal.
  // PNL_vector::iterator unoriented_points_begin =
    // CGAL::mst_orient_normals(points, c.metrics_normal_k,
                              // CGAL::parameters::point_map(Point_map()).
                              // normal_map(Normal_map()));
  // Optional: delete points with an unoriented normal
  // if you plan to call a reconstruction algorithm that expects oriented normals.
  // points.erase(unoriented_points_begin, points.end());

  // Orient normals quick n dirty and predictable way
  auto up = Vector(0,0,1);
  for ( auto& pv : points) {
    auto &n = boost::get<1>(pv);
    if (n*up<0) 
      boost::get<1>(pv) = -n;
  }


  size_t i=0;
  std::vector<Point> points_vec;
  std::vector<Vector> normals_vec;
  points_vec.reserve(points.size());
  for (auto &p : points) {
    points_vec.push_back(boost::get<0>(p));
    normals_vec.push_back(boost::get<1>(p));
    boost::get<8>(p) = i++;
  }

  planedect::PlaneDetector PD(points_vec, normals_vec);
  PD.dist_thres = c.metrics_plane_epsilon * c.metrics_plane_epsilon;
  PD.normal_thres = c.metrics_plane_normal_threshold;
  PD.min_segment_count = c.metrics_plane_min_points;
  PD.N = c.metrics_normal_k;
  PD.detect();
  // std::cout << PD.segment_shapes.size() << " shapes detected." << std::endl;

  // // Instantiates shape detection engine.
  // Region_growing shape_detection;

  // // Sets parameters for shape detection.
  // Region_growing::Parameters parameters;
  // // Sets probability to miss the largest primitive at each iteration.
  // // parameters.probability = 0.05;
  // // Detect shapes with at least 500 points.
  // parameters.min_points = c.metrics_plane_min_points;
  // // Sets maximum Euclidean distance between a point and a shape.
  // parameters.epsilon = c.metrics_plane_epsilon;
  // // Sets maximum Euclidean distance between points to be clustered.
  // // parameters.cluster_epsilon = 0.01;
  // // Sets maximum normal deviation.
  // // 0.9 < dot(surface_normal, point_normal); 
  // parameters.normal_threshold = c.metrics_plane_normal_threshold;

  // // Provides the input data.
  // shape_detection.set_input(points);
  // // Registers planar shapes via template method.
  // shape_detection.add_shape_factory<SCPlane>();
  // // Detects registered shapes with parameters.
  // std::cout << "points.size: " << points.size() << "\n";
  // shape_detection.detect(parameters);
  // // Prints number of detected shapes.
  // std::cout << shape_detection.shapes().end() - shape_detection.shapes().begin() << " shapes detected." << std::endl;

  i=1;
  for(auto seg: PD.segment_shapes){
    auto& plane = seg.second;
    auto plane_idx = PD.get_point_indices(seg.first);
    // this dot product is close to 0 for vertical planes
    // SCPlane* plane = dynamic_cast<SCPlane*>(shape.get());
    // std::cout << shape->info() << std::endl;
    Vector n = plane.orthogonal_vector();
    // std::cout << n*Vector(0,0,1) << std::endl;
    auto horizontality = CGAL::abs(n*Vector(0,0,1));
    bool is_wall = horizontality < c.metrics_is_wall_threshold;
    bool is_horizontal = horizontality > c.metrics_is_horizontal_threshold;
    
    // store cluster id's and is_wall as point attributes
    int label = i++;
    for(auto ind : plane_idx){
      boost::get<2>(points[ind])=label;
      boost::get<3>(points[ind])=is_wall;
      boost::get<9>(points[ind])=is_horizontal;
      // std::cout <<  << std::endl;
    }

    if(!is_wall){
      const unsigned int N = c.metrics_k_linefit;
      PNL_vector cluster_points;
      for(auto ind : plane_idx){
        cluster_points.push_back(points[ind]);
      }
      Tree tree(cluster_points.begin(), cluster_points.end());
      // Point_d query(0,0);
      // Select candidate edge point based on line fit through k-neighborhood
      for(auto ind : plane_idx){
        Point q(boost::get<0>(points[ind]));
        Neighbor_search search(tree, q, N);
        std::vector<Point_SCK> neighbor_points;
        for (auto neighbor: search)
          neighbor_points.push_back(Point_SCK(boost::get<0>(neighbor.first).x(), boost::get<0>(neighbor.first).y(), boost::get<0>(neighbor.first).z()));
        Line_SCK line;
        linear_least_squares_fitting_3(neighbor_points.begin(),neighbor_points.end(),line,CGAL::Dimension_tag<0>());
        auto line_dist = CGAL::squared_distance(line, Point_SCK(q.x(), q.y(), q.z()));
        boost::get<4>(points[ind]) = line_dist;
      }
      
    }
  }

  // detect height jumps
  PNL_vector roof_points;
  for (auto p : points){
    bool is_wall = boost::get<3>(p);
    bool is_on_plane = boost::get<2>(p) != 0;
    if ((!is_wall) && is_on_plane){
      auto p_ = p;
      boost::get<0>(p_)= Point(boost::get<0>(p).x(), boost::get<0>(p).y(), 0);
      roof_points.push_back(p_);
    }
  }
  Tree tree(roof_points.begin(), roof_points.end());
  const unsigned int N = c.metrics_k_jumpcnt_elediff;
  for (auto q : points){
    bool is_wall = boost::get<3>(q);
    bool is_on_plane = boost::get<2>(q) != 0;
    int jump_cnt = 0;
    double jump_ele=0;
    if ((!is_wall) && is_on_plane){
      auto q_segment = boost::get<2>(q);
      auto q_ = Point(boost::get<0>(q).x(), boost::get<0>(q).y(), 0);
      Neighbor_search search(tree, q_, N);
      for (auto neighbor: search){
        auto np = neighbor.first;
        auto np_segment = boost::get<2>(np);
        if (q_segment!=np_segment)
          jump_cnt++;
        auto ele_diff = CGAL::abs(boost::get<0>(points[boost::get<8>(np)]).z()-boost::get<0>(points[boost::get<8>(q)]).z());
        jump_ele = std::max(ele_diff, jump_ele);
      }
      boost::get<5>(points[boost::get<8>(q)]) = jump_cnt;
      boost::get<7>(points[boost::get<8>(q)]) = jump_ele;
    }
    
  }
}

void classify_edgepoints(std::vector<linedect::Point> &edge_points, PNL_vector &points, config c) {
  for (auto &p : points){
    double line_dist = boost::get<4>(p);
    double jump_count = boost::get<5>(p);
    double jump_ele = boost::get<7>(p);
    // std::cout << line_dist << " " << jump_count << " " << jump_ele << " " << p.get<8>() << " " << p.get<3>() << " " << p.get<2>() << std::endl;
    // if (line_dist > 0.01){// ie we assume the point to be on an edge of this cluster
    bool is_step = (jump_count >= c.classify_jump_count_min && jump_count <= c.classify_jump_count_max) && (line_dist > c.classify_line_dist) && (jump_ele > c.classify_jump_ele);
    // bool is_step = (jump_count >= 1 && jump_count <= 5) && line_dist > 0.01;
    // bool is_step = (jump_count >= 1) && line_dist > 0.01;
    // bool is_step = line_dist > 0.01;
    boost::get<6>(p) = is_step;
    if (is_step){// ie we assume the point to be on an edge of this cluster
      // edge_points.push_back(p); // add this point to a new set of candidate points for region-growing line-fitting
      // project to z=0 - seems to yield more reliable line detection
      edge_points.push_back(
        // linedect::Point(p.get<0>().x(),p.get<0>().y(),0)
        linedect::Point(boost::get<0>(p).x(), boost::get<0>(p).y(), boost::get<0>(p).z())
      ); // add this point to a new set of candidate points for region-growing line-fitting
    }
  }
}

void detect_lines(std::vector<std::pair<Point,Point>> & edge_segments, std::vector<linedect::Point> &edge_points, config c) {
  // line fitting in set of candidate edgepoints
  linedect::LineDetector LD(edge_points);
  LD.dist_thres = c.linedetect_dist_threshold * c.linedetect_dist_threshold;
  LD.min_segment_count = c.linedetect_min_segment_count;
  LD.N = c.linedetect_k;
  LD.detect();

  // std::cout << LD.segment_shapes.size() << " lines detected." << std::endl;
  //  std::vector<int> &edges_index_map,
  
  int line_id=0;
  for(auto seg: LD.segment_shapes){
    auto l = seg.second;
    auto l_idx = LD.get_point_indices(seg.first);
    // std::cout << l.direction() << ", #Pts: " << l_idx.size() <<std::endl;
    //find the two extreme points on this line
    double minpl=1, maxpl=0;
    size_t j=0;
    auto point_on_line = l.point(0);
    auto l_normal = l.to_vector()/CGAL::sqrt(l.to_vector().squared_length());
    for(auto lind : l_idx){
      // edges_index_map[lind] = line_id;
      // project this_p on l
      linedect::Point this_p(edge_points[lind]);
      const Vector a(point_on_line, this_p);
      double pl = a*l_normal;

      if (j++==0){
        minpl = maxpl = pl;
      } else {
        if (pl < minpl) 
          minpl=pl;
        else if (pl > maxpl) 
          maxpl=pl;
      }
    }
    line_id++;
    Point p0 = (point_on_line + minpl*l_normal);
    Point p1 = (point_on_line + maxpl*l_normal);
    edge_segments.push_back(std::make_pair(p0,p1));
  }
}

void build_arrangement(geoflow::LinearRing &footprint, geoflow::LineStringCollection & edge_segments, Arrangement_2 &arr, bool remove_unsupported){
  Face_split_observer obs (arr);
  // const double s = 100;

  // insert footprint segments
  std::vector<Point_2> footprint_pts;
  for (auto p : footprint) {
    footprint_pts.push_back(Point_2(p[0], p[1]));
  }
  // footprint_pts.pop_back(); // get rid of repeated vertex in boost polygon
  Polygon_2 cgal_footprint(footprint_pts.begin(), footprint_pts.end());
  // std::cout << "fp size=" <<footprint_pts.size() << "; " << footprint_pts[0].x() <<","<<footprint_pts[0].y()<<"\n";
  insert_non_intersecting_curves(arr, cgal_footprint.edges_begin(), cgal_footprint.edges_end());

  // for (auto edge : arr.edge_handles()) {
  //   edge->data().is_touched = true;
  //   edge->data().is_footprint = true;
  //   edge->twin()->data().is_touched = true;
  //   edge->twin()->data().is_footprint = true;
  // }
  // std::cout << arr.number_of_faces() <<std::endl;
  // for (auto face: arr.face_handles()){
  //   if(face->is_unbounded())
  //     std::cout << "unbounded face, id: " << face->data() << std::endl;
  //   else
  //     std::cout << "Bounded face, id: " << face->data() << std::endl;
  // }


  // insert step-edge lines
  // std::vector<std::pair<Plane,bool>> wall_planes;
  // std::vector<X_monotone_curve_2> lines;
  for (auto s : edge_segments) {
    // wall_planes.push_back(std::make_pair(Plane(s.first, s.second, s.first+Vector(0,0,1)),0));
    const Kernel::Point_2 a(s[0][0],s[0][1]);
    const Kernel::Point_2 b(s[1][0],s[1][1]);
    insert(arr, Line_2(Point_2(s[0][0],s[0][1]), Point_2(s[1][0],s[1][1])));
    
    // if (remove_unsupported) for (auto edge : arr.edge_handles()) {
    //   if (!edge->data().is_touched) {
    //     auto v = (b-a)/2;
    //     edge->data().c = a+v;
    //     edge->data().halfdist_sq = v.squared_length();
    //     edge->data().is_touched = true;
    //     edge->twin()->data().c = a+v;
    //     edge->twin()->data().halfdist_sq = v.squared_length();
    //     edge->twin()->data().is_touched = true;
    //   }
    // }
  }

  // remove edges that do not overlap with their detected edge
  // if (remove_unsupported) {
  //   std::vector<Arrangement_2::Halfedge_handle> edges;
  //   for (auto edge : arr.edge_handles()) {
  //     if (!edge->data().is_footprint && !(edge->source()->is_at_open_boundary() || edge->target()->is_at_open_boundary())) {
  //       auto s = Kernel::Point_2(CGAL::to_double(edge->source()->point().x()), CGAL::to_double(edge->source()->point().y()));
  //       auto t = Kernel::Point_2(CGAL::to_double(edge->target()->point().x()), CGAL::to_double(edge->target()->point().y()));
  //       auto c = edge->data().c;
  //       auto d = edge->data().halfdist_sq;
        
  //       auto left = s-c;
  //       auto right = t-c;
  //       bool partial_overlap = (left.squared_length() < d) || (right.squared_length() < d);
  //       bool c_inbetween_st = left*right < 0; //should also check d, but not needed in combination with partial_overlap
  //       if (!(partial_overlap || c_inbetween_st)) {
  //         edges.push_back(edge);
  //       }
  //     }
  //   }
  //   for (auto edge : edges) {
  //     arr.remove_edge(edge); 
  //   }
  // }
}

// void build_arrangement(geoflow::LinearRing &footprint, geoflow::LinearRingCollection & rings, Arrangement_2 &arr, geoflow::vec1i& plane_idx, bool remove_unsupported){

// }

void arrangementface_to_polygon(Face_handle face, vec2f& polygons){
  // if(extract_face){ // ie it is a face on the interior of the footprint
  auto he = face->outer_ccb();
  auto first = he;

  while(true){
    // if (!he->source()- at_infinity())
      polygons.push_back({
        float(CGAL::to_double(he->source()->point().x())),
        float(CGAL::to_double(he->source()->point().y()))
      });

    he = he->next();
    if (he==first) break;
  // }
  }
}

// inline void merge_faces(Face_handle f1, Face_handle f2) {
//   // we modify both faces since we don't know which one will remain (should look again at the arrangement observer class...)
//   auto count1 = f1->data().segid_count;
//   auto count2 = f2->data().segid_count;
//   auto sum_count = count1+count2;
//   auto new_elevation = f1->data().elevation_avg * (count1/sum_count) + f2->data().elevation_avg * (count2/sum_count);
//   f2->data().elevation_avg = f1->data().elevation_avg = new_elevation;
//   // and sum the counts
//   f2->data().segid_count = f1->data().segid_count = sum_count;
//   f1->data().elevation_min = f2->data().elevation_min = std::min(f1->data().elevation_min, f2->data().elevation_min);
//   f1->data().elevation_max = f2->data().elevation_min = std::max(f1->data().elevation_max, f2->data().elevation_max);
//   // merge the point lists
//   if (f1==f2){
//     std::cout << "merging the same face!?\n";
//     return;
//   }
//   f1->data().points.insert(f1->data().points.end(), f2->data().points.begin(), f2->data().points.end() );
//   f2->data().points.insert(f2->data().points.end(), f1->data().points.begin(), f1->data().points.end() );
// }

// convert each face to polygon and compute an average elevation
void process_arrangement(PNL_vector& points, Arrangement_2& arr, config c) {
  typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2> Point_location;
  
  Face_merge_observer obs (arr);

  Point_location   pl(arr);
  std::unordered_map<Face_handle, std::vector<PNL>> points_per_face;
  // std::unordered_set<Face_handle> faces_with_points;

  // collect for each face the points it contains
  for (auto& p : points){
    bool is_wall = boost::get<3>(p);
    bool is_on_plane = boost::get<2>(p) != 0;
    if ((!is_wall) && is_on_plane){
      auto obj = pl.locate( Point_2(boost::get<0>(p).x(), get<0>(p).y()) );
      if (auto f = boost::get<Face_const_handle>(&obj)) {
        auto fh = arr.non_const_handle(*f);
        fh->data().points.push_back(p);
        points_per_face[fh].push_back(p);
        // faces_with_points.insert(fh);
      }
    }
  }
  
  // check for the points in each face what segment occurs most often
  for(auto ppf : points_per_face) {
    auto fh = ppf.first;
    auto& fpoints = ppf.second;

    std::unordered_map<size_t,size_t> segment_histogram;
    for (auto& p : fpoints) {
      segment_histogram[boost::get<2>(p)]++;
    }
    size_t max_count = 0, max_segid;
    for (auto& pair : segment_histogram) {
      if (pair.second>max_count) {
        max_count = pair.second;
        max_segid = pair.first;
      }
    }
    float sum = 0, minz, maxz;
    minz=maxz=boost::get<0>(fpoints[0]).z();
    for (auto& p : fpoints) {
      if (boost::get<2>(p) == max_segid) {
        auto z = float(boost::get<0>(p).z());
        sum = sum + z;
        minz = std::min(minz,z);
        maxz = std::max(maxz,z);
      }
    }
    fh->data().segid = max_segid;
    fh->data().segid_coverage = max_count/fpoints.size();
    fh->data().elevation_avg = sum/max_count;
    fh->data().elevation_min = minz;
    fh->data().elevation_max = maxz;
    fh->data().segid_count = max_count;
  }
  // merge faces with the same segment id
  if (c.merge_segid) {
    std::vector<Arrangement_2::Halfedge_handle> edges;
    for (auto edge : arr.edge_handles()) {
      edges.push_back(edge);
    }
    for (auto& edge : edges) {
      auto f1 = edge->face();
      auto f2 = edge->twin()->face();
      if((f1->data().is_finite && f2->data().is_finite) && (f1->data().segid!=0 && f2->data().segid!=0)) {
        if(f1->data().segid == f2->data().segid){
          // elevation of new face is a weighted sum of elevation_avg of two old faces
          // merge_faces(f1,f2);
          arr.remove_edge(edge); // should add face merge call back in face observer class...
        }
      }
    }
  }
  // merge faces with segments that have an overlapping z-range, ie two adjacent segments without a step edge
  if (c.merge_zrange) {
    std::vector<Arrangement_2::Halfedge_handle> edges;
    for (auto edge : arr.edge_handles()) {
      edges.push_back(edge);
    }
    for (auto& edge : edges) {
      auto f1 = edge->face();
      auto f2 = edge->twin()->face();
      if((f1->data().is_finite && f2->data().is_finite) && (f1->data().segid!=0 && f2->data().segid!=0)) {
        auto z1_min = f1->data().elevation_min;
        auto z1_max = f1->data().elevation_max;
        auto z2_min = f2->data().elevation_min;
        auto z2_max = f2->data().elevation_max;
        if(((z2_max+c.zrange_threshold) > z1_min) && ((z2_min-c.zrange_threshold) < z1_max)) {
          // should add face merge call back in face observer class...
          // elevation of new face is a weighted sum of elevation_avg of two old faces
          // merge_faces(f1,f2);
          arr.remove_edge(edge);
        }
      }
    }
  }
  // merge faces with step height below threshold
  if (c.merge_step_height) {
    std::vector<Arrangement_2::Halfedge_handle> edges;
    for (auto edge : arr.edge_handles()) {
      edges.push_back(edge);
    }
    for (auto& edge : edges) {
      auto f1 = edge->face();
      auto f2 = edge->twin()->face();
      if((f1->data().is_finite && f2->data().is_finite) && (f1->data().segid!=0 && f2->data().segid!=0)) {
        if(std::abs(f1->data().elevation_avg - f2->data().elevation_avg) < c.step_height_threshold){
          // should add face merge call back in face observer class...
          // pick elevation of the segment with the highest count
          if (f1->data().segid_count > f2->data().segid_count) {
            f2->data().elevation_avg = f1->data().elevation_avg;
          } else {
            f1->data().elevation_avg = f2->data().elevation_avg;
          }
          arr.remove_edge(edge);
        }
      }
    }
  }
  // cleanup faces with segid==0 by merging them to a finite neighbour face
  if (c.merge_unsegmented) {
    std::vector<Face_handle> empty_faces;
    for (auto face: arr.face_handles()){
      if(face->data().is_finite && face->data().segid==0 ) {
        empty_faces.push_back(face);
      }
    }
    for (auto face : empty_faces) {
      auto he = face->outer_ccb();
      auto first = he;
      while(true){
        auto face_neighbour = he->twin()->face();
        // only care about interior faces
        if (face_neighbour->data().is_finite && face_neighbour->data().segid!=0){ 
          arr.remove_edge(he); //merge with first finite face we encounter!
          face->data() = face_neighbour->data(); // we want to preserve data of the face with segid !=0
          break;
        }
        he = he->next();
        if (he==first) break;
      }
    }

  }
  // cleanup dangling edges 
  if (c.merge_dangling_egdes) {
    std::vector<Arrangement_2::Halfedge_handle> edges;
    for (auto edge : arr.edge_handles()) {
      edges.push_back(edge);
    }
    for (auto& edge : edges) {
      if (edge->face() == edge->twin()->face()){
        arr.remove_edge(edge);
      }
    }
  }

  // compute final errors
  for (auto face: arr.face_handles()){
    if (face->data().is_finite) {
      auto cnt = face->data().points.size();
      if (cnt>0) {
        auto z = face->data().elevation_avg;
        double square_sum = 0;
        for (auto& p : face->data().points) {
          float d = z - boost::get<0>(p).z();
          square_sum += d*d;
          face->data().max_error = std::max(face->data().max_error, std::abs(d));
        }
        face->data().rms_error_to_avg = CGAL::sqrt(square_sum/cnt);
      }
      face->data().total_count = cnt;
    }
  }
  // // cleanup faces with segid==0 by merging them to valid neighbour with most shared edges
  // {
  //   // collect empty faces
  //   std::vector<Face_handle> empty_faces;
  //   for (auto face: arr.face_handles()){
  //     if(face->data().is_finite && face->data().segid==0 ) {
  //       empty_faces.push_back(face);
  //     }
  //   }
  //   // loop until all empty faces are merged
  //   while (empty_faces.size()!=0) {
  //     std::unordered_map<Face_handle, size_t> face_histogram;
  //     // find for each empty face the neighbor with most shared edges
  //     for (auto face : empty_faces) {  
  //       auto he = face->outer_ccb();
  //       auto first = he;

  //       // loop over neighbour faces
  //       while(true){
  //         auto face_neighbour = he->twin()->face();
  //         // only care about interior faces
  //         if (face_neighbour->data().is_finite){ 
  //           // make histogram of how many edges we share with each neighbour
  //           if (face_histogram.count(face_neighbour)) {
  //             face_histogram[face_neighbour]++;
  //           } else {
  //             face_histogram[face_neighbour] = 1;
  //           }
  //           // find most occuring neighbour face in face_histogram
  //           Face_handle max_face;
  //           size_t max_count=0;
  //           for(auto el : face_histogram) {
  //             auto new_max = std::max(max_count, el.second);
  //             if (new_max!=max_count) {
  //               max_count = new_max;
  //               max_face = el.first;
  //             }
  //           }
  //           //finish it by adding 10 more nested loops?! maybe later...
  //           std

  //         }
  //         he = he->next();
  //         if (he==first) break;
  //       }
  //     }
  //   }
        
  // }
}