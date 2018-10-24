#include "point_edge.h"
#include <CGAL/Arr_walk_along_line_point_location.h>
#include <unordered_map>
#include <boost/tuple/tuple.hpp>

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
          CGAL::cpp11::get<0>(pv) = Point(lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z());
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

  if(points.size()==0) return;

  // Estimates normals
  // Note: pca_estimate_normals() requiresa range of points
  // as well as property maps to access each point's position and normal.
  const int nb_neighbors = 5; // K-nearest neighbors = 3 rings
  CGAL::pca_estimate_normals<Concurrency_tag>
    (points, nb_neighbors,
    CGAL::parameters::point_map(Point_map()).
    normal_map(Normal_map()));
  // Orients normals.
  // Note: mst_orient_normals() requires a range of points
  // as well as property maps to access each point's position and normal.
  PNL_vector::iterator unoriented_points_begin =
    CGAL::mst_orient_normals(points, nb_neighbors,
                              CGAL::parameters::point_map(Point_map()).
                              normal_map(Normal_map()));
  // Optional: delete points with an unoriented normal
  // if you plan to call a reconstruction algorithm that expects oriented normals.
  // points.erase(unoriented_points_begin, points.end());

  size_t i=0;
  for (auto &p : points) {
    boost::get<8>(p) = i++;
  }
  // Instantiates shape detection engine.
  Region_growing shape_detection;

  // Sets parameters for shape detection.
  Region_growing::Parameters parameters;
  // Sets probability to miss the largest primitive at each iteration.
  // parameters.probability = 0.05;
  // Detect shapes with at least 500 points.
  parameters.min_points = c.metrics_plane_min_points;
  // Sets maximum Euclidean distance between a point and a shape.
  parameters.epsilon = c.metrics_plane_epsilon;
  // Sets maximum Euclidean distance between points to be clustered.
  // parameters.cluster_epsilon = 0.01;
  // Sets maximum normal deviation.
  // 0.9 < dot(surface_normal, point_normal); 
  parameters.normal_threshold = c.metrics_plane_normal_threshold;

  // Provides the input data.
  shape_detection.set_input(points);
  // Registers planar shapes via template method.
  shape_detection.add_shape_factory<SCPlane>();
  // Detects registered shapes with parameters.
  shape_detection.detect(parameters);
  // Prints number of detected shapes.
  std::cout << shape_detection.shapes().end() - shape_detection.shapes().begin() << " shapes detected." << std::endl;

  i=1;
  for(auto shape: shape_detection.shapes()){
    // this dot product is close to 0 for vertical planes
    SCPlane* plane = dynamic_cast<SCPlane*>(shape.get());
    std::cout << shape->info() << std::endl;
    Vector n = plane->plane_normal();
    std::cout << n*Vector(0,0,1) << std::endl;
    bool is_wall = CGAL::abs(n*Vector(0,0,1)) < c.metrics_is_wall_threshold;
    
    // store cluster id's and is_wall as point attributes
    int label = i++;
    for(auto ind : shape->indices_of_assigned_points()){
      boost::get<2>(points[ind])=label;
      boost::get<3>(points[ind])=is_wall;
      // std::cout <<  << std::endl;
    }

    if(!is_wall){
      const unsigned int N = c.metrics_k_linefit;
      PNL_vector cluster_points;
      for(auto ind : shape->indices_of_assigned_points()){
        cluster_points.push_back(points[ind]);
      }
      Tree tree(cluster_points.begin(), cluster_points.end());
      // Point_d query(0,0);
      // Select candidate edge point based on line fit through k-neighborhood
      for(auto ind : shape->indices_of_assigned_points()){
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
  auto LD = linedect::LineDetector(edge_points);
  LD.dist_thres = c.linedetect_dist_threshold * c.linedetect_dist_threshold;
  LD.min_segment_count = c.linedetect_min_segment_count;
  LD.N = c.linedetect_k;
  LD.detect();

  std::cout << LD.segment_shapes.size() << " lines detected." << std::endl;
  //  std::vector<int> &edges_index_map,
  
  int line_id=0;
  for(auto seg: LD.segment_shapes){
    auto l = seg.second;
    auto l_idx = LD.get_point_indices(seg.first);
    std::cout << l.direction() << ", #Pts: " << l_idx.size() <<std::endl;
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

void build_arrangement(bg::model::polygon<point_type> &footprint, std::vector<std::pair<Point,Point>> & edge_segments, Arrangement_2 &arr){
  Face_index_observer obs (arr);
  // const double s = 100;

  // insert footprint segments
  std::vector<Point_2> footprint_pts;
  for (auto p : footprint.outer()) {
    footprint_pts.push_back(Point_2(bg::get<0>(p), bg::get<1>(p)));
  }
  footprint_pts.pop_back(); // get rid of repeated vertex in boost polygon
  Polygon_2 cgal_footprint(footprint_pts.begin(), footprint_pts.end());
  std::cout << "fp size=" <<footprint_pts.size() << "; " << footprint_pts[0].x() <<","<<footprint_pts[0].y()<<"\n";
  insert_non_intersecting_curves(arr, cgal_footprint.edges_begin(), cgal_footprint.edges_end());

  // std::cout << arr.number_of_faces() <<std::endl;
  // for (auto face: arr.face_handles()){
  //   if(face->is_unbounded())
  //     std::cout << "unbounded face, id: " << face->data() << std::endl;
  //   else
  //     std::cout << "Bounded face, id: " << face->data() << std::endl;
  // }

  // line regularisation


  // insert step-edge lines
  // std::vector<std::pair<Plane,bool>> wall_planes;
  std::vector<X_monotone_curve_2> lines;
  for (auto s : edge_segments){
    // wall_planes.push_back(std::make_pair(Plane(s.first, s.second, s.first+Vector(0,0,1)),0));
    const Point_2 a(s.first.x(),s.first.y());
    const Point_2 b(s.second.x(),s.second.y());
    lines.push_back(Line_2(a, b));
  }
  insert(arr, lines.begin(), lines.end());
}

void arrangementface_to_polygon(Face_handle face, vec2f& polygons){
  if(face->data().is_finite){ // ie it is a face on the interior of the footprint
    auto he = face->outer_ccb();
    auto first = he;

    while(true){
      polygons.push_back({
        float(CGAL::to_double(he->source()->point().x())),
        float(CGAL::to_double(he->source()->point().y()))
      });

      he = he->next();
      if (he==first) break;
    }
  }
}

// convert each face to polygon and compute an average elevation
void process_arrangement(PNL_vector& points, Arrangement_2& arr) {
  typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2> Point_location;
  

  Point_location   pl(arr);
  std::unordered_map<Face_handle, std::vector<PNL>> points_per_face;

  // collect for each face the points it contains
  for (auto& p : points){
    bool is_wall = boost::get<3>(p);
    bool is_on_plane = boost::get<2>(p) != 0;
    if ((!is_wall) && is_on_plane){
      auto obj = pl.locate( Point_2(boost::get<0>(p).x(), get<0>(p).y()) );
      if (auto f = boost::get<Face_const_handle>(&obj)) {
        points_per_face[arr.non_const_handle(*f)].push_back(p);
      }
    }
  }
  
  // check for the points in each face what segment occurs most often
  for(auto face_points : points_per_face) {
    auto fh = face_points.first;
    auto fpoints = face_points.second;

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
    float sum = 0;
    for (auto& p : fpoints) {
      if (boost::get<2>(p) == max_segid)
        sum = sum + boost::get<0>(p).z();
    }
    fh->data().segid = max_segid;
    fh->data().segid_coverage = max_count/fpoints.size();
    fh->data().elevation_avg = sum/max_count;
  }

  // merge faces with the same segment id
  for (auto& edge : arr.edge_handles()) {
    auto&& f1 = edge->face();
    auto&& f2 = edge->twin()->face();
    if(f1->data().is_finite && f2->data().is_finite) {
      if(f1->data().segid == f2->data().segid){
        arr.remove_edge(edge); // should add face merge call back in face observer class...
      }
    }
  }
}