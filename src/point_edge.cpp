#include "point_edge.h"

void pc_in_footprint(std::string las_filename, bg::model::polygon<point_type> &footprint, PNL_vector &points) {
  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(las_filename.c_str());
  // lasreadopener.set_file_name("/Users/ravi/SyncCode/bkt.las");
  LASreader* lasreader = lasreadopener.open();

  // LASwriteOpener laswriteopener;
  // laswriteopener.set_file_name("compressed.laz");
  // LASwriter* laswriter = laswriteopener.open(&lasreader->header);

  while (lasreader->read_point()) {
    if (lasreader->point.get_classification() == 6){
      point_type p;
      p.set<0>(lasreader->point.get_x());
      p.set<1>(lasreader->point.get_y());
      // p.set<2>(lasreader->point.get_z());
      if (bg::within(p,footprint)){
        PNL pv;
        CGAL::cpp11::get<0>(pv) = Point(lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z());
        points.push_back(pv);
        // laswriter->write_point(&lasreader->point);
      }
    }
  }
  lasreader->close();
  delete lasreader;

  // Estimates normals direction.
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
  points.erase(unoriented_points_begin, points.end());

  int i=0;
  for (auto &p : points) {
    p.get<8>() = i++;
  }

}

void compute_metrics(PNL_vector &points) {

  // Instantiates shape detection engine.
  Region_growing shape_detection;

  // Sets parameters for shape detection.
  Region_growing::Parameters parameters;
  // Sets probability to miss the largest primitive at each iteration.
  // parameters.probability = 0.05;
  // Detect shapes with at least 500 points.
  parameters.min_points = 40;
  // Sets maximum Euclidean distance between a point and a shape.
  parameters.epsilon = 0.2;
  // Sets maximum Euclidean distance between points to be clustered.
  // parameters.cluster_epsilon = 0.01;
  // Sets maximum normal deviation.
  // 0.9 < dot(surface_normal, point_normal); 
  parameters.normal_threshold = 0.95; 

  // Provides the input data.
  shape_detection.set_input(points);
  // Registers planar shapes via template method.
  shape_detection.add_shape_factory<SCPlane>();
  // Detects registered shapes with parameters.
  shape_detection.detect(parameters);
  // Prints number of detected shapes.
  std::cout << shape_detection.shapes().end() - shape_detection.shapes().begin() << " shapes detected." << std::endl;

  int i=1;
  for(auto shape: shape_detection.shapes()){
    // this dot product is close to 0 for vertical planes
    SCPlane* plane = dynamic_cast<SCPlane*>(shape.get());
    std::cout << shape->info() << std::endl;
    Vector n = plane->plane_normal();
    std::cout << n*Vector(0,0,1) << std::endl;
    bool is_wall = CGAL::abs(n*Vector(0,0,1)) < 0.1;
    
    // store cluster id's and is_wall as point attributes
    int label = i++;
    for(auto ind : shape->indices_of_assigned_points()){
      (points[ind]).get<2>()=label;
      (points[ind]).get<3>()=is_wall;
      // std::cout <<  << std::endl;
    }

    if(!is_wall){
      const unsigned int N = 30;
      PNL_vector cluster_points;
      for(auto ind : shape->indices_of_assigned_points()){
        cluster_points.push_back(points[ind]);
      }
      Tree tree(cluster_points.begin(), cluster_points.end());
      // Point_d query(0,0);
      // Select candidate edge point based on line fit through k-neighborhood
      for(auto ind : shape->indices_of_assigned_points()){
        Point q(points[ind].get<0>());
        Neighbor_search search(tree, q, N);
        std::vector<Point_SCK> neighbor_points;
        for (auto neighbor: search)
          neighbor_points.push_back(Point_SCK(neighbor.first.get<0>().x(), neighbor.first.get<0>().y(), neighbor.first.get<0>().z()));
        Line_SCK line;
        linear_least_squares_fitting_3(neighbor_points.begin(),neighbor_points.end(),line,CGAL::Dimension_tag<0>());
        auto line_dist = CGAL::squared_distance(line, Point_SCK(q.x(), q.y(), q.z()));
        (points[ind]).get<4>() = line_dist;
      }
      
    }
  }

  // detect height jumps
  PNL_vector roof_points;
  for (auto p : points){
    bool is_wall = p.get<3>();
    bool is_on_plane = p.get<2>() != 0;
    if ((!is_wall) && is_on_plane){
      auto p_ = p;
      p_.get<0>()= Point(p.get<0>().x(), p.get<0>().y(), 0);
      roof_points.push_back(p_);
    }
  }
  Tree tree(roof_points.begin(), roof_points.end());
  const unsigned int N = 10;
  for (auto q : points){
    bool is_wall = q.get<3>();
    bool is_on_plane = q.get<2>() != 0;
    int jump_cnt = 0;
    double jump_ele=0;
    if ((!is_wall) && is_on_plane){
      auto q_segment = q.get<2>();
      auto q_ = Point(q.get<0>().x(), q.get<0>().y(), 0);
      Neighbor_search search(tree, q_, N);
      for (auto neighbor: search){
        auto np = neighbor.first;
        auto np_segment = np.get<2>();
        if (q_segment!=np_segment)
          jump_cnt++;
        auto ele_diff = CGAL::abs(points[np.get<8>()].get<0>().z()-points[q.get<8>()].get<0>().z());
        jump_ele = std::max(ele_diff, jump_ele);
      }
      points[q.get<8>()].get<5>() = jump_cnt;
      points[q.get<8>()].get<7>() = jump_ele;
    }
    
  }
}

void classify_edgepoints(std::vector<linedect::Point> &edge_points, PNL_vector &points) {
  for (auto &p : points){
    double line_dist = p.get<4>();
    double jump_count = p.get<5>();
    double jump_ele = p.get<7>();
    // if (line_dist > 0.01){// ie we assume the point to be on an edge of this cluster
    bool is_step = (jump_count >= 1 && jump_count <= 5) && (line_dist > 0.01) && (jump_ele > 1);
    // bool is_step = (jump_count >= 1 && jump_count <= 5) && line_dist > 0.01;
    // bool is_step = (jump_count >= 1) && line_dist > 0.01;
    // bool is_step = line_dist > 0.01;
    p.get<6>() = is_step;
    if (is_step){// ie we assume the point to be on an edge of this cluster
      // edge_points.push_back(p); // add this point to a new set of candidate points for region-growing line-fitting
      // project to z=0 - seems to yield more reliable line detection
      edge_points.push_back(
        // linedect::Point(p.get<0>().x(),p.get<0>().y(),0)
        linedect::Point(p.get<0>().x(),p.get<0>().y(),p.get<0>().z())
      ); // add this point to a new set of candidate points for region-growing line-fitting
    }
  }
}

void detect_lines(std::vector<std::pair<Point,Point>> & edge_segments, std::vector<linedect::Point> &edge_points, std::vector<int> &edges_index_map) {
  // line fitting in set of candidate edgepoints
  auto LD = linedect::LineDetector(edge_points);
  LD.dist_thres = 0.5 * 0.5;
  LD.min_segment_count = 20;
  LD.N = 17;
  LD.detect();

  std::cout << LD.segment_shapes.size() << " lines detected." << std::endl;
  
  int line_id=0;
  for(auto seg: LD.segment_shapes){
    auto l = seg.second;
    auto l_idx = LD.get_point_indices(seg.first);
    std::cout << l.direction() << ", #Pts: " << l_idx.size() <<std::endl;
    //find the two extreme points on this line
    double minpl=1, maxpl=0;
    size_t j=0;
    auto l_normal = l.to_vector()/CGAL::sqrt(l.to_vector().squared_length());
    for(auto lind : l_idx){
      edges_index_map[lind] = line_id;
      // project this_p on l
      linedect::Point this_p(edge_points[lind]);
      const Vector a(l.point(0), this_p);
      double pl = a*l_normal;

      if (j++==0){
        minpl = maxpl = pl;
      } else {
        if (pl < minpl) minpl=pl;
        else maxpl=pl;
      }
    }
    line_id++;
    Point p0 = (l.point(0) + minpl*l_normal);
    Point p1 = (l.point(0) + maxpl*l_normal);
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
  Polygon_2 cgal_footprint(footprint_pts.begin(), footprint_pts.end());
  insert_non_intersecting_curves(arr, cgal_footprint.edges_begin(), cgal_footprint.edges_end());

  // std::cout << arr.number_of_faces() <<std::endl;
  // for (auto face: arr.face_handles()){
  //   if(face->is_unbounded())
  //     std::cout << "unbounded face, id: " << face->data() << std::endl;
  //   else
  //     std::cout << "Bounded face, id: " << face->data() << std::endl;
  // }

  // insert step-edge lines
  std::vector<std::pair<Plane,bool>> wall_planes;
  std::vector<X_monotone_curve_2> lines;
  for (auto s : edge_segments){
    wall_planes.push_back(std::make_pair(Plane(s.first, s.second, s.first+Vector(0,0,1)),0));
    const Point_2 a(s.first.x(),s.first.y());
    const Point_2 b(s.second.x(),s.second.y());
    lines.push_back(Line_2(a, b));
  }
  insert(arr, lines.begin(), lines.end());
}