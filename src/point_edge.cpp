#include <iostream>
#include <fstream>

#include <LASlib/lasreader.hpp>
#include <LASlib/laswriter.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>

// CGAL
#include <CGAL/number_utils.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/regularize_planes.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Cartesian_converter.h>
#include <utility> // defines std::pair

// #include "line_shape.cpp"
#include "index_map.cpp"
#include "region_growing.h"
// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Line_3 Line;
// Point with normal vector stored in a std::pair.
typedef CGAL::cpp11::tuple<Point, Vector, int, bool, double, int, bool, double, int> PNL;
typedef CGAL::Nth_of_tuple_property_map<0, PNL> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNL> Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNL> Label_map;
typedef CGAL::Nth_of_tuple_property_map<3, PNL> IsWall_map;
typedef CGAL::Nth_of_tuple_property_map<4, PNL> LineFit_map;
typedef CGAL::Nth_of_tuple_property_map<5, PNL> JumpCount_map;
typedef CGAL::Nth_of_tuple_property_map<6, PNL> IsStep_map;
typedef CGAL::Nth_of_tuple_property_map<7, PNL> JumpEle_map;
typedef CGAL::Nth_of_tuple_property_map<8, PNL> Id_map;
typedef std::vector<PNL>                        PNL_vector;
// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif
// In Shape_detection_traits the basic types, i.e., Point and Vector types
// as well as iterator type and property maps, are defined.
typedef CGAL::Shape_detection_3::Shape_detection_traits
  <Kernel, PNL_vector, Point_map, Normal_map>      Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>    Efficient_ransac;
typedef CGAL::Shape_detection_3::Region_growing<Traits>      Region_growing;
typedef CGAL::Shape_detection_3::Plane<Traits>               SCPlane;

// search tree
// typedef boost::tuple<Point_3,int>                           Point_and_int;
// typedef CGAL::Random_points_in_cube_3<Point_3>              Random_points_iterator;
typedef CGAL::Search_traits_3<Kernel>                       Traits_base;
typedef CGAL::Search_traits_adapter<PNL,
  CGAL::Nth_of_tuple_property_map<0, PNL>,
  Traits_base>                                              TreeTraits;
// typedef CGAL::Search_traits_3<SCK> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;

// least squares stuff
typedef CGAL::Simple_cartesian<double> SCK;
typedef SCK::Point_3 Point_SCK;
typedef SCK::Line_3 Line_SCK;


// 2D arrangement
// typedef CGAL::Exact_rational Number_type;
// typedef double Number_type;
// typedef CGAL::Cartesian<Number_type>           AK;
typedef CGAL::Exact_predicates_exact_constructions_kernel   AK;
typedef CGAL::Arr_linear_traits_2<AK>                 Traits_2;
typedef Traits_2::Point_2                             Point_2;
typedef CGAL::Polygon_2<AK>                           Polygon_2;
// typedef Kernel::Point_2 PolygonPoint;
typedef Traits_2::Segment_2                           Segment_2;
typedef Traits_2::Ray_2                               Ray_2;
typedef Traits_2::Line_2                              Line_2;
typedef Traits_2::X_monotone_curve_2                  X_monotone_curve_2;
typedef CGAL::Arrangement_2<Traits_2>                 Arrangement_2;
typedef Arrangement_2::Vertex_handle                  Vertex_handle;
typedef Arrangement_2::Halfedge_handle                Halfedge_handle;
typedef CGAL::Arr_accessor<Arrangement_2>             Arr_accessor;
typedef Arr_accessor::Dcel_vertex                     DVertex;
// typedef CGAL::Cartesian_converter<IK,EK>                         IK_to_EK;
// typedef CGAL::Cartesian_converter<Number_type,SCK>          ToInexact;

// inline Number_type arr_s(double v){return static_cast<Number_type>(100*v);}

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> point_type;
typedef bg::model::point<double, 3, bg::cs::cartesian> point_type_3d;
typedef bg::model::segment<point_type> segment;

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

void detect_lines(std::vector<std::pair<Point,Point>> & edge_segments, std::vector<linedect::Point> &edge_points) {
  // line fitting in set of candidate edgepoints
  auto LD = linedect::LineDetector(edge_points);
  LD.dist_thres = 0.5 * 0.5;
  LD.min_segment_count = 20;
  LD.N = 17;
  LD.detect();

  std::cout << LD.segment_shapes.size() << " lines detected." << std::endl;
  std::vector<int> edges_index_map(edge_points.size(),-1); // property map that stores the line index for each point
  
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

int main(int argc, char *argv[])
{
  // read las pointcloud
  // read wkt polygons
  // perform boost point within polygon
  // [per polygon: cgal ransac]
  // per polygon: project elevation/plane-labels to raster
  // per polygon: opencv edge-detection and line-extraction
  // write lines as wkt
  std::cout << std::fixed << std::setprecision(2);
  
  bg::model::polygon<point_type> footprint;
  bg::read_wkt("Polygon ((91716.13000000000465661 438312.07000000000698492, 91716.77199999999720603 438312.76799999998183921, 91717.66099999999278225 438313.73499999998603016, 91729.11999999999534339 438326.19000000000232831, 91729.2559999999939464 438326.0659999999916181, 91738.30999999999767169 438317.84000000002561137, 91739.07000000000698492 438318.66999999998370185, 91743.60000000000582077 438314.53999999997904524, 91749.55000000000291038 438321.03000000002793968, 91736.07000000000698492 438333.34000000002561137, 91731.43499999999767169 438337.55499999999301508, 91728.17699999999604188 438340.51799999998183921, 91727.96300000000337604 438340.71299999998882413, 91726.35000000000582077 438342.17999999999301508, 91730.4419999999954598 438346.66800000000512227, 91731.20600000000558794 438347.50500000000465661, 91743.52000000000407454 438361.01000000000931323, 91739.02800000000570435 438365.08699999999953434, 91736.75800000000162981 438367.14699999999720603, 91736.58999999999650754 438367.29999999998835847, 91722.71600000000034925 438352.11300000001210719, 91721.10899999999674037 438353.64000000001396984, 91711.05000000000291038 438342.59999999997671694, 91712.69500000000698492 438341.14600000000791624, 91709.50999999999476131 438337.65999999997438863, 91716.41400000000430737 438331.41300000000046566, 91717.96199999999953434 438330.01099999999860302, 91719.08000000000174623 438329, 91710.89999999999417923 438320.01000000000931323, 91707.22999999999592546 438323.28999999997904524, 91706.2559999999939464 438322.22399999998742715, 91704.65200000000186265 438320.46799999999348074, 91703.85000000000582077 438319.59000000002561137, 91703.91199999999662396 438319.53299999999580905, 91706.33999999999650754 438317.30999999999767169, 91693.41999999999825377 438303.19000000000232831, 91691 438305.40000000002328306, 91687.58999999999650754 438301.70000000001164153, 91697.96899999999732245 438292.19599999999627471, 91702.67799999999988358 438297.37099999998463318, 91707.28299999999580905 438302.4469999999855645, 91711.53299999999580905 438307.13299999997252598, 91711.66000000000349246 438307.28999999997904524, 91713.7470000000030268 438309.60700000001816079, 91716.13000000000465661 438312.07000000000698492))", footprint);
  // bg::read_wkt("Polygon ((85144.81600000000617001 446857.39500000001862645, 85143.8779999999969732 446858.19000000000232831, 85143.3110000000015134 446857.79300000000512227, 85135.34100000000034925 446848.21399999997811392, 85138.76600000000325963 446845.15700000000651926, 85112.8139999999984866 446814.73200000001816079, 85111.96000000000640284 446815.47100000001955777, 85111.4379999999946449 446815.12699999997857958, 85103.44000000000232831 446805.43800000002374873, 85118.54099999999743886 446792.7440000000060536, 85126.94800000000395812 446802.63199999998323619, 85122.94299999999930151 446806.20699999999487773, 85136.94100000000617001 446822.63199999998323619, 85140.11500000000523869 446819.92999999999301508, 85148.36400000000139698 446829.61300000001210719, 85148.71000000000640284 446829.32699999999022111, 85152.38000000000465661 446833.63699999998789281, 85167.39599999999336433 446820.8159999999916181, 85157.72100000000500586 446809.56199999997625127, 85153.55599999999685679 446813.143999999971129, 85151.22199999999429565 446810.40600000001722947, 85155.86000000000058208 446806.45100000000093132, 85150.23200000000360887 446800.13400000002002344, 85149.8690000000060536 446799.39799999998649582, 85150.35000000000582077 446798.73800000001210719, 85160.5620000000053551 446790.30400000000372529, 85166.88599999999860302 446797.70500000001629815, 85167.61800000000221189 446798.26799999998183921, 85168.63700000000244472 446797.85300000000279397, 85176.97400000000197906 446807.66600000002654269, 85176.66000000000349246 446808.39199999999254942, 85178.9419999999954598 446811.02399999997578561, 85196.32399999999324791 446796.21199999999953434, 85177.78500000000349246 446774.40700000000651926, 85172.9360000000015134 446778.37800000002607703, 85169.47900000000663567 446774.356000000028871, 85170.23299999999289867 446773.54599999997299165, 85164.38999999999941792 446766.72999999998137355, 85157.3139999999984866 446772.6120000000228174, 85149.45200000000477303 446763.41499999997904524, 85174.02899999999499414 446742.13699999998789281, 85174.71700000000419095 446741.85100000002421439, 85175.44500000000698492 446742.71500000002561137, 85176.72299999999813735 446740.04100000002654269, 85178.23799999999755528 446741.63599999999860302, 85176.86599999999452848 446744.44799999997485429, 85182.63499999999476131 446751.18499999999767169, 85177.61800000000221189 446755.42599999997764826, 85205.6190000000060536 446788.28299999999580905, 85209.95799999999871943 446784.3090000000083819, 85210.92600000000675209 446784.25500000000465661, 85214.03800000000046566 446781.58500000002095476, 85222.10099999999511056 446791.07099999999627471, 85218.97500000000582077 446793.72700000001350418, 85218.7480000000068685 446794.65600000001722947, 85208.45200000000477303 446803.40299999999115244, 85207.81600000000617001 446803.72100000001955777, 85207.45900000000256114 446803.29999999998835847, 85189.20699999999487773 446818.80699999997159466, 85188.85899999999674037 446819.77199999999720603, 85183.4970000000030268 446824.3279999999795109, 85182.48900000000139698 446824.51500000001396984, 85178.42200000000593718 446827.96999999997206032, 85193.33999999999650754 446845.45000000001164153, 85194.3129999999946449 446844.6190000000060536, 85194.62900000000081491 446844.98900000000139698, 85196.69500000000698492 446843.22600000002421439, 85199.80299999999988358 446845.27000000001862645, 85196.94800000000395812 446847.70600000000558794, 85201.92500000000291038 446853.79899999999906868, 85202.61699999999837019 446854.34899999998742715, 85205.48699999999371357 446851.89899999997578561, 85207.10199999999895226 446855.19500000000698492, 85204.22299999999813735 446858.20100000000093132, 85219.10799999999289867 446875.64299999998183921, 85223.21099999999569263 446872.11099999997531995, 85222.88000000000465661 446871.72600000002421439, 85228.91400000000430737 446866.61099999997531995, 85229.69700000000011642 446866.15999999997438863, 85229.95699999999487773 446866.46199999999953434, 85241.86400000000139698 446856.21399999997811392, 85241.4940000000060536 446855.78299999999580905, 85241.88599999999860302 446855.21999999997206032, 85252.15799999999580905 446846.53700000001117587, 85260.76799999999639113 446856.45000000001164153, 85260.42299999999522697 446857.16899999999441206, 85246.9389999999984866 446868.40600000001722947, 85249.19899999999324791 446871.10499999998137355, 85249.97100000000500586 446870.92099999997299165, 85258.33299999999871943 446880.7440000000060536, 85257.77199999999720603 446881.68900000001303852, 85258.21899999999732245 446882.51699999999254942, 85264.23099999999976717 446889.1720000000204891, 85264.59299999999348074 446889.893999999971129, 85264.1269999999931315 446890.52600000001257285, 85254.05299999999988358 446898.94900000002235174, 85247.89999999999417923 446891.96500000002561137, 85243.26300000000628643 446895.93499999999767169, 85240.9389999999984866 446893.15799999999580905, 85244.6919999999954598 446889.92999999999301508, 85237.4980000000068685 446881.67099999997299165, 85237.77099999999336433 446880.88199999998323619, 85235.50699999999778811 446878.30499999999301508, 85205.14500000000407454 446904.16800000000512227, 85226.58100000000558794 446929.44500000000698492, 85252.31799999999930151 446907.32299999997485429, 85252.93499999999767169 446907.00799999997252598, 85253.5029999999969732 446907.40600000001722947, 85261.49099999999452848 446917.19400000001769513, 85237.65899999999965075 446937.375, 85241.62099999999918509 446941.99800000002142042, 85243.97000000000116415 446940.13000000000465661, 85247.4379999999946449 446944.17599999997764826, 85243.4419999999954598 446947.63699999998789281, 85257.58800000000337604 446964.18499999999767169, 85259.01700000000710133 446962.89100000000325963, 85259.70299999999406282 446963.25199999997857958, 85267.32000000000698492 446972.30400000000372529, 85267.76900000000023283 446972.90600000001722947, 85267.34699999999429565 446973.48900000000139698, 85258.27000000000407454 446981.01500000001396984, 85258.49099999999452848 446981.27399999997578561, 85257.89800000000104774 446981.78000000002793968, 85257.67699999999604188 446981.52199999999720603, 85255.47299999999813735 446983.46899999998277053, 85254.8169999999954598 446983.11400000000139698, 85247.08599999999569263 446974.09399999998277053, 85246.77400000000488944 446973.46000000002095476, 85247.55400000000372529 446972.75699999998323619, 85195.77599999999802094 446912.11400000000139698, 85192.19400000000314321 446914.98999999999068677, 85184.04700000000593718 446905.58799999998882413, 85183.75500000000465661 446904.97399999998742715, 85184.15399999999499414 446904.40700000000651926, 85193.20900000000256114 446896.73300000000745058, 85194.58400000000256114 446896.54499999998370185, 85209.08500000000640284 446884.22200000000884756, 85194.47400000000197906 446866.93699999997625127, 85191.97000000000116415 446868.70000000001164153, 85191.17299999999522697 446868.55099999997764826, 85190.75900000000547152 446868.06400000001303852, 85190.91099999999278225 446867.14699999999720603, 85182.48699999999371357 446857.24499999999534339, 85181.55499999999301508 446857.28899999998975545, 85181.13300000000162981 446856.79499999998370185, 85181.10799999999289867 446855.96299999998882413, 85183.42999999999301508 446854.0470000000204891, 85183.25800000000162981 446853.75300000002607703, 85168.71000000000640284 446836.71299999998882413, 85153.97800000000279397 446848.8809999999939464, 85153.5620000000053551 446850.20699999999487773, 85144.81600000000617001 446857.39500000001862645))", footprint);
  bg::model::box<point_type> bbox;
  bg::envelope(footprint, bbox);

  PNL_vector points;
  pc_in_footprint("/Users/ravi/git/heightjump-detect/build/ahn3.las", footprint, points);
  
  compute_metrics(points);

  //add wall points to candidate edge points? => then we would need to set z=0 on all points, ie 2D line detection...

  // select candidate edge points
  // TODO: also try to add any points on the walls, eg from detected wall segments and by searching for them from the detected edgepoints (by xy-projection) for the case there were too few points to construct a vertical plane/wall
  std::vector<linedect::Point> edge_points;
  classify_edgepoints(edge_points, points);

  std::vector<std::pair<Point,Point>> edge_segments;
  detect_lines(edge_segments, edge_points);

  
  
  // regularisation of lines
  std::vector<std::pair<Plane,bool>> wall_planes;
  for (auto s : edge_segments){
    wall_planes.push_back(std::make_pair(Plane(s.first, s.second, s.first+Vector(0,0,1)),0));
  }
  // CGAL::regularize_planes(edge_points,
                          //  Point_map(),
                          //  wall_planes,
                          //  CGAL::First_of_pair_property_map<std::pair<Plane,bool>>(),
                          //  Index_map(edges_index_map),
                          //  true, // Regularize parallelism
                          //  true, // Regularize orthogonality
                          //  true, // Do not regularize coplanarity
                          //  false, // Regularize Z-symmetry (default)
                          //  40, // 10 degrees of tolerance for parallelism/orthogonality
                          //  0.8); //tolerance_coplanarity
  

  std::ofstream f_obj("edges.obj");
  f_obj << std::fixed << std::setprecision(2);
  for(auto s: edge_segments){
    f_obj << "v " << s.first.x() << " " << s.first.y() << " " << s.first.z() << std::endl;
    f_obj << "v " << s.second.x() << " " << s.second.y() << " " << s.second.z() << std::endl;
  }
  for(size_t si=0; si < edge_segments.size(); si++)
    f_obj << "l " << si*2+1 << " " << si*2+2 << std::endl;
  f_obj.close();

  std::ofstream f("out.ply");
  // CGAL::set_binary_mode(f); // The PLY file will be written in the binary format
  f << std::fixed << std::setprecision(2);

  CGAL::write_ply_points_with_properties(f, points,
     CGAL::make_ply_point_writer (Point_map()),
     CGAL::make_ply_normal_writer (Normal_map()),
      // std::make_pair (Label_map(), CGAL::PLY_property<int>("label")),
      //  std::make_pair (LineFit_map(), CGAL::PLY_property<double>("line_fit")),
      //  std::make_pair (JumpCount_map(), CGAL::PLY_property<int>("jump_count")),
      std::make_pair (IsStep_map(), CGAL::PLY_property<int>("is_step"))
      //  std::make_pair (JumpEle_map(), CGAL::PLY_property<double>("jump_ele"))
      //  std::make_pair (Id_map(), CGAL::PLY_property<int>("id"))
  );
  f.close();

  

  // build 3D spatial index
  // do foreach point a knn search 
  //  fit line to k-neighbourhood and compute distance to that line
  // fit lines to set of candidate edge points
  // insert footprint edges as lines and the fitted lines in a 2D arrangement bounded by the footprint
  // merge adjacent faces in the arrangement if they are co-planar/have the same segment-id
  // remaining edges are the step edges and original footprint edges

  Arrangement_2 arr;
  // const double s = 100;

  // add footprint outline
  std::vector<Point_2> footprint_pts;
  for (auto p : footprint.outer()) {
    footprint_pts.push_back(Point_2(bg::get<0>(p), bg::get<1>(p)));
  }
  Polygon_2 cgal_footprint(footprint_pts.begin(), footprint_pts.end());
  insert(arr, cgal_footprint.edges_begin(), cgal_footprint.edges_end());

  // prepare step-edge lines
  std::vector<X_monotone_curve_2> lines;
  Plane horizontal_plane(Point(0,0,-100), Vector(0,0,1));
  for (auto wp: wall_planes){
    auto result = CGAL::intersection(horizontal_plane, wp.first);
    if (const Line* line = boost::get<Line>(&*result)){
      auto p0 = line->point(0);
      auto p1 = line->point(1);
      const Point_2 a(p0.x(),p0.y());
      const Point_2 b(p1.x(),p1.y());
      lines.push_back(Line_2(a, b));
    }
  }
  insert(arr, lines.begin(), lines.end());

  // for (auto s: edge_segments)
  //   lines.push_back(Line_2(Point_2(s.first.x(), s.first.y()), Point_2(s.second.x(), s.second.y())));
  // insert(arr, lines.begin(), lines.end());

  // write to wkt
  std::ofstream f_arr("arr.wkt");

  f_arr << std::fixed << std::setprecision(2);
  for (auto he: arr.edge_handles()){
    const DVertex* sv = &(*he->source());
    const DVertex* tv = &(*he->target());
    // std::cout << he->curve() << std::endl;
    if (!(sv->has_null_point() || tv->has_null_point()))
      f_arr << "LINESTRING(" << he->source()->point().x() << " " << he->source()->point().y() << "," << he->target()->point().x() << " " << he->target()->point().y() << ")" << std::endl;
  }
  f_arr.close();



  return 0;
}
