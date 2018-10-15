#include "imgui.h"
#include "geoflow.hpp"
#include "ogrsf_frmts.h"

//CDT
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Triangulation_vertex_base_with_id_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

//AABB tree
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

#include <lasreader.hpp>

#include "tinsimp.hpp"

using namespace geoflow;

class LASLoaderNode:public Node {

  public:
  char filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
  int thin_nth = 20;
  bool use_thin = true;

  LASLoaderNode(NodeManager& manager):Node(manager, "LASLoader") {
    add_output("geometry", TT_geometry);
    add_output("points", TT_vec3f);
    add_output("classification", TT_vec1i);
    add_output("intensity", TT_vec1f);
    GDALAllRegister();
  }

  void gui(){
    ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 1, 100);
  }

  void process(){
    gfGeometry3D geometry;
    geometry.type = geoflow::points;
    geometry.format = geoflow::simple;
    vec1i classification;
    vec1f intensity;

    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(filepath);
    LASreader* lasreader = lasreadopener.open();
    if (!lasreader)
      return;

    geometry.bounding_box.set(
      {float(lasreader->get_min_x()), float(lasreader->get_min_y()), float(lasreader->get_min_z())},
      {float(lasreader->get_max_x()), float(lasreader->get_max_y()), float(lasreader->get_max_z())}
    );

    size_t i=0;
    while (lasreader->read_point()) {

      if (i++ % thin_nth == 0) {
        classification.push_back(lasreader->point.get_classification());
        intensity.push_back(float(lasreader->point.get_intensity()));
        geometry.vertices.push_back({
          float(lasreader->point.get_x()), 
          float(lasreader->point.get_y()), 
          float(lasreader->point.get_z())}
        );
        if(i%100000000==0) std::cout << "Read " << i << " points...\n";
      }
    }
    lasreader->close();
    delete lasreader;

    set_value("points", geometry.vertices);
    set_value("geometry", geometry);
    set_value("classification", classification);
    set_value("intensity", intensity);
  }
};

class OGRLoaderNode:public Node {
  int layer_count = 0;
  
  int current_layer_id = 0;
  std::string geometry_type_name;
  OGRwkbGeometryType geometry_type;

  // GDALDatasetUniquePtr poDS = nullptr;

  public:
  char filepath[256] = "/Users/ravi/surfdrive/Data/step-edge-detector/hoogtelijnen_dgmr_.gpkg";
  // char filepath[256] = "/Users/ravi/surfdrive/Data/step-edge-detector/hoogtelijnen_v01_simp_dp1m.gpkg";
  // char filepath[256] = "/Users/ravi/surfdrive/Projects/RWS-Basisbestand-3D-geluid/3D-basisbestand-geluid-v0.1/output/hoogtelijnen/hoogtelijnen_v2/hoogtelijnen_out";

  OGRLoaderNode(NodeManager& manager):Node(manager, "OGRLoader") {
    add_output("geometries", TT_geometry);
    add_output("vertices", TT_vec3f);
    GDALAllRegister();
  }

  void gui(){
    ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Layer id", &current_layer_id, 0, layer_count-1);
    ImGui::Text(geometry_type_name.c_str());
  }

  void process(){
    GDALDatasetUniquePtr poDS(GDALDataset::Open( filepath, GDAL_OF_VECTOR));
    if( poDS == nullptr )
    {
        std::cerr<<"Open failed.\n";
        return;
    }
    layer_count = poDS->GetLayerCount();
    std::cout << "Layer count: " << layer_count << "\n";
    current_layer_id = 0;

    // Set up vertex data (and buffer(s)) and attribute pointers
    gfGeometry3D geometries;
    vec3f vertices_vec3f;

    OGRLayer  *poLayer;
    poLayer = poDS->GetLayer( current_layer_id );
    std::cout << "Layer " << current_layer_id << " feature count: " << poLayer->GetFeatureCount() << "\n";
    geometry_type = poLayer->GetGeomType();
    geometry_type_name = OGRGeometryTypeToName(geometry_type);
    std::cout << "Layer geometry type: " << geometry_type_name << "\n";

    if(geometry_type == wkbLineString25D || geometry_type == wkbLineStringZM)
      geometries.type = geoflow::line_strip;
    geometries.format = geoflow::count;
    
    // OGREnvelope *poExtent;
    // poLayer->GetExtent(poExtent);
    // geometries.bounding_box.set({poExtent->MinX, poExtent->MinY, 0}, {poExtent->MaxX, poExtent->MaxY, 0});
    poLayer->ResetReading();
    size_t count = 0;
    for( auto& poFeature: poLayer )
    {
      // read feature geometry
      OGRGeometry *poGeometry;
      poGeometry = poFeature->GetGeometryRef();
      // std::cout << "Layer geometry type: " << poGeometry->getGeometryType() << " , " << geometry_type << "\n";
      if( poGeometry != nullptr ) // FIXME: we should check if te layer geometrytype matches with this feature's geometry type. Messy because they can be a bit different eg. wkbLineStringZM and wkbLineString25D
      {
        if (poGeometry->getGeometryType() == wkbLineString25D || poGeometry->getGeometryType() == wkbLineStringZM) {
          OGRLineString *poLineString = poGeometry->toLineString();
          
          geometries.firsts.push_back(count);
          for(auto& poPoint : poLineString){
            std::array<float,3> p = {float(poPoint.getX()), float(poPoint.getY()), float(poPoint.getZ())};
            geometries.vertices.push_back(p);
            geometries.bounding_box.add(p.data());
            count++;
          }
          geometries.counts.push_back(poLineString->getNumPoints());
          

          // temporary code, until Painter is updated to handle the gfGeometry3D struct
          vec3f line;
          for(auto& poPoint : poLineString){
            // std::cout << poPoint.getX() << " " << poPoint.getY() << " " << poPoint.getZ() << "\n";
            line.push_back({float(poPoint.getX()), float(poPoint.getY()), float(poPoint.getZ())});
          }
          
          vertices_vec3f.push_back(line[0]);
          for (size_t i=1; i<(line.size()-1); i++){
            vertices_vec3f.push_back(line[i]);
            vertices_vec3f.push_back(line[i]);
          }
          vertices_vec3f.push_back(line[line.size()-1]);
          // end of temporary code

        } else if( poGeometry != nullptr
                && (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon || poGeometry->getGeometryType() == wkbPolygonZM || poGeometry->getGeometryType() == wkbPolygonM) ) 
        {
          
        } else {
          std::cout << "no supported geometry\n";
        }
      }

    }
    std::cout << "pushed " << geometries.counts.size() << " features...\n";
    set_value("geometries", geometries);
    set_value("vertices", vertices_vec3f);
  }
};
// class OGRLoaderOldNode:public Node {
//   char filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/test-lines.gpkg";
//   // char filepath[256] = "/Users/ravi/surfdrive/Projects/RWS-Basisbestand-3D-geluid/3D-basisbestand-geluid-v0.1/output/hoogtelijnen/hoogtelijnen_v2/hoogtelijnen_out";

//   public:
//   OGRLoaderOldNode(NodeManager& manager):Node(manager, "OGRLoaderOld") {
//     add_output("lines", TT_any);
//     add_output("lines_vec3f", TT_vec3f);
//   }

//   void gui(){
//     ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
//   }

//   void process(){
//     // Set up vertex data (and buffer(s)) and attribute pointers
//     std::vector<vec3f> lines;
//     vec3f lines_vec3f;

//     GDALAllRegister();
//     GDALDataset *poDS = static_cast<GDALDataset*>(
//     GDALOpenEx( filepath, GDAL_OF_VECTOR, NULL, NULL, NULL ));
//     if( poDS == NULL )
//         {
//             std::cerr<<"Open failed.\n";
//             return;
//         }
//     OGRLayer  *poLayer = poDS->GetLayer( 0 );

//     OGREnvelope *poExtent;
//     poLayer->GetExtent(poExtent);
//     auto center_x = poExtent->MaxX - poExtent->MinX;
//     auto center_y = poExtent->MaxY - poExtent->MinY;

//     OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
//         poLayer->ResetReading();
//     OGRFeature *poFeature;
//     while( (poFeature = poLayer->GetNextFeature()) != NULL )
//     {
//       // for( int iField = 0; iField < poFDefn->GetFieldCount(); iField++ )
//       // {
//       //   OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn( iField );
//       //   switch( poFieldDefn->GetType() )
//       //               {
//       //   case OFTInteger:
//       //                       printf( "%d,", poFeature->GetFieldAsInteger( iField ) );
//       //   break;
//       //   case OFTInteger64:
//       //                       printf( CPL_FRMT_GIB ",", poFeature->GetFieldAsInteger64( iField ) );
//       //   break;
//       //   case OFTReal:
//       //                       printf( "%.3f,", poFeature->GetFieldAsDouble(iField) );
//       //   break;
//       //   case OFTString:
//       //                       printf( "%s,", poFeature->GetFieldAsString(iField) );
//       //   break;
//       //   default:
//       //                       printf( "%s,", poFeature->GetFieldAsString(iField) );
//       //   break;
//       //               }
//       // }
//       OGRGeometry *poGeometry = poFeature->GetGeometryRef();
//       if( poGeometry != NULL
//         && wkbFlatten(poGeometry->getGeometryType()) == wkbLineString25D )
//       {
//         OGRLineString *poLineString = (OGRLineString *) poGeometry;

//         auto n = poLineString->getNumPoints();
//         vec3f line;
//         OGRPoint* poPoint;
//         for(size_t i=0; i<n; i++) {
//           poLineString->getPoint(i, poPoint);
//           line.push_back({float(poPoint->getX()-center_x), float(poPoint->getY()-center_y), float(poPoint->getZ())});
//         }
//         lines_vec3f.push_back(line[0]);
//         for (size_t i=1; i<(line.size()-1); i++){
//           lines_vec3f.push_back(line[i]);
//           lines_vec3f.push_back(line[i]);
//         }
//         lines_vec3f.push_back(line[line.size()-1]);
//         lines.push_back(line);

//       } else {
//         std::cout << "no point geometry\n";
//       }
//       OGRFeature::DestroyFeature( poFeature );
//     }
//     GDALClose( poDS );
    
//     // poLayer = poDS->GetLayerByName( "point" );

//     set_value("lines", lines);
//     set_value("lines_vec3f", lines_vec3f);
//   }
// };


class CDTNode:public Node {

  public:
  CDTNode(NodeManager& manager):Node(manager, "CDT") {
    // add_input("points", TT_any);
    add_input("lines_vec3f", TT_vec3f);
    add_output("cgal_CDT", TT_any);
    add_output("normals_vec3f", TT_vec3f);
    add_output("triangles_vec3f", TT_vec3f);
  }

  void gui(){
  }

  void process(){
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef CGAL::Projection_traits_xy_3<K>								Gt;
    typedef CGAL::Exact_predicates_tag									Itag;
    typedef CGAL::Constrained_Delaunay_triangulation_2<Gt, CGAL::Default, Itag>	CDT;
    typedef CDT::Point													Point;

    // Set up vertex data (and buffer(s)) and attribute pointers
    auto lines = std::any_cast<vec3f>(get_value("lines_vec3f"));
   
    CDT cdt;

    for (size_t i=0 ; i<lines.size()/2; i++) {
      cdt.insert_constraint( 
        Point(lines[i*2][0], lines[i*2][1], lines[i*2][2]),
        Point(lines[i*2+1][0], lines[i*2+1][1], lines[i*2+1][2])
      );
    }
    
    std::cout << "Completed CDT with " << cdt.number_of_faces() << " triangles...\n";
    // assert(cdt.is_valid());
    vec3f triangles_vec3f;
    for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
      fit != cdt.finite_faces_end();
      ++fit)
    {
        auto p0 = fit->vertex(0)->point();
        triangles_vec3f.push_back({float(p0.x()), float(p0.y()), float(p0.z())});
        auto p1 = fit->vertex(1)->point();
        triangles_vec3f.push_back({float(p1.x()), float(p1.y()), float(p1.z())});
        auto p2 = fit->vertex(2)->point();
        triangles_vec3f.push_back({float(p2.x()), float(p2.y()), float(p2.z())});
    }

    // set_value("cgal_CDT", cdt);
    set_value("triangles_vec3f", triangles_vec3f);
  }
};

class ComparePointDistanceNode:public Node {
  public:
  char las_filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
  char log_filepath[256] = "ComparePointDistanceNode.out";
  int thin_nth = 20;

  ComparePointDistanceNode(NodeManager& manager):Node(manager, "ComparePointDistance") {
    add_input("triangles1_vec3f", TT_vec3f);
    add_input("triangles2_vec3f", TT_vec3f);
    add_output("points", TT_vec3f);
    add_output("distances1", TT_vec1f);
    add_output("distances2", TT_vec1f);
    add_output("diff", TT_vec1f);
  }

  void gui(){
    ImGui::InputText("LAS file path", las_filepath, IM_ARRAYSIZE(las_filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 0, 100);
  }

  void process(){
    typedef CGAL::Simple_cartesian<double> K;
    typedef K::FT FT;
    typedef K::Ray_3 Ray;
    typedef K::Line_3 Line;
    typedef K::Point_3 Point;
    typedef K::Triangle_3 Triangle;
    typedef std::list<Triangle>::iterator Iterator;
    typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
    typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
    typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

    // Triangles 1
    auto trin1 = std::any_cast<vec3f>(get_value("triangles1_vec3f"));
    std::list<Triangle> triangles1;
    for(size_t i=0; i< trin1.size()/3; i++){
      auto a = Point(trin1[i*3+0][0], trin1[i*3+0][1], trin1[i*3+0][2]);
      auto b = Point(trin1[i*3+1][0], trin1[i*3+1][1], trin1[i*3+1][2]);
      auto c = Point(trin1[i*3+2][0], trin1[i*3+2][1], trin1[i*3+2][2]);
      triangles1.push_back(Triangle(a,b,c));
    }
    Tree tree1(triangles1.begin(),triangles1.end());
    tree1.accelerate_distance_queries();

    // Triangles 2
    auto trin2 = std::any_cast<vec3f>(get_value("triangles2_vec3f"));
    std::list<Triangle> triangles2;
    for(size_t i=0; i< trin2.size()/3; i++){
      auto a = Point(trin2[i*3+0][0], trin2[i*3+0][1], trin2[i*3+0][2]);
      auto b = Point(trin2[i*3+1][0], trin2[i*3+1][1], trin2[i*3+1][2]);
      auto c = Point(trin2[i*3+2][0], trin2[i*3+2][1], trin2[i*3+2][2]);
      triangles2.push_back(Triangle(a,b,c));
    }
    Tree tree2(triangles2.begin(),triangles2.end());
    tree2.accelerate_distance_queries();

    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(las_filepath);
    LASreader* lasreader = lasreadopener.open();

    vec1f distances1, distances2, diff;
    vec3f points;
    std::ofstream f_out(log_filepath);
    f_out << std::fixed << std::setprecision(2);
    size_t i=0;
    while (lasreader->read_point()) {
      if (lasreader->point.get_classification() == 2){

        if (i++ % thin_nth == 0){
          auto q = Point(lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z());
          float d1 = std::sqrt(tree1.squared_distance(q));
          distances1.push_back(d1);
          float d2 = std::sqrt(tree2.squared_distance(q));
          distances2.push_back(d2);
          auto difference = d2-d1;
          diff.push_back(difference);
          points.push_back({
            float(lasreader->point.get_x()), 
            float(lasreader->point.get_y()), 
            float(lasreader->point.get_z())}
          );
          f_out << float(lasreader->point.get_x()) << " " << float(lasreader->point.get_y()) << " " << float(lasreader->point.get_z()) << " ";
          f_out << std::sqrt(d1) << " " << std::sqrt(d2) << " " << difference << "\n";
        }
        if(i%100000000==0) std::cout << "Read " << i << " points...\n";
        // laswriter->write_point(&lasreader->point);
      }
    }
    lasreader->close();
    delete lasreader;
    f_out.close();
    
    for(int i=0; i<points.size(); i++) {
      f_out << points[i][0] << " " << points[i][1] << " " << points[i][2] << " ";
      f_out << std::sqrt(distances1[i]) << " " << std::sqrt(distances2[i]) << " " << diff[i] << "\n";
    }
    

    set_value("points", points);
    set_value("diff", diff);
    set_value("distances1", distances1);
    set_value("distances2", distances2);
  }
};

class PointDistanceNode:public Node {
  public:
  char filepath[256] = "/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ";
  int thin_nth = 5;

  PointDistanceNode(NodeManager& manager):Node(manager, "PointDistance") {
    add_input("triangles_vec3f", TT_vec3f);
    add_output("points", TT_vec3f);
    add_output("distances", TT_vec1f);
  }

  void gui(){
    ImGui::InputText("LAS file path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 0, 100);
  }

  void process(){
    typedef CGAL::Simple_cartesian<double> K;
    typedef K::FT FT;
    typedef K::Ray_3 Ray;
    typedef K::Line_3 Line;
    typedef K::Point_3 Point;
    typedef K::Triangle_3 Triangle;
    typedef std::list<Triangle>::iterator Iterator;
    typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
    typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
    typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

    auto trin = std::any_cast<vec3f>(get_value("triangles"));
    std::list<Triangle> triangles;
    for(size_t i=0; i< trin.size()/3; i++){
      auto a = Point(trin[i*3+0][0], trin[i*3+0][1], trin[i*3+0][2]);
      auto b = Point(trin[i*3+1][0], trin[i*3+1][1], trin[i*3+1][2]);
      auto c = Point(trin[i*3+2][0], trin[i*3+2][1], trin[i*3+2][2]);
      triangles.push_back(Triangle(a,b,c));
    }
    
    // constructs AABB tree
    Tree tree(triangles.begin(),triangles.end());
    tree.accelerate_distance_queries();

    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(filepath);
    LASreader* lasreader = lasreadopener.open();

    vec1f distances;
    vec3f points;
    size_t i=0;
    while (lasreader->read_point()) {
      if (lasreader->point.get_classification() == 2){

        if (i++ % thin_nth == 0){
          auto q = Point(lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z());
          FT sqd = tree.squared_distance(q);
          distances.push_back(sqd);
          points.push_back({
            float(lasreader->point.get_x()), 
            float(lasreader->point.get_y()), 
            float(lasreader->point.get_z())}
          );
        }
        if(i%10000==0) std::cout << "Read " << i << " points...\n";
        // laswriter->write_point(&lasreader->point);
      }
    }
    lasreader->close();
    delete lasreader;

    set_value("points", points);
    set_value("distances", distances);
  }
};

class CSVLoaderNode:public Node {
  public:
  char filepath[256] = "/Users/ravi/git/heightjump-detect/build/ComparePointDistanceNode.out";
  int thin_nth = 5;

  CSVLoaderNode(NodeManager& manager):Node(manager, "CSVLoader") {
    add_output("points", TT_vec3f);
    add_output("distances1", TT_vec1f);
    add_output("distances2", TT_vec1f);
    add_output("difference", TT_vec1f);
  }

  void gui(){
    ImGui::InputText("CSV file path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Thin nth", &thin_nth, 0, 100);
  }

  void process(){
    vec3f points;
    vec1f distances1;
    vec1f distances2;
    vec1f difference;
    
    std::ifstream f_in(filepath);
    float px, py, pz, d1, d2, df;
    size_t i=0;
    while(f_in >> px >> py >> pz >> d1 >> d2 >> df) {
      if(i++%thin_nth==0) {
        points.push_back({px,py,pz});
        distances1.push_back(d1);
        distances2.push_back(d2);
        difference.push_back(df);
      }
    }
    f_in.close();

    set_value("points", points);
    set_value("distances1", distances1);
    set_value("distances2", distances2);
    set_value("difference", difference);
  }
};

class TinSimpNode:public Node {
  public:
  float thres_error = 2;

  TinSimpNode(NodeManager& manager):Node(manager, "TinSimp") {
    add_input("geometry", TT_geometry);
    add_output("triangles_vec3f", TT_vec3f);
    add_output("lines_vec3f", TT_vec3f);
  }

  void gui(){
    ImGui::SliderFloat("Error threshold", &thres_error, 0, 100);
  }

  void process(){
    auto geometry = std::any_cast<gfGeometry3D>(get_value("geometry"));

    float min_x = geometry.bounding_box.min()[0]-1;
    float min_y = geometry.bounding_box.min()[1]-1;
    float max_x = geometry.bounding_box.max()[0]+1;
    float max_y = geometry.bounding_box.max()[1]+1;
    float center_z = (geometry.bounding_box.max()[2]-geometry.bounding_box.min()[2])/2;

    tinsimp::CDT cdt;

    std::vector<tinsimp::Point> initial_points = {
      tinsimp::Point(min_x, min_y, center_z),
      tinsimp::Point(max_x, min_y, center_z),
      tinsimp::Point(max_x, max_y, center_z),
      tinsimp::Point(min_x, max_y, center_z)
    };
    cdt.insert(initial_points.begin(), initial_points.end());

    if (geometry.type == geoflow::points) {
      tinsimp::greedy_insert(cdt, geometry.vertices, double(thres_error));

    } else if (geometry.type == geoflow::line_strip) {
      auto selected_lines = tinsimp::greedy_insert_lines(cdt, geometry.vertices, geometry.counts, double(thres_error));
      vec3f lines_vec3f;
      for (auto line_id : selected_lines) {
        auto first = geometry.firsts[line_id];
        lines_vec3f.push_back(geometry.vertices[first]);
          for (size_t i=1; i<(geometry.counts[line_id]-1); i++){
            lines_vec3f.push_back(geometry.vertices[first+i]);
            lines_vec3f.push_back(geometry.vertices[first+i]);
          }
          lines_vec3f.push_back(geometry.vertices[geometry.counts[line_id]]);
      }
      set_value("lines_vec3f", lines_vec3f);
    }

    vec3f triangles_vec3f;
    for (tinsimp::CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
      fit != cdt.finite_faces_end();
      ++fit)
    {
        auto p0 = fit->vertex(0)->point();
        triangles_vec3f.push_back({float(p0.x()), float(p0.y()), float(p0.z())});
        auto p1 = fit->vertex(1)->point();
        triangles_vec3f.push_back({float(p1.x()), float(p1.y()), float(p1.z())});
        auto p2 = fit->vertex(2)->point();
        triangles_vec3f.push_back({float(p2.x()), float(p2.y()), float(p2.z())});
    }

    set_value("triangles_vec3f", triangles_vec3f);

  }
};
