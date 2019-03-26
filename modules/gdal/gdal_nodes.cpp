#include "gdal_nodes.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> point_type;
typedef bg::model::point<double, 3, bg::cs::cartesian> point_type_3d;

#include <unordered_map>
#include <variant>
#include <fstream>

namespace geoflow::nodes::gdal {
  
  void OGRLoaderNode::push_attributes(OGRFeature& poFeature) {
    for(auto& [name, term] : output_group("attributes").terminals) {
      if(term->type == typeid(vec1i)) {
//        std::cout << term->has_data() << "\n";
        auto& term_data = term->get<vec1i&>();
        term_data.push_back(poFeature.GetFieldAsInteger64(name.c_str()));
      } else if(term->type == typeid(vec1f)) {
        auto& term_data = term->get<vec1f&>();
        term_data.push_back(poFeature.GetFieldAsDouble(name.c_str()));
      } else if(term->type == typeid(vec1s)) {
        auto& term_data = term->get<vec1s&>();
        term_data.push_back( (std::string) poFeature.GetFieldAsString(name.c_str()) );
      }
    }
  }
  
  void OGRLoaderNode::process(){
    GDALDatasetUniquePtr poDS(GDALDataset::Open( param<std::string>("filepath").c_str(), GDAL_OF_VECTOR));
    if( poDS == nullptr )
    {
        std::cerr<<"Open failed.\n";
        return;
    }
    layer_count = poDS->GetLayerCount();
    std::cout << "Layer count: " << layer_count << "\n";
    param<int>("layer_id") = 0;

    // Set up vertex data (and buffer(s)) and attribute pointers
    LineStringCollection line_strings;
    LinearRingCollection linear_rings;

    OGRLayer  *poLayer;
    poLayer = poDS->GetLayer( param<int>("layer_id") );
    std::cout << "Layer " << param<int>("layer_id") << " feature count: " << poLayer->GetFeatureCount() << "\n";
    geometry_type = poLayer->GetGeomType();
    geometry_type_name = OGRGeometryTypeToName(geometry_type);
    std::cout << "Layer geometry type: " << geometry_type_name << "\n";


    auto layer_def = poLayer->GetLayerDefn();
    auto field_count = layer_def->GetFieldCount();
  
//    output_group("attributes").clear();
    for (size_t i=0; i < field_count; ++i) {
      auto field_def = layer_def->GetFieldDefn(i);
      auto t = field_def->GetType();
      auto field_name = (std::string) field_def->GetNameRef();
      if (t==OFTInteger || t==OFTInteger64) {
        auto& term = output_group("attributes").add(field_name, typeid(vec1i));
        term.set(vec1i());
      } else if (t==OFTString) {
        auto& term = output_group("attributes").add(field_name, typeid(vec1s));
        term.set(vec1s());
      } else if (t==OFTReal) {
        auto& term = output_group("attributes").add(field_name, typeid(vec1f));
        term.set(vec1f());
      }
    }

    bool found_offset = manager.data_offset.has_value();
    poLayer->ResetReading();

    for( auto& poFeature: poLayer )
    {
      // read feature geometry
      OGRGeometry *poGeometry;
      poGeometry = poFeature->GetGeometryRef();
      // std::cout << "Layer geometry type: " << poGeometry->getGeometryType() << " , " << geometry_type << "\n";
      if( poGeometry != nullptr ) // FIXME: we should check if te layer geometrytype matches with this feature's geometry type. Messy because they can be a bit different eg. wkbLineStringZM and wkbLineString25D
      {

        if (
          poGeometry->getGeometryType() == wkbLineString25D || poGeometry->getGeometryType() == wkbLineStringZM ||
          poGeometry->getGeometryType() == wkbLineString) {
          OGRLineString *poLineString = poGeometry->toLineString();
          
          vec3f line_string;
          for(auto& poPoint : poLineString){
            if (!found_offset) {
              manager.data_offset = {poPoint.getX(), poPoint.getY(), 0};
              found_offset = true;
            }
            std::array<float,3> p = {
              float(poPoint.getX()-(*manager.data_offset)[0]), 
              float(poPoint.getY()-(*manager.data_offset)[1]), 
              float(poPoint.getZ()-(*manager.data_offset)[2])
            };
            line_string.push_back(p);
          }
          line_strings.push_back(line_string);
          
          push_attributes(*poFeature);

        } else if (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon || poGeometry->getGeometryType() == wkbPolygonZM || poGeometry->getGeometryType() == wkbPolygonM ) {
          OGRPolygon *poPolygon = poGeometry->toPolygon();
    
          vec3f ring;
          // for(auto& poPoint : poPolygon->getExteriorRing()) {
          OGRPoint poPoint;
          for(size_t i=0; i<poPolygon->getExteriorRing()->getNumPoints()-1; ++i) {
            poPolygon->getExteriorRing()->getPoint(i, &poPoint);
            if (!found_offset) {
              manager.data_offset = {poPoint.getX(), poPoint.getY(), 0};
              found_offset = true;
            }
            std::array<float,3> p = {float(poPoint.getX()-(*manager.data_offset)[0]), float(poPoint.getY()-(*manager.data_offset)[1]), float(poPoint.getZ()-(*manager.data_offset)[2])};
            ring.push_back(p);
          }
          // ring.erase(--ring.end());
          // bg::model::polygon<point_type_3d> boost_poly;
          // for (auto& p : ring) {
          //   bg::append(boost_poly.outer(), point_type_3d(p[0], p[1], p[2]));
          // }
          // bg::unique(boost_poly);
          // vec3f ring_dedup;
          // for (auto& p : boost_poly.outer()) {
          //   ring_dedup.push_back({float(bg::get<0>(p)), float(bg::get<1>(p)), float(bg::get<2>(p))}); //FIXME losing potential z...
          // }
          linear_rings.push_back(ring);
          
          push_attributes(*poFeature);

        } else {
          std::cout << "unsupported geometry\n";
        }
      }

    }
    // if (geometry_type == wkbLineString25D || geometry_type == wkbLineStringZM) {
    if (line_strings.size() > 0) {
      output("line_strings").set(line_strings);
      std::cout << "pushed " << line_strings.size() << " line_string features...\n";
    // } else if (geometry_type == wkbPolygon || geometry_type == wkbPolygon25D || geometry_type == wkbPolygonZM || geometry_type == wkbPolygonM) {
    } else if (linear_rings.size() > 0) {
      output("linear_rings").set(linear_rings);
      std::cout << "pushed " << linear_rings.size() << " linear_ring features...\n";
    }
    
//    for(auto& [name, term] : output_group("attributes").terminals) {
//      std::cout << "group_term " << name << "\n";
//      if (term->type == typeid(vec1f))
//        for (auto& val : term->get<vec1f>()) {
//          std::cout << "\t" << val << "\n";
//        }
//      if (term->type == typeid(vec1i))
//        for (auto& val : term->get<vec1i>()) {
//          std::cout << "\t" << val << "\n";
//        }
//      if (term->type == typeid(vec1s))
//        for (auto& val : term->get<vec1s>()) {
//          std::cout << "\t" << val << "\n";
//        }
//    }
  }

void OGRWriterNode::process() {
    auto geom_term = input("geometries");

//    const char *gszDriverName = "ESRI Shapefile";
    const char *gszDriverName = "GPKG";
    GDALDriver *poDriver;

    GDALAllRegister();

    poDriver = GetGDALDriverManager()->GetDriverByName(gszDriverName );
    if( poDriver == NULL )
    {
        printf( "%s driver not available.\n", gszDriverName );
        exit( 1 );
    }

    GDALDataset *poDS;

    poDS = poDriver->Create( param<std::string>("filepath").c_str(), 0, 0, 0, GDT_Unknown, NULL );
    if( poDS == NULL )
    {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }

    OGRSpatialReference oSRS;
    OGRLayer *poLayer;

    oSRS.importFromEPSG( epsg ); 
    OGRwkbGeometryType wkbType;
    std::variant<LineStringCollection,LinearRingCollection> geometry_collection;
    size_t geom_count;
    if (geom_term.connected_type == typeid(LinearRingCollection)) {
      wkbType = wkbPolygon;
      geometry_collection = geom_term.get<LinearRingCollection>();
      geom_count = std::get<LinearRingCollection>(geometry_collection).size();
    } else if (geom_term.connected_type == typeid(LineStringCollection)) {
      wkbType = wkbLineString25D;
      geometry_collection = geom_term.get<LineStringCollection>();
      geom_count = std::get<LineStringCollection>(geometry_collection).size();
    }
    poLayer = poDS->CreateLayer( "geom", &oSRS, wkbType, NULL );
    if( poLayer == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }

    std::unordered_map<std::string, size_t> attr_id_map;
    int fcnt = poLayer->GetLayerDefn()->GetFieldCount();
    for(auto& [name, term] : input_group("attributes").terminals) {
//      std::cout << "group_term " << name << "\n";
      if (term->connected_type == typeid(vec1f)) {
        OGRFieldDefn oField( name.c_str(), OFTReal );
        if( poLayer->CreateField( &oField ) != OGRERR_NONE )
        {
            printf( "Creating Name field failed.\n" );
            exit( 1 );
        }
        attr_id_map[name] = fcnt++;
      }else if (term->connected_type == typeid(vec1i)) {
        OGRFieldDefn oField( name.c_str(), OFTInteger64 );
        if( poLayer->CreateField( &oField ) != OGRERR_NONE )
        {
          printf( "Creating Name field failed.\n" );
          exit( 1 );
        }
        attr_id_map[name] = fcnt++;
      } else if (term->connected_type == typeid(vec1s)) {
        OGRFieldDefn oField( name.c_str(), OFTString );
        if( poLayer->CreateField( &oField ) != OGRERR_NONE )
        {
          printf( "Creating Name field failed.\n" );
          exit( 1 );
        }
        attr_id_map[name] = fcnt++;
      }
    }
  
    for (size_t i = 0; i != geom_count; ++i)
    {
        OGRFeature *poFeature;
        poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
        for(auto& [tname, term] : input_group("attributes").terminals) {
          if (term->connected_type == typeid(vec1f)) {
            auto& val = term->get<vec1f&>();
            poFeature->SetField( attr_id_map[tname], val[i] );
          } else if (term->connected_type == typeid(vec1i)) {
            auto& val = term->get<vec1i&>();
            poFeature->SetField( attr_id_map[tname], val[i] );
          } else if (term->connected_type == typeid(vec1s)) {
            auto& val = term->get<vec1s&>();
            poFeature->SetField( attr_id_map[tname], val[i].c_str() );
          }
        }
      
        if (geom_term.connected_type == typeid(LinearRingCollection)) {
          OGRLinearRing ogrring;
          LinearRingCollection& lr = std::get<LinearRingCollection>(geometry_collection);
          for (auto const& g: lr[i]) {
              ogrring.addPoint( g[0]+(*manager.data_offset)[0], g[1]+(*manager.data_offset)[1], g[2]+(*manager.data_offset)[2] );
          }
          ogrring.closeRings();
          OGRPolygon bouwpoly;
          bouwpoly.addRing( &ogrring );
          poFeature->SetGeometry( &bouwpoly );
        }
        if (geom_term.connected_type == typeid(LineStringCollection)) {
          OGRLineString ogrlinestring;
          LineStringCollection& ls = std::get<LineStringCollection>(geometry_collection);
          for (auto const& g: ls[i]) {
              ogrlinestring.addPoint( g[0]+(*manager.data_offset)[0], g[1]+(*manager.data_offset)[1], g[2]+(*manager.data_offset)[2] );
          }
          poFeature->SetGeometry( &ogrlinestring );
        }

        if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
        {
            printf( "Failed to create feature in geopackage.\n" );
            exit( 1 );
        }
        OGRFeature::DestroyFeature( poFeature );
    }
    GDALClose( poDS );
}

void OGRWriterNoAttributesNode::process() {
    auto geom_term = input("geometries");

    const char *gszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;

    GDALAllRegister();

    poDriver = GetGDALDriverManager()->GetDriverByName(gszDriverName );
    if( poDriver == NULL )
    {
        printf( "%s driver not available.\n", gszDriverName );
        exit( 1 );
    }

    GDALDataset *poDS;

    poDS = poDriver->Create( param<std::string>("filepath").c_str(), 0, 0, 0, GDT_Unknown, NULL );
    if( poDS == NULL )
    {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }

    OGRSpatialReference oSRS;
    OGRLayer *poLayer;

    oSRS.importFromEPSG( epsg ); 
    OGRwkbGeometryType wkbType;
    std::variant<LineStringCollection,LinearRingCollection> geometry_collection;
    size_t geom_count;
    if (geom_term.connected_type == typeid(LinearRingCollection)) {
      wkbType = wkbPolygon;
      geometry_collection = geom_term.get<LinearRingCollection>();
      geom_count = std::get<LinearRingCollection>(geometry_collection).size();
    } else if (geom_term.connected_type == typeid(LineStringCollection)) {
      wkbType = wkbLineString25D;
      geometry_collection = geom_term.get<LineStringCollection>();
      geom_count = std::get<LineStringCollection>(geometry_collection).size();
    }
    poLayer = poDS->CreateLayer( "geom", &oSRS, wkbType, NULL );
    if( poLayer == NULL )
    {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }

    // do the actual geometry conversion and writing here
    for (size_t i = 0; i != geom_count; ++i)
    {
        OGRFeature *poFeature;
        poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );

        
        if (geom_term.connected_type == typeid(LinearRingCollection)) {
          OGRLinearRing ogrring;
          LinearRingCollection& lr = std::get<LinearRingCollection>(geometry_collection);
          for (auto const& g: lr[i]) {
              ogrring.addPoint( g[0]+(*manager.data_offset)[0], g[1]+(*manager.data_offset)[1], g[2]+(*manager.data_offset)[2] );
          }
          ogrring.closeRings();
          OGRPolygon bouwpoly;
          bouwpoly.addRing( &ogrring );
          poFeature->SetGeometry( &bouwpoly );
        }
        if (geom_term.connected_type == typeid(LineStringCollection)) {
          OGRLineString ogrlinestring;
          LineStringCollection& ls = std::get<LineStringCollection>(geometry_collection);
          for (auto const& g: ls[i]) {
              ogrlinestring.addPoint( g[0]+(*manager.data_offset)[0], g[1]+(*manager.data_offset)[1], g[2]+(*manager.data_offset)[2] );
          }
          poFeature->SetGeometry( &ogrlinestring );
        }

        if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
        {
            printf( "Failed to create feature in geopackage.\n" );
            exit( 1 );
        }
        OGRFeature::DestroyFeature( poFeature );
    }
    GDALClose( poDS );
}

void CSVLoaderNode::process(){
  PointCollection points;
  auto& thin_nth = param<int>("thin_nth");
  
  std::ifstream f_in(param<std::string>("filepath"));
  float px, py, pz;
  size_t i=0;
  std::string header;
  std::getline(f_in, header);
  while(f_in >> px >> py >> pz) {
    if(i++%thin_nth==0) {
      points.push_back({px,py,pz});
    }
  }
  f_in.close();

  output("points").set(points);\
}

void CSVWriterNode::process() {
  auto points = input("points").get<PointCollection>();
  auto distances = input("distances").get<vec1f>();
  
  std::ofstream f_out(param<std::string>("filepath"));
  f_out << std::fixed << std::setprecision(2);
  f_out << "x y z distance\n";
  for (size_t i=0; i< points.size(); ++i) {
    f_out 
    << points[i][0] + (*manager.data_offset)[0] << " " 
    << points[i][1] + (*manager.data_offset)[1] << " " 
    << points[i][2] + (*manager.data_offset)[2] << " "
    << distances[i] << "\n";
  }
  f_out.close();
}

void GEOSMergeLinesNode::process() {
    std::cout << "Merging lines\n";
    auto lines = input("lines").get<LineStringCollection>();
    GEOSContextHandle_t gc = GEOS_init_r();
    std::vector<GEOSGeometry*> linearray;
    for (int i = 0; i < lines.size(); i++) {
      GEOSCoordSequence* points = GEOSCoordSeq_create_r(gc, 2, 3);
      for (int j = 0; j < 2; j++) {
        GEOSCoordSeq_setX_r(gc, points, j, lines[i][j][0]);
        GEOSCoordSeq_setY_r(gc, points, j, lines[i][j][1]);
        GEOSCoordSeq_setZ_r(gc, points, j, lines[i][j][2]);
      }
      GEOSGeometry* line = GEOSGeom_createLineString_r(gc, points);
      linearray.push_back(line);
    }
    GEOSGeometry* lineCollection = GEOSGeom_createCollection_r(gc, GEOS_GEOMETRYCOLLECTION, linearray.data(), lines.size());
    GEOSGeometry* mergedlines = GEOSLineMerge_r(gc, lineCollection);

    LineStringCollection outputLines;
    for (int i = 0; i < GEOSGetNumGeometries_r(gc, mergedlines); i++) {
      const GEOSCoordSequence* l = GEOSGeom_getCoordSeq_r(gc, GEOSGetGeometryN_r(gc, mergedlines, i));
      vec3f line_vec3f;
      unsigned int size;
      GEOSCoordSeq_getSize_r(gc, l, &size);
      for (int j = 0; j < size;j++) {
        double x,y,z = 0;
        GEOSCoordSeq_getX_r(gc, l, j, &x);
        GEOSCoordSeq_getY_r(gc, l, j, &y);
        GEOSCoordSeq_getZ_r(gc, l, j, &z);
        line_vec3f.push_back({ float(x),float(y),float(z) });
      }
      outputLines.push_back(line_vec3f);
    }

    // clean GEOS geometries
    for (auto l : linearray) {
      GEOSGeom_destroy_r(gc, l);
    }
    //GEOSGeom_destroy_r(gc, lineCollection);
    GEOSGeom_destroy_r(gc, mergedlines);
    GEOS_finish_r(gc);

    output("lines").set(outputLines);
}
}
