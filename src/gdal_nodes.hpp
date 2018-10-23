#include "imgui.h"
#include "geoflow.hpp"
#include "ogrsf_frmts.h"

#include <unordered_map>

using namespace geoflow;

class OGRLoaderNode:public Node {
  int layer_count = 0;
  
  int current_layer_id = 0;
  std::string geometry_type_name;
  OGRwkbGeometryType geometry_type;

  // GDALDatasetUniquePtr poDS = nullptr;

  public:
  // char filepath[256] = "/Users/ravi/surfdrive/Data/step-edge-detector/hoogtelijnen_dgmr_.gpkg";
  char filepath[256] = "/Users/ravi/surfdrive/Data/step-edge-detector/hoogtelijnen_v01_simp_dp1m_subset.gpkg";
  // char filepath[256] = "/Users/ravi/surfdrive/Projects/RWS-Basisbestand-3D-geluid/3D-basisbestand-geluid-v0.1/output/hoogtelijnen/hoogtelijnen_v2/hoogtelijnen_out";

  OGRLoaderNode(NodeManager& manager):Node(manager, "OGRLoader") {
    add_output("geometries", TT_geometry);
    add_output("features", TT_any);
    add_output("vertices", TT_vec3f);
    GDALAllRegister();
  }

  void gui(){
    ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    ImGui::SliderInt("Layer id", &current_layer_id, 0, layer_count-1);
    ImGui::Text("%s", geometry_type_name.c_str());
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
    Feature features;

    OGRLayer  *poLayer;
    poLayer = poDS->GetLayer( current_layer_id );
    std::cout << "Layer " << current_layer_id << " feature count: " << poLayer->GetFeatureCount() << "\n";
    geometry_type = poLayer->GetGeomType();
    geometry_type_name = OGRGeometryTypeToName(geometry_type);
    std::cout << "Layer geometry type: " << geometry_type_name << "\n";

    if (geometry_type == wkbLineString25D || geometry_type == wkbLineStringZM) {
      geometries.type = geoflow::line_strip;
    } else if (geometry_type == wkbPolygon || geometry_type == wkbPolygon25D || geometry_type == wkbPolygonZM || geometry_type == wkbPolygonM) {
      geometries.type = geoflow::line_loop;
      features.type = geoflow::line_loop;
    }
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
          features.geom.push_back(line);
          
          vertices_vec3f.push_back(line[0]);
          for (size_t i=1; i<(line.size()-1); i++){
            vertices_vec3f.push_back(line[i]);
            vertices_vec3f.push_back(line[i]);
          }
          vertices_vec3f.push_back(line[line.size()-1]);
          // end of temporary code

        } else if (poGeometry->getGeometryType() == wkbPolygon25D || poGeometry->getGeometryType() == wkbPolygon || poGeometry->getGeometryType() == wkbPolygonZM ||    poGeometry->getGeometryType() == wkbPolygonM ) {
          OGRPolygon *poPolygon = poGeometry->toPolygon();
    
          vec3f ring;
          for(auto& poPoint : poPolygon->getExteriorRing()){
            std::array<float,3> p = {float(poPoint.getX()), float(poPoint.getY()), float(poPoint.getZ())};
            ring.push_back(p);
          }
          ring.erase(ring.end()-1);
          features.geom.push_back(ring);

        } else {
          std::cout << "no supported geometry\n";
        }
      }

    }
    std::cout << "pushed " << geometries.counts.size() << " features...\n";
    set_value("features", features);
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

class OGRWriterNode:public Node {
    public:
    char filepath[256] = "blabla.gpkg";
    std::string epsg_string = "EPSG:7415";
    
    
    OGRWriterNode(NodeManager& manager):Node(manager, "OGRWriter") {
        add_input("features", TT_any); // struct with 'type' attrubute and a std::vector of vectors of coordinates
        // add_input("attributes", TT_any); // unordered map with attribute names as keys and the values as values (similar to python dict)
    }

    void gui() {
      ImGui::InputText("File path", filepath, IM_ARRAYSIZE(filepath));
    }
    
    void process() {
        // do these getter functions exist? <- yes they are defined in the base class geoflow::Node
        auto features = std::any_cast<Feature>(get_value("features"));
        // auto attributes = std::any_cast<Attributes>(get_value("attributes"));
        // what about the attributes?

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

        poDS = poDriver->Create( filepath, 0, 0, 0, GDT_Unknown, NULL );
        if( poDS == NULL )
        {
            printf( "Creation of output file failed.\n" );
            exit( 1 );
        }

        OGRSpatialReference oSRS;
        OGRLayer *poLayer;

        oSRS.SetWellKnownGeogCS( epsg_string.c_str() ); 
        OGRwkbGeometryType wkbType;
        if (features.type == line_loop) {
          wkbType = wkbPolygon;
        } else if (features.type == line_strip) {
          wkbType = wkbLineString25D;
        }
        poLayer = poDS->CreateLayer( "geom", &oSRS, wkbType, NULL );
        if( poLayer == NULL )
        {
            printf( "Layer creation failed.\n" );
            exit( 1 );
        }

        for (auto& attr : features.attr) {
          OGRFieldDefn oField( attr.first.c_str(), OFTReal );
          if( poLayer->CreateField( &oField ) != OGRERR_NONE )
          {
              printf( "Creating Name field failed.\n" );
              exit( 1 );
          }
        }
        // do the actual geometry conversion and writing here
        for (std::vector<int>::size_type i = 0; i != features.geom.size(); i++)
        {
            OGRFeature *poFeature;
            poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
            for (auto& attr : features.attr) {
              poFeature->SetField( attr.first.c_str(), attr.second[i] );
            }
            
            if (features.type == line_loop) {
              OGRLinearRing ogrring;
              for (auto const& g: features.geom[i]) {
                  ogrring.addPoint( g[0], g[1], g[2] );
              }
              ogrring.closeRings();
              OGRPolygon bouwpoly;
              bouwpoly.addRing( &ogrring );
              poFeature->SetGeometry( &bouwpoly );
            }
            if (features.type == line_strip) {
              OGRLineString ogrlinestring;
              for (auto const& g: features.geom[i]) {
                  ogrlinestring.addPoint( g[0], g[1], g[2] );
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
