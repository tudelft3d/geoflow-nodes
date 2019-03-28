#include <fstream>
#include <geoflow/core/geoflow.hpp>
#include <nlohmann/json.hpp>

namespace geoflow::nodes::cityjson {
  
  static std::unordered_map <std::string, int> st_map = 
  {
    {"RoofSurface", 0},
    {"GroundSurface",1},
    {"WallSurface", 2},
    {"ClosureSurface", 3},
    {"OuterCeilingSurface", 4},
    {"OuterFloorSurface", 5},
    {"Window", 6},
    {"Door", 7}
  };

  class CityJSONReaderNode : public Node {
    public:
    using Node::Node;
    
    void init() {
      // declare ouput terminals
      add_output("faces", typeid(LinearRingCollection));
      add_output("surface_types", typeid(vec1i));

      // declare parameters
      add_param("filepath", (std::string) "DenHaag_01.json");
      add_param("extract_lod", (int) 2);
    }

    // GUI to configure the  parameters
    void gui() {
      ImGui::InputText("File path", &param<std::string>("filepath"));
      ImGui::SliderInt("Extract only LoD", &param<int>("extract_lod"), 0, 3);
    }

    void process() {
      // get filepath from paramter
      auto& file_path = param<std::string>("filepath");

      // read json file from disk
      std::ifstream inputStream(file_path);
      nlohmann::json json;
      try {
        inputStream >> json;
      } catch (const std::exception& e) {
        std::cerr << e.what();
        return;
      }

      // extract geometries
      // WARNING: this is code is only written to work with the dataset 'DenHaag_01.json', expect crashes with other files
      std::vector<std::vector<double>> verts = json["vertices"];
      std::vector<double> scale = json["transform"]["scale"];

      LinearRingCollection faces;
      vec1i surface_types;
      for (const auto& cobject : json["CityObjects"]) {
        // iterate all geometries
        for (const auto& geom : cobject["geometry"]) {
          if (
            geom["type"] == "Solid" && // only care about solids
            geom["lod"] == param<int>("extract_lod") // of this LoD
          ) {
            size_t face_cnt = 0;
            // get faces of exterior shell
            for (const auto& ext_face : geom["boundaries"][0]) {
              LinearRing ring;
              for (const auto& i : ext_face[0]) { // get vertices of outer rings
                ring.push_back({
                  float(verts[i][0] * scale[0]), 
                  float(verts[i][1] * scale[1]), 
                  float(verts[i][2] * scale[2])
                });
                // get the surface type
              }
              int value = geom["semantics"]["values"][0][face_cnt++];
              const std::string type_string = geom["semantics"]["surfaces"][value]["type"];
              surface_types.push_back(st_map[type_string]);
              faces.push_back(ring);
            }
          }
        }
      }

      // set outputs
      output("surface_types").set(surface_types);
      output("faces").set(faces);
    }
  };

  // Create a NodeRegister, ie a list of all available nodes
  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create("CityJSON");
    R->register_node<CityJSONReaderNode>("CityJSONReader");
    return R;
  }
}