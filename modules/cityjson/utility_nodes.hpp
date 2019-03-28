#include <fstream>

#include <geoflow/core/geoflow.hpp>

#include <earcut.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace geoflow::nodes::utility {

  class RingTriangulatorNode:public Node {

  glm::vec3 calculate_normal(const LinearRing& ring)  {
    glm::vec3 normal(0,0,0);
    for(size_t i=0; i< ring.size(); ++i){
      const auto& curr = ring[i];
      const auto& next = ring[(i+1)%ring.size()];
      normal[0] += (curr[1]-next[1])*(curr[2]+next[2]);
      normal[1] += (curr[2]-next[2])*(curr[0]+next[0]);
      normal[2] += (curr[0]-next[0])*(curr[1]+next[1]);
    }
    return glm::normalize(normal);
  }

  vec2f project_ring_2d(const LinearRing& ring, const glm::vec3& n) {
    vec2f ring_2d;
    auto nx = glm::normalize(glm::vec3(-n.y, n.x, 0));
    auto ny = glm::normalize(glm::cross(nx,n));
    for (const auto& p : ring) {
      auto pp = glm::make_vec3(p.data());
      ring_2d.push_back({
        float(glm::dot(pp,nx)), 
        float(glm::dot(pp,ny))
      });
    }
    return ring_2d;
  }


  public:
    using Node::Node;
    
    void init() {
      // declare ouput terminals
      add_input("rings", typeid(LinearRingCollection));
      add_input("values", typeid(vec1i));
      add_output("values", typeid(vec1i));
      add_output("triangles", typeid(TriangleCollection));
      add_output("normals", typeid(vec3f));

      // declare parameters
      // add_param("extract_lod", (int) 2);
    }

    // GUI to configure the  parameters
    void gui() {
      // ImGui::SliderInt("Extract only LoD", &param<int>("extract_lod"), 0, 3);
    }

    void process() {
      const auto& rings = input("rings").get<LinearRingCollection&>();
      const auto& values_in = input("values").get<vec1i&>();
      typedef uint32_t N;

      TriangleCollection triangles;
      vec3f normals;
      vec1i values_out;
      size_t vi=0;
      for(const auto& poly_3d : rings){
        auto normal = calculate_normal(poly_3d);
        vec2f poly_2d = project_ring_2d(poly_3d, normal);
        vec3f vertices;
        std::vector<N> indices = mapbox::earcut<N>(std::vector<vec2f>({poly_2d}));
        for(auto i : indices) {
          vertices.push_back({poly_3d[i]});
        }
        for (size_t i=0; i<indices.size()/3; ++i) {
          Triangle triangle;
          for (size_t j=0; j<3; ++j) {
            triangle[j] = {vertices[i*3+j][0], vertices[i*3+j][1], vertices[i*3+j][2]};
            normals.push_back({normal.x, normal.y, normal.z});
            values_out.push_back(values_in[vi]);
          }
          triangles.push_back(triangle);
        }
        vi++;
      }

      // set outputs
      output("triangles").set(triangles);
      output("normals").set(normals);
      output("values").set(values_out);
    }
  };

  // Create a NodeRegister, ie a list of all available nodes
  NodeRegisterHandle create_register() {
    auto R = NodeRegister::create("Utility");
    R->register_node<RingTriangulatorNode>("RingTriangulator");
    return R;
  }
}