#ifndef MY_Line_SHAPE_H
#define MY_Line_SHAPE_H
#include <CGAL/Shape_detection_3.h>
#include <CGAL/number_utils.h>
/*
My_Line derives from Shape_base. The plane is represented by
its normal vector and distance to the origin.
*/
template <class Traits>
class My_Line : public CGAL::Shape_detection_3::Shape_base<Traits> {
public:
  typedef typename Traits::FT FT;
  typedef typename Traits::Point_3 Point;
  typedef typename Traits::Vector_3 Vector;
public:
  My_Line()
    : CGAL::Shape_detection_3::Shape_base<Traits>()
  {}
  //  Computes squared Euclidean distance from query point to the shape.
  virtual FT squared_distance(const Point &p) const {
    const Vector a = this->constr_vec(m_point_on_primitive, p);
    return this->sqlen(a - (a*m_normal)*m_normal);
  }
  FT projection(const Point &p) const {
    const Vector a = this->constr_vec(m_point_on_primitive, p);
    return (a*m_normal);
  }
  Point point_from_projection(FT pl) const {
    return (m_point_on_primitive + pl*m_normal);
  }
  Vector line_normal() const {
    return m_normal;
  }
  Point support() const {
    return m_point_on_primitive;
  }

  // Returns a string with shape parameters.
  virtual std::string info() const {
    std::stringstream sstr;
    sstr << "Type: line;" 
      // <<"n = " << this->get_x(m_normal) << ", " 
      // << this->get_y(m_normal) << ", " << this->get_z(m_normal) << ", support = "
      // << this->get_x(m_point_on_primitive) << ", "  << this->get_y(m_point_on_primitive) << ", " << this->get_z(m_point_on_primitive)
      << " #Pts: " << this->m_indices.size();
    return sstr.str();
  }
protected:
  // Constructs shape based on minimal set of samples from the input data.    
  virtual void create_shape(const std::vector<std::size_t> &indices) {
    const Point p1 = this->point(indices[0]);
    const Point p2 = this->point(indices[1]);
    m_normal = p2 - p1;
    m_normal = m_normal * (1.0 / sqrt(this->sqlen(m_normal)));
    m_point_on_primitive = p1;
    this->m_is_valid = true;
  }
  // Computes squared Euclidean distance from a set of points.
  virtual void squared_distance(const std::vector<std::size_t> &indices,
                                std::vector<FT> &dists) const {
      for (std::size_t i = 0; i < indices.size(); i++) {
        const Vector a = this->constr_vec(m_point_on_primitive, this->point(indices[i]));
        dists[i] = this->sqlen(a - (a*m_normal)*m_normal);
      }
  }
  /*
  Computes the normal deviation between shape and
  a set of points with normals.
  */
  virtual void cos_to_normal(const std::vector<std::size_t> &indices,
                             std::vector<FT> &angles) const {
      for (std::size_t i = 0; i < indices.size(); i++)
        angles[i] = 1-CGAL::abs(this->normal(indices[i]) * m_normal); //normal of point should be perpendicular to line direction
  }
  // Returns the number of required samples for construction.
  virtual std::size_t minimum_sample_size() const {
    return 2;
  }
private:
  Point m_point_on_primitive;
  Vector m_normal; // vector of unit length pointing in the direction of this line
};
#endif