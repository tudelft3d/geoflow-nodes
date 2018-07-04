
  class Index_map
  {
    boost::shared_ptr<std::vector<int> > m_indices;
    // const std::vector<int> m_indices;
  public:
    typedef int value_type; ///< Index of the shape (-1 if the point is not assigned to any shape).
    typedef int reference;
    typedef boost::readable_property_map_tag category;

    Index_map(){}

    /*!
      Constructs a property map to map points to their associated shape.
      \note `shapes` must be a range of shapes detected using `points`.
      \tparam ShapeRange an `Iterator_range` with a bidirectional
      constant iterator type with value type
      `boost::shared_ptr<CGAL::Shape_detection_3::Shape_base<Traits> >`.
     */
    Index_map (const std::vector<int> intvector)
      : m_indices (new std::vector<int>(intvector))
    {
    }

    inline friend int get (const Index_map& pm, const size_t& k)
    {
      return (*(pm.m_indices))[k];
    }

  };