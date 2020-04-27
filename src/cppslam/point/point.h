#include <Eigen/Dense>
/*
    This class implements points using templated Eigen Matrix functions 
    To represent points in an XYZ format.
    May be extended to have associated colours.
*/

  template<typename T>
  class Point : public Eigen::Matrix<T,3,1>
  {
    typedef Eigen::Matrix<T,3,1> BaseClass;
  
  public:
    Point () 
      : BaseClass()
    {}
    // 3-element constructor, delegates to base class constructor
    Point(const T& x, const T& y, const T& z)
      : BaseClass(x, y, z)
    {}
  
    // Copy constructor from any Eigen matrix type
    template<typename OtherDerived>
    Point(const Eigen::MatrixBase<OtherDerived>& other)
      : BaseClass(other)
    {}
  
    // Reuse assignment operators from base class
    using BaseClass::operator=;
  };
