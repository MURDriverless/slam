#include <Eigen/Dense>
#include <string>
/*
    This class implements points using templated Eigen Matrix functions 
    To represent points in an XYZ format.
    May be extended to have associated colours.
*/

  template<typename T>
  class Point
  {
  public:
    Point()
    {}

    Point(T x_in, T y_in, T z_in, std::string colour_in)
      {
        x = x_in;
        y = y_in;
        z = z_in;
        colour = colour_in; 
      }
    Point(T x_in, T y_in, T z_in)
      {
        x = x_in;
        y = y_in;
        z = z_in;
      }
    T x; 
    T y; 
    T z;
    std::string colour;

  };
