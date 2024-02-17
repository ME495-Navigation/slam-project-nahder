#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <string>

namespace turtlelib
{
/// @brief class to create an SVG file with points, vectors, and coordinate frames
class SVG
{

public:
  SVG();
  /// @brief adds a point to the SVG
  /// @param p
  /// @param color
  /// @param strokeWidth
  /// @param radius
  void addPoint(
    Point2D p,
    std::string color = "black", float strokeWidth = 5.0, float radius = 3.0);

  /// @brief adds a vector to the SVG
  /// @param tail
  /// @param head
  /// @param color
  /// @param strokeWidth
  void addVector(
    Point2D tail, Point2D head,
    std::string color = "black", float strokeWidth = 5.0);

  /// @brief adds a coordinate frame to the SVG
  /// @param p1
  /// @param p2
  /// @param p3
  /// @param p4
  /// @param label
  /// @param color1
  /// @param color2
  /// @param strokeWidth
  void addCoordinateFrame(
    Point2D p1, Point2D p2, Point2D p3, Point2D p4, std::string label,
    std::string color1 = "green", std::string color2 = "blue",
    float strokeWidth = 5.0);

  /// @brief converts a point from SVG coordinates to world coordinates
  /// @param tf
  /// @param p
  /// @return converted Point2D object
  Point2D cvt_svg_to_world(const Transform2D & tf, Point2D p);

  /// @brief scales the point p from pixels to inches
  /// @param p
  /// @return converted Point2D object
  Point2D scale_vb_to_world(Point2D p);

  /// @brief draws a coordinate frame in the SVG
  /// @param tf
  /// @param label
  /// @param color
  /// @param strokeWidth
  void draw_xform(
    Transform2D tf, std::string label,
    std::string color, float strokeWidth);

  /// @brief closes the SVG file
  void close_SVG();

  /// @brief writes the SVG content to a file
  void write_to_file(const ::std::string & filename);

private:
  std::string svgContent;

  float vb_min_x{0.0}, vb_min_y{0.0}, vb_width{816.0}, vb_height{1056.0};

  const Transform2D tf_svg_to_world {
    Vector2D{vb_width / 2, vb_height / 2}, 0.0
  };
};
}
