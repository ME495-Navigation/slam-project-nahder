
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <string>

namespace turtlelib
{

    class SVG
    {

    public:
        SVG(); // in the constructor add the header to the svg file

        void addPoint(Point2D p,
                      std::string color = "black", float strokeWidth = 1.0, float radius = 3.0);

        void addVector(Point2D head, Point2D tail,
                       std::string color = "black", float strokeWidth = 5.0);

        // will just call addVector twice, once for each vector
        void addCoordinateFrame(Point2D p1, Point2D p2, Point2D p3, Point2D p4,
                                std::string color = "black", float strokeWidth = 5.0);

        // use the transform but then invert the y axis
        Point2D cvt_world_to_svg(const Transform2D &tf, Point2D p);

        void close_SVG();

        void write_to_file(const::std::string &filename);

    private:
        // The location of a point can be determined by a Transform2D relative to the midpoint of the page
        // The midpoint of the page is the origin of the SVG coordinate system. Store the midpoint
        // as a Point2D object.
        std::string svgContent; 
        float vb_min_x{0.0}, vb_min_y{0.0}, vb_width{816.0}, vb_height{1056.0}; // viewbox parameters

        const Transform2D tf_world_to_svg {
            Vector2D{ vb_width / 2,
                        -vb_height / 2 }, 0.0 };
    };
}