
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <string>

namespace turtlelib
{

    class SVG
    {

    public:
        SVG(); 

        void addPoint(Point2D p,
                      std::string color = "black", float strokeWidth = 5.0, float radius = 3.0);

        void addVector(Point2D head, Point2D tail,
                      std::string color = "black", float strokeWidth = 5.0);

        void addCoordinateFrame(Point2D p1, Point2D p2, Point2D p3, Point2D p4,
                                std::string color1 = "green", std::string color2 = "blue",
                                 float strokeWidth = 5.0);

        Point2D cvt_svg_to_world(const Transform2D &tf, Point2D p);

        Point2D scale_vb_to_world(Point2D p);

        void close_SVG();

        void write_to_file(const::std::string &filename);

    private:
        std::string svgContent; 

        float vb_min_x{0.0}, vb_min_y{0.0}, vb_width{816.0}, vb_height{1056.0}; // viewbox parameters

        const Transform2D tf_svg_to_world {
            Vector2D{vb_width / 2, vb_height / 2}, 0.0
        };
    };
}