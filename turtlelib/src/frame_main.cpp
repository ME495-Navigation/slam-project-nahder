#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include <ostream>
#include <iostream>


int main(void)
{
  turtlelib::Transform2D T_a, T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;
  turtlelib::Point2D p_a, p_b, p_c;
  turtlelib::Vector2D v_a, v_b, v_c;
  turtlelib::Twist2D V_a, V_b, V_c;

  std::cout << "Enter transform T_{a,b}:" << std::endl;
  std::cin >> T_ab;

  std::cout << "Enter transform T_{b,c}:" << std::endl;
  std::cin >> T_bc;

  T_ba = T_ab.inv();
  T_cb = T_bc.inv();
  T_ac = T_ab * T_bc;
  T_ca = T_ac.inv();

  std::cout << "T_{a,b} = " << T_ab << std::endl;
  std::cout << "T_{b,a} = " << T_ba << std::endl;
  std::cout << "T_{b,c} = " << T_bc << std::endl;
  std::cout << "T_{c,b} = " << T_cb << std::endl;
  std::cout << "T_{a,c} = " << T_ac << std::endl;
  std::cout << "T_{c,a} = " << T_ca << std::endl;

  //draw coordinate frames
  turtlelib::SVG svg;
  svg.draw_xform(T_a, "a", "purple", 5.0);
  svg.draw_xform(T_ab, "b", "brown", 5.0);
  svg.draw_xform(T_ac, "c", "orange", 5.0);

  std::cout << "Enter point p_a:" << std::endl;
  std::cin >> p_a;

  p_b = T_ba(p_a);
  p_c = T_ca(p_a);

  std::cout << "p_a = " << p_a << std::endl;
  std::cout << "p_b = " << p_b << std::endl;
  std::cout << "p_c = " << p_c << std::endl;

  svg.addPoint(p_a, "purple", 3.0, 5.0);
  svg.addPoint(p_b, "brown", 3.0, 5.0);
  svg.addPoint(p_c, "orange", 3.0, 5.0);

  std::cout << "Enter vector v_b:" << std::endl;
  std::cin >> v_b;

  turtlelib::Vector2D v_bhat{turtlelib::normalize_vector(v_b)};
  std::cout << "v_bhat = " << v_bhat << std::endl;   //frame b

  v_a = T_ab(v_b);
  v_c = T_cb(v_b);

  std::cout << "v_a = " << v_a << std::endl;
  std::cout << "v_b = " << v_b << std::endl;
  std::cout << "v_c = " << v_c << std::endl;

  turtlelib::Point2D tail_b = T_ab(turtlelib::Point2D{0.0, 0.0});
  turtlelib::Point2D head_b = turtlelib::Point2D{tail_b.x + v_bhat.x, tail_b.y + v_bhat.y};
  svg.addVector(tail_b, head_b, "brown", 5.0);

  turtlelib::Point2D tail_a{0.0, 0.0};
  turtlelib::Point2D head_a{tail_a.x + v_a.x, tail_a.y + v_a.y};
  svg.addVector(tail_a, head_a, "purple", 5.0);

  turtlelib::Point2D tail_c = T_ac(turtlelib::Point2D{0.0, 0.0});
  turtlelib::Point2D head_c = turtlelib::Point2D{tail_c.x + v_c.x, tail_c.y + v_c.y};
  svg.addVector(tail_c, head_c, "orange", 5.0);

  std::cout << "Enter twist V_b:" << std::endl;
  std::cin >> V_b;

  V_a = T_ab(V_b);
  V_c = T_cb(V_b);
  std::cout << "V_a = " << V_a << std::endl;
  std::cout << "V_b = " << V_b << std::endl;
  std::cout << "V_c = " << V_c << std::endl;

  svg.write_to_file("/tmp/frames.svg");

  return 0;

}
