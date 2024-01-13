#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include <ostream>
#include <iostream>


int main(void){

    turtlelib::Transform2D T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;
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

    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> p_a;

    p_b = T_ba(p_a);
    p_c = T_ca(p_a);

    std::cout << "p_a = " << p_a << std::endl;
    std::cout << "p_b = " << p_b << std::endl;
    std::cout << "p_c = " << p_c << std::endl;

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;
    
    turtlelib::Vector2D v_bhat{turtlelib::normalize_vector(v_b)};
    std::cout << "v_bhat = " << v_bhat << std::endl;

    v_a = T_ab(v_b);
    v_c = T_cb(v_b);

    std::cout << "v_a = " << v_a << std::endl;
    std::cout << "v_b = " << v_b << std::endl; 
    std::cout << "v_c = " << v_c << std::endl;

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> V_b;

    V_a = T_ab(V_b);
    V_c = T_cb(V_b);
    std::cout << "V_a = " << V_a << std::endl;
    std::cout << "V_b = " << V_b << std::endl;
    std::cout << "V_c = " << V_c << std::endl;



    return 0;
    
}