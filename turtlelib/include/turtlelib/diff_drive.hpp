

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib { 

    //robot configuration
    struct robotConfig { 
        double x{0.0};
        double y{0.0};
        double theta{0.0};
    };

    //wheel velocities
    //the wheel velocities are in terms of unit 1 unit time
    struct wheelVel {
        double left_wheel_vel{0.0};
        double right_wheel_vel{0.0};
    };

    class DiffDrive {

    public:
        DiffDrive(double wheel_radius, double track_width);
        //fk: determine the POSITION of the robot based on the known wheel motion
        //update the current configuration
        void forwardKinematics(const wheelVel new_wheel_config);

        //ik: compute the wheel velocities required to achieve a desired twist
        wheelVel inverseKinematics(const Twist2D twist) const;

        //convert wheel velocities to actual body twis
        Twist2D computeBodyTwist(const wheelVel u) const;

        robotConfig get_config() const { return cur_config; }

    private:
        robotConfig cur_config;
        wheelVel cur_wheel_vel; 

        double wheel_radius{0.033}; //defaults for turtlebot3 burger
        double track_width{0.16};
    };

}