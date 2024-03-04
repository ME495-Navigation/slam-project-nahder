#include "turtlelib/diff_drive.hpp"
#include <armadillo>

namespace turtlelib
{
constexpr int n = 20;

class EKF
{

public:
  EKF();

  arma::vec get_state();

  void predict(Twist2D u);

  void update(double obs_x, double obs_y, int j);

private:
  arma::mat construct_measurement_jacobian(const Vector2D & diff, double dist, int j);

  arma::vec state, predicted_state;
  arma::mat cov, predicted_cov;

  Twist2D prev_twist;
  arma::mat H, K, R, Q;

  std::vector<bool> obstacle_seen;


};
}
