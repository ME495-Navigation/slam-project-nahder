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
  arma::vec state, predicted_state;
  arma::mat cov, predicted_cov;

  Twist2D prev_twist;
  arma::mat H, K, R, Q;

  std::vector<bool> obstacle_seen;
};
}
