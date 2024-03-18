#include "turtlelib/ekf.hpp"
#include <armadillo>
#include <cstdio>

namespace turtlelib
{
EKF::EKF()
{
  // combined state vectors
  state = arma::vec((2 * n + 3), arma::fill::zeros); //combined state vector (q_t, m_t)
  predicted_state = state;

  // initial covariance matrix elements
  arma::mat robot_cov(3, 3, arma::fill::zeros);

  // measurement covariances are very high (unknown)
  arma::mat obs_cov(2 * n, 2 * n, arma::fill::eye);
  obs_cov *= 9999.0;

  // off-diagonal blocks
  arma::mat zeros_1(3, 2 * n, arma::fill::zeros);
  arma::mat zeros_2(2 * n, 3, arma::fill::zeros);

  // combine the covariance matrices into one
  cov = arma::join_cols(
    arma::join_rows(robot_cov, zeros_1),
    arma::join_rows(zeros_2, obs_cov));

  predicted_cov = cov;

  prev_twist = Twist2D{0.0, 0.0, 0.0};
  obstacle_seen = std::vector<bool>(n, false);
}

arma::vec EKF::get_state()
{
  return predicted_state;
}


void EKF::predict(Twist2D u)
{
  // calculate the change in the twist
  auto ut = Twist2D{
    normalize_angle(u.omega - prev_twist.omega),
    u.x - prev_twist.x,
    u.y - prev_twist.y
  };

  // update the state

  predicted_state(0) = normalize_angle(state(0) + ut.omega);
  predicted_state(1) += ut.x * cos(state(0));
  predicted_state(2) += ut.x * sin(state(0));
  prev_twist = u;

  // calculate the A matrix
  arma::mat At(2 * n + 3, 2 * n + 3, arma::fill::eye);
  double theta = normalize_angle(state(0));

  if (almost_equal(ut.omega, 0.0)) {
    At(1, 0) = -ut.x * sin(theta);
    At(2, 0) = ut.x * cos(theta);
  } else { // non-zero angular velocity
    At(1, 0) = (-ut.x / ut.omega) * cos(theta) + (ut.x / ut.omega) * cos(
      normalize_angle(theta + ut.omega));

    At(2, 0) = (-ut.x / ut.omega) * sin(theta) + (ut.x / ut.omega) * sin(
      normalize_angle(theta + ut.omega));
  }

  // calculate the Q matrix
  arma::mat Q(3, 3, arma::fill::eye);
  Q *= 0.2; // multiply by process noise

  // create the augmented Q matrix
  arma::mat Q_bar = arma::join_cols(
    arma::join_rows(Q, arma::zeros(3, 2 * n)),
    arma::join_rows(arma::zeros(2 * n, 3), arma::zeros(2 * n, 2 * n))
  );

  // update the covariance
  predicted_cov = At * cov * At.t() + Q_bar;
}

void EKF::update(double obs_x, double obs_y, int j)
{
  // compute relative distance (r_j) and angle (phi_j) to landmark j
  double r_j = sqrt(pow(obs_x, 2) + pow(obs_y, 2));
  double phi_j = normalize_angle(atan2(obs_y, obs_x));

  // if the landmark hasn't been observed before, update its position in the state vector
  if (!obstacle_seen.at(j)) {
    obstacle_seen.at(j) = true;     // mark landmark as seen
    // innovation part: updating landmark position based on current observation
    predicted_state(3 + 2 * j) = predicted_state(1) + r_j *
      cos(normalize_angle(phi_j + predicted_state(0)));
    predicted_state(3 + 2 * j + 1) = predicted_state(2) + r_j *
      sin(normalize_angle(phi_j + predicted_state(0)));
  }

  // compute expected measurement (zbar) based on current state estimate
  Vector2D diff = {
    predicted_state(3 + 2 * j) - predicted_state(1),
    predicted_state(3 + 2 * j + 1) - predicted_state(2)
  };
  double dist = sqrt(pow(diff.x, 2) + pow(diff.y, 2));
  double phi = normalize_angle(atan2(diff.y, diff.x) - predicted_state(0));
  arma::vec zbar = {dist, phi};   // expected measurement vector

  // construct the measurement model jacobian (H matrix)
  arma::mat H = construct_measurement_jacobian(diff, dist, j);

  // compute the Kalman gain
  arma::mat R = arma::mat(2, 2, arma::fill::eye) * 0.15;   // measurement noise
  arma::mat K = predicted_cov * H.t() * arma::inv(H * predicted_cov * H.t() + R);   // Kalman gain calculation

  // compute innovation: the difference between actual and expected measurement
  arma::vec z_diff = arma::vec{r_j, phi_j} - zbar;
  z_diff(1) = normalize_angle(z_diff(1));

  // update state estimate with innovation
  predicted_state += K * z_diff;
  predicted_state(0) = normalize_angle(predicted_state(0));

  // update covariance matrix
  arma::mat I = arma::eye<arma::mat>(2 * n + 3, 2 * n + 3);
  predicted_cov = (I - K * H) * predicted_cov;

  // align internal state and covariance with the updated predictions
  state = predicted_state;
  cov = predicted_cov;
}

arma::mat EKF::construct_measurement_jacobian(const Vector2D & diff, double dist, int j)
{
  // prepare components of the H matrix
  arma::mat h1 = {
    {0.0, -diff.x / dist, -diff.y / dist},
    {-1.0, diff.y / pow(dist, 2), -diff.x / pow(dist, 2)}
  };
  arma::mat zeros_before(2, 2 * j, arma::fill::zeros);
  arma::mat h2 = {
    {diff.x / dist, diff.y / dist},
    {-diff.y / pow(dist, 2), diff.x / pow(dist, 2)}
  };
  arma::mat zeros_after(2, 2 * n - 2 * (j + 1), arma::fill::zeros);

  // construct and return the full H matrix
  return arma::join_rows(h1, zeros_before, h2, zeros_after);
}

} // namespace turtlelib
