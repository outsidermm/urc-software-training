#include "particle_filter/particle_filter.hpp"

namespace particle_filter
{

void ParticleFilter::applyMotionUpdate(
  double dx_body, double dy_body, double dyaw,
  double translation, double rotation)
{

  double sigma_x = x_noise_ * translation;
  double sigma_y = y_noise_ * translation;
  double sigma_theta = theta_noise_ * rotation;

  for (auto & particle : particles_) {
    double noisy_dx = dx_body + gaussianNoise(sigma_x);
    double noisy_dy = dy_body + gaussianNoise(sigma_y);
    double noisy_dyaw = dyaw + gaussianNoise(sigma_theta);

    // Update particle position and orientation
    particle.x += noisy_dx * std::cos(particle.theta) - noisy_dy * std::sin(particle.theta);
    particle.y += noisy_dx * std::sin(particle.theta) + noisy_dy * std::cos(particle.theta);
    particle.theta = wrapAngle(particle.theta + noisy_dyaw);
  }
}

}  // namespace particle_filter
