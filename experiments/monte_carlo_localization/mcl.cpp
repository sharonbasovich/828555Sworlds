#include "mcl.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mcl {
namespace {

constexpr double kGeometryEpsilon = 1e-9;
constexpr double kTwoPi = 2.0 * kPi;
const double kGaussianNormalization = 0.5 * std::log(2.0 * kPi);

double clampToField(double value, const FieldBounds& bounds) {
    return std::clamp(value, -bounds.half_width_inches, bounds.half_width_inches);
}

bool isValidMeasurement(const RangeMeasurement& measurement) {
    return std::isfinite(measurement.front) && measurement.front >= 0.0 &&
           std::isfinite(measurement.left) && measurement.left >= 0.0 &&
           std::isfinite(measurement.right) && measurement.right >= 0.0;
}

double logGaussianLikelihood(
    double observed,
    double expected,
    double standard_deviation) {
    if (!std::isfinite(observed) || !std::isfinite(expected) ||
        !std::isfinite(standard_deviation) || standard_deviation <= 0.0) {
        return -std::numeric_limits<double>::infinity();
    }

    const double standardized_error = (observed - expected) / standard_deviation;
    return -0.5 * standardized_error * standardized_error -
           std::log(standard_deviation) - kGaussianNormalization;
}

} // namespace

double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / kPi;
}

double normalizeAngle(double radians) {
    if (!std::isfinite(radians)) return 0.0;

    double normalized = std::fmod(radians + kPi, kTwoPi);
    if (normalized < 0.0) normalized += kTwoPi;
    return normalized - kPi;
}

double normalizeAnglePositive(double radians) {
    if (!std::isfinite(radians)) return 0.0;

    double normalized = std::fmod(radians, kTwoPi);
    if (normalized < 0.0) normalized += kTwoPi;
    return normalized;
}

bool isInsideField(double x, double y, FieldBounds bounds) {
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(bounds.half_width_inches) ||
        bounds.half_width_inches <= 0.0) {
        return false;
    }

    return x >= -bounds.half_width_inches - kGeometryEpsilon &&
           x <= bounds.half_width_inches + kGeometryEpsilon &&
           y >= -bounds.half_width_inches - kGeometryEpsilon &&
           y <= bounds.half_width_inches + kGeometryEpsilon;
}

bool isInsideField(const Pose& pose, FieldBounds bounds) {
    return isInsideField(pose.x, pose.y, bounds);
}

std::optional<double> distanceToWall(
    double x,
    double y,
    double theta,
    FieldBounds bounds) {
    if (!isInsideField(x, y, bounds) || !std::isfinite(theta)) {
        return std::nullopt;
    }

    const double half_width = bounds.half_width_inches;
    const double direction_x = std::cos(theta);
    const double direction_y = std::sin(theta);
    double nearest_distance = std::numeric_limits<double>::infinity();

    const auto considerIntersection = [&](double distance) {
        if (distance < -kGeometryEpsilon) return;

        const double positive_distance = std::max(0.0, distance);
        const double intersection_x = x + positive_distance * direction_x;
        const double intersection_y = y + positive_distance * direction_y;

        if (intersection_x < -half_width - kGeometryEpsilon ||
            intersection_x > half_width + kGeometryEpsilon ||
            intersection_y < -half_width - kGeometryEpsilon ||
            intersection_y > half_width + kGeometryEpsilon) {
            return;
        }

        nearest_distance = std::min(nearest_distance, positive_distance);
    };

    // Only the boundary in the ray's direction can be the forward exit for
    // each axis. This also avoids treating a pose on a wall as intersecting
    // that same wall at t=0 when the ray points back into the field.
    if (direction_x > kGeometryEpsilon) {
        considerIntersection((half_width - x) / direction_x);
    } else if (direction_x < -kGeometryEpsilon) {
        considerIntersection((-half_width - x) / direction_x);
    }

    if (direction_y > kGeometryEpsilon) {
        considerIntersection((half_width - y) / direction_y);
    } else if (direction_y < -kGeometryEpsilon) {
        considerIntersection((-half_width - y) / direction_y);
    }

    if (!std::isfinite(nearest_distance)) return std::nullopt;
    return nearest_distance;
}

std::optional<RangeMeasurement> predictRanges(
    const Pose& pose,
    FieldBounds bounds) {
    if (!isInsideField(pose, bounds)) return std::nullopt;

    // Sensor orientations are explicit: left/right are +/-90 degrees from
    // the forward-facing sensor in the field frame.
    const auto front = distanceToWall(pose.x, pose.y, pose.theta, bounds);
    const auto left = distanceToWall(pose.x, pose.y, pose.theta + kPi / 2.0, bounds);
    const auto right = distanceToWall(pose.x, pose.y, pose.theta - kPi / 2.0, bounds);

    if (!front || !left || !right) return std::nullopt;
    return RangeMeasurement{*front, *left, *right};
}

double measurementStandardDeviation(
    double expected_distance,
    const MclConfig& config) {
    const double variance = std::max(
        expected_distance * config.measurement_variance_per_inch,
        config.minimum_measurement_variance);
    return std::sqrt(std::max(variance, kGeometryEpsilon));
}

std::vector<Particle> initializeParticles(
    const MclConfig& config,
    std::mt19937& rng) {
    std::vector<Particle> particles;
    particles.reserve(config.particle_count);
    if (config.particle_count == 0 || config.field.half_width_inches <= 0.0) {
        return particles;
    }

    std::uniform_real_distribution<double> position_distribution(
        -config.field.half_width_inches,
        config.field.half_width_inches);
    std::uniform_real_distribution<double> heading_distribution(0.0, kTwoPi);
    const double uniform_weight = 1.0 / static_cast<double>(config.particle_count);

    for (std::size_t index = 0; index < config.particle_count; ++index) {
        particles.push_back({
            position_distribution(rng),
            position_distribution(rng),
            heading_distribution(rng),
            uniform_weight,
        });
    }

    return particles;
}

Pose applyOdometry(const Pose& pose, const OdometryDelta& delta) {
    const double cosine = std::cos(pose.theta);
    const double sine = std::sin(pose.theta);

    return {
        pose.x + delta.forward * cosine - delta.sideways * sine,
        pose.y + delta.forward * sine + delta.sideways * cosine,
        normalizeAngle(pose.theta + delta.heading),
    };
}

void motionUpdate(
    std::vector<Particle>& particles,
    const OdometryDelta& delta,
    const MclConfig& config,
    std::mt19937& rng) {
    std::normal_distribution<double> position_noise(
        0.0,
        std::max(0.0, config.odometry_position_noise_inches));
    std::normal_distribution<double> heading_noise(
        0.0,
        std::max(0.0, config.odometry_heading_noise_radians));

    for (auto& particle : particles) {
        const Pose updated = applyOdometry(
            {particle.x, particle.y, particle.theta},
            {
                delta.forward + position_noise(rng),
                delta.sideways + position_noise(rng),
                delta.heading + heading_noise(rng),
            });

        // Clamping keeps hypotheses inside the legal field. This is simple
        // and conservative for a bounded-field experiment, while avoiding
        // impossible particles influencing the measurement update.
        particle.x = clampToField(updated.x, config.field);
        particle.y = clampToField(updated.y, config.field);
        particle.theta = updated.theta;
    }
}

void normalizeWeights(std::vector<Particle>& particles) {
    if (particles.empty()) return;

    double total_weight = 0.0;
    for (auto& particle : particles) {
        if (!std::isfinite(particle.weight) || particle.weight < 0.0) {
            particle.weight = 0.0;
        }
        total_weight += particle.weight;
    }

    if (!std::isfinite(total_weight) || total_weight <= 0.0) {
        const double uniform_weight = 1.0 / static_cast<double>(particles.size());
        for (auto& particle : particles) particle.weight = uniform_weight;
        return;
    }

    for (auto& particle : particles) particle.weight /= total_weight;
}

void measurementUpdate(
    std::vector<Particle>& particles,
    const RangeMeasurement& measurement,
    const MclConfig& config) {
    if (particles.empty()) return;

    const bool valid_measurement = isValidMeasurement(measurement);
    std::vector<double> log_weights(
        particles.size(),
        -std::numeric_limits<double>::infinity());
    double maximum_log_weight = -std::numeric_limits<double>::infinity();

    for (std::size_t index = 0; index < particles.size(); ++index) {
        const auto expected = predictRanges(
            {particles[index].x, particles[index].y, particles[index].theta},
            config.field);
        if (!valid_measurement || !expected) continue;

        const double front_sd = measurementStandardDeviation(expected->front, config);
        const double left_sd = measurementStandardDeviation(expected->left, config);
        const double right_sd = measurementStandardDeviation(expected->right, config);
        const double prior_weight = particles[index].weight;
        if (!std::isfinite(prior_weight) || prior_weight <= 0.0) continue;

        const double log_weight = std::log(prior_weight) +
            logGaussianLikelihood(measurement.front, expected->front, front_sd) +
            logGaussianLikelihood(measurement.left, expected->left, left_sd) +
            logGaussianLikelihood(measurement.right, expected->right, right_sd);

        if (std::isfinite(log_weight)) {
            log_weights[index] = log_weight;
            maximum_log_weight = std::max(maximum_log_weight, log_weight);
        }
    }

    if (!std::isfinite(maximum_log_weight)) {
        for (auto& particle : particles) particle.weight = 0.0;
        normalizeWeights(particles);
        return;
    }

    for (std::size_t index = 0; index < particles.size(); ++index) {
        particles[index].weight = std::isfinite(log_weights[index])
                                      ? std::exp(log_weights[index] - maximum_log_weight)
                                      : 0.0;
    }
    normalizeWeights(particles);
}

double effectiveSampleSize(const std::vector<Particle>& particles) {
    if (particles.empty()) return 0.0;

    double total_weight = 0.0;
    double squared_weight_sum = 0.0;
    for (const auto& particle : particles) {
        if (!std::isfinite(particle.weight) || particle.weight <= 0.0) continue;
        total_weight += particle.weight;
        squared_weight_sum += particle.weight * particle.weight;
    }

    if (total_weight <= 0.0 || squared_weight_sum <= 0.0) return 0.0;
    return (total_weight * total_weight) / squared_weight_sum;
}

std::vector<Particle> systematicResample(
    const std::vector<Particle>& particles,
    std::mt19937& rng) {
    if (particles.empty()) return {};

    double total_weight = 0.0;
    for (const auto& particle : particles) {
        if (std::isfinite(particle.weight) && particle.weight > 0.0) {
            total_weight += particle.weight;
        }
    }

    std::vector<Particle> resampled;
    resampled.reserve(particles.size());
    const double uniform_weight = 1.0 / static_cast<double>(particles.size());

    if (!std::isfinite(total_weight) || total_weight <= 0.0) {
        for (const auto& particle : particles) {
            auto copy = particle;
            copy.weight = uniform_weight;
            resampled.push_back(copy);
        }
        return resampled;
    }

    const double step = 1.0 / static_cast<double>(particles.size());
    std::uniform_real_distribution<double> offset_distribution(0.0, step);
    double target = offset_distribution(rng);
    std::size_t particle_index = 0;
    double cumulative_weight =
        (std::isfinite(particles[0].weight) && particles[0].weight > 0.0)
            ? particles[0].weight / total_weight
            : 0.0;

    for (std::size_t sample = 0; sample < particles.size(); ++sample) {
        while (target >= cumulative_weight && particle_index + 1 < particles.size()) {
            ++particle_index;
            if (std::isfinite(particles[particle_index].weight) &&
                particles[particle_index].weight > 0.0) {
                cumulative_weight += particles[particle_index].weight / total_weight;
            }
        }

        auto copy = particles[particle_index];
        copy.weight = uniform_weight;
        resampled.push_back(copy);
        target += step;
    }

    return resampled;
}

Pose estimatePose(const std::vector<Particle>& particles) {
    if (particles.empty()) return {};

    double total_weight = 0.0;
    for (const auto& particle : particles) {
        if (std::isfinite(particle.weight) && particle.weight > 0.0) {
            total_weight += particle.weight;
        }
    }

    const bool use_uniform_weights = !std::isfinite(total_weight) || total_weight <= 0.0;
    const double fallback_weight = 1.0 / static_cast<double>(particles.size());
    double x_sum = 0.0;
    double y_sum = 0.0;
    double sine_sum = 0.0;
    double cosine_sum = 0.0;

    for (const auto& particle : particles) {
        const double weight = use_uniform_weights
                                  ? fallback_weight
                                  : std::max(0.0, particle.weight) / total_weight;
        x_sum += particle.x * weight;
        y_sum += particle.y * weight;
        sine_sum += std::sin(particle.theta) * weight;
        cosine_sum += std::cos(particle.theta) * weight;
    }

    return {
        x_sum,
        y_sum,
        normalizeAngle(std::atan2(sine_sum, cosine_sum)),
    };
}

MonteCarloLocalization::MonteCarloLocalization(
    MclConfig config,
    std::uint32_t seed)
    : config_(config), rng_(seed) {}

void MonteCarloLocalization::initialize() {
    particles_ = initializeParticles(config_, rng_);
}

void MonteCarloLocalization::update(
    const OdometryDelta& delta,
    const RangeMeasurement& measurement) {
    motionUpdate(delta);
    measurementUpdate(measurement);

    const double particle_count = static_cast<double>(particles_.size());
    const double threshold =
        config_.resample_effective_sample_ratio * particle_count;
    if (particle_count > 0.0 && config_.resample_effective_sample_ratio > 0.0 &&
        effectiveSampleSize() <= threshold) {
        resample();
    }
}

void MonteCarloLocalization::motionUpdate(const OdometryDelta& delta) {
    mcl::motionUpdate(particles_, delta, config_, rng_);
}

void MonteCarloLocalization::measurementUpdate(
    const RangeMeasurement& measurement) {
    mcl::measurementUpdate(particles_, measurement, config_);
}

void MonteCarloLocalization::resample() {
    particles_ = systematicResample(particles_, rng_);
}

Pose MonteCarloLocalization::estimatePose() const {
    return mcl::estimatePose(particles_);
}

double MonteCarloLocalization::effectiveSampleSize() const {
    return mcl::effectiveSampleSize(particles_);
}

const std::vector<Particle>& MonteCarloLocalization::particles() const {
    return particles_;
}

const MclConfig& MonteCarloLocalization::config() const {
    return config_;
}

} // namespace mcl
