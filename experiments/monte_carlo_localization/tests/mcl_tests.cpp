#include "mcl.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kTolerance = 1e-9;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(
            message + " (actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected) + ")");
    }
}

void testAngleConversionsAndNormalization() {
    requireNear(mcl::degreesToRadians(180.0), mcl::kPi, kTolerance, "180 degrees converts to pi");
    requireNear(mcl::radiansToDegrees(mcl::kPi), 180.0, kTolerance, "pi converts to 180 degrees");
    requireNear(mcl::normalizeAngle(0.0), 0.0, kTolerance, "zero remains zero");
    requireNear(mcl::normalizeAngle(2.0 * mcl::kPi), 0.0, kTolerance, "2pi wraps to zero");
    requireNear(mcl::normalizeAngle(-mcl::kPi), -mcl::kPi, kTolerance, "-pi is canonical");
    requireNear(mcl::normalizeAngle(mcl::kPi), -mcl::kPi, kTolerance, "pi wraps to canonical -pi");
    requireNear(
        mcl::normalizeAnglePositive(-0.25),
        2.0 * mcl::kPi - 0.25,
        kTolerance,
        "positive normalization wraps negative angles");
}

void testRayToWallGeometry() {
    const mcl::FieldBounds field{};
    requireNear(*mcl::distanceToWall(0.0, 0.0, 0.0, field), 72.0, kTolerance, "center faces +x");
    requireNear(*mcl::distanceToWall(0.0, 0.0, mcl::kPi, field), 72.0, kTolerance, "center faces -x");
    requireNear(*mcl::distanceToWall(0.0, 0.0, mcl::kPi / 2.0, field), 72.0, kTolerance, "center faces +y");
    requireNear(
        *mcl::distanceToWall(0.0, 0.0, mcl::kPi / 4.0, field),
        72.0 * std::sqrt(2.0),
        kTolerance,
        "center faces a corner");
    requireNear(*mcl::distanceToWall(20.0, -10.0, 0.0, field), 52.0, kTolerance, "off-center +x range");
    requireNear(
        *mcl::distanceToWall(20.0, -10.0, mcl::kPi / 2.0, field),
        82.0,
        kTolerance,
        "off-center +y range");
    requireNear(
        *mcl::distanceToWall(72.0, 0.0, mcl::kPi, field),
        144.0,
        kTolerance,
        "a ray from the wall points to the opposite wall when facing inward");
    requireNear(
        *mcl::distanceToWall(72.0, 0.0, 0.0, field),
        0.0,
        kTolerance,
        "a ray facing outward from the wall has zero range");
    require(!mcl::distanceToWall(73.0, 0.0, 0.0, field), "outside poses have no wall intersection");
}

void testWeightNormalization() {
    std::vector<mcl::Particle> particles{
        {0.0, 0.0, 0.0, 2.0},
        {0.0, 0.0, 0.0, 3.0},
        {0.0, 0.0, 0.0, 5.0},
    };

    mcl::normalizeWeights(particles);
    requireNear(particles[0].weight, 0.2, kTolerance, "first weight normalizes");
    requireNear(particles[1].weight, 0.3, kTolerance, "second weight normalizes");
    requireNear(particles[2].weight, 0.5, kTolerance, "third weight normalizes");
    requireNear(
        particles[0].weight + particles[1].weight + particles[2].weight,
        1.0,
        kTolerance,
        "normalized weights sum to one");
}

void testSystematicResampling() {
    constexpr std::size_t kParticleCount = 100;
    std::vector<mcl::Particle> particles;
    particles.reserve(kParticleCount);
    particles.push_back({1.0, 0.0, 0.0, 0.91});
    for (std::size_t index = 1; index < kParticleCount; ++index) {
        particles.push_back({static_cast<double>(index), 0.0, 0.0, 0.09 / 99.0});
    }

    std::mt19937 rng(82855);
    const auto resampled = mcl::systematicResample(particles, rng);
    require(resampled.size() == kParticleCount, "resampling preserves particle count");

    std::size_t high_weight_particle_count = 0;
    for (const auto& particle : resampled) {
        if (particle.x == 1.0) ++high_weight_particle_count;
        requireNear(
            particle.weight,
            1.0 / static_cast<double>(kParticleCount),
            kTolerance,
            "resampled particles receive uniform weights");
    }
    require(high_weight_particle_count > 50, "systematic resampling favors high-weight particles");
}

void testCircularHeadingMean() {
    const double one_degree = mcl::degreesToRadians(1.0);
    const double three_hundred_fifty_nine_degrees = mcl::degreesToRadians(359.0);
    const std::vector<mcl::Particle> particles{
        {0.0, 0.0, three_hundred_fifty_nine_degrees, 0.5},
        {0.0, 0.0, one_degree, 0.5},
    };

    const mcl::Pose estimate = mcl::estimatePose(particles);
    require(
        std::abs(mcl::normalizeAngle(estimate.theta)) < mcl::degreesToRadians(0.1),
        "circular mean stays near zero across the angle wrap");
}

void testDeterministicMotionUpdate() {
    mcl::MclConfig config;
    config.particle_count = 1;
    config.odometry_position_noise_inches = 0.0;
    config.odometry_heading_noise_radians = 0.0;

    std::vector<mcl::Particle> particles{{1.0, 2.0, mcl::kPi / 2.0, 1.0}};
    std::mt19937 rng(1);
    mcl::motionUpdate(
        particles,
        {3.0, 1.0, 0.1},
        config,
        rng);

    requireNear(particles[0].x, 0.0, kTolerance, "forward/sideways motion transforms x");
    requireNear(particles[0].y, 5.0, kTolerance, "forward/sideways motion transforms y");
    requireNear(
        particles[0].theta,
        mcl::kPi / 2.0 + 0.1,
        kTolerance,
        "heading motion is applied");
}

void testMeasurementModel() {
    mcl::MclConfig config;
    config.particle_count = 2;
    config.odometry_position_noise_inches = 0.0;
    config.odometry_heading_noise_radians = 0.0;
    const mcl::RangeMeasurement measurement{72.0, 72.0, 72.0};
    std::vector<mcl::Particle> particles{
        {0.0, 0.0, 0.0, 0.5},
        {30.0, 0.0, 0.0, 0.5},
    };

    mcl::measurementUpdate(particles, measurement, config);
    require(particles[0].weight > particles[1].weight, "true-pose particle receives higher likelihood");
    requireNear(
        particles[0].weight + particles[1].weight,
        1.0,
        kTolerance,
        "measurement update normalizes weights");
}

void testParticleInitialization() {
    mcl::MclConfig config;
    config.particle_count = 64;
    std::mt19937 rng(82855);
    const auto particles = mcl::initializeParticles(config, rng);
    require(particles.size() == config.particle_count, "initialization creates requested particle count");
    for (const auto& particle : particles) {
        require(mcl::isInsideField(particle.x, particle.y, config.field), "initial particles are in field");
        require(particle.theta >= 0.0 && particle.theta <= 2.0 * mcl::kPi, "initial headings are bounded");
        requireNear(
            particle.weight,
            1.0 / static_cast<double>(config.particle_count),
            kTolerance,
            "initial weights are uniform");
    }
}

} // namespace

int main() {
    const std::vector<std::pair<const char*, void (*)()>> tests{
        {"angle conversions and normalization", testAngleConversionsAndNormalization},
        {"ray-to-wall geometry", testRayToWallGeometry},
        {"weight normalization", testWeightNormalization},
        {"systematic resampling", testSystematicResampling},
        {"circular heading mean", testCircularHeadingMean},
        {"deterministic motion update", testDeterministicMotionUpdate},
        {"measurement model", testMeasurementModel},
        {"particle initialization", testParticleInitialization},
    };

    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception& error) {
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
            return EXIT_FAILURE;
        }
    }

    std::cout << tests.size() << " MCL tests passed\n";
    return EXIT_SUCCESS;
}
