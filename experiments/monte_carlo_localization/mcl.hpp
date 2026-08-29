#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

namespace mcl {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kFieldHalfWidthInches = 72.0;
inline constexpr std::size_t kDefaultParticleCount = 2'000;

struct FieldBounds {
    double half_width_inches = kFieldHalfWidthInches;
};

// Angles are radians internally. The field frame uses +x/+y axes and
// counter-clockwise-positive headings, with zero pointing along +x.
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct Particle {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double weight = 0.0;
};

// Translation is expressed in the robot frame: forward is along the
// particle heading and sideways is positive to the robot's left.
struct OdometryDelta {
    double forward = 0.0;
    double sideways = 0.0;
    double heading = 0.0;
};

struct RangeMeasurement {
    double front = 0.0;
    double left = 0.0;
    double right = 0.0;
};

struct MclConfig {
    std::size_t particle_count = kDefaultParticleCount;
    FieldBounds field{};

    // These are experimental model parameters, not calibrated claims about
    // a particular VEX distance sensor.
    double odometry_position_noise_inches = 0.5;
    double odometry_heading_noise_radians = 0.02;
    double measurement_variance_per_inch = 0.05;
    double minimum_measurement_variance = 0.590551;

    // Resample only when the effective sample size falls below this fraction
    // of the particle count. Set to 1.0 to resample after every update.
    double resample_effective_sample_ratio = 0.5;
};

double degreesToRadians(double degrees);
double radiansToDegrees(double radians);

// Returns an angle in [-pi, pi). A circular representation avoids a
// discontinuity when headings cross 0/2pi.
double normalizeAngle(double radians);
double normalizeAnglePositive(double radians);

bool isInsideField(double x, double y, FieldBounds bounds = {});
bool isInsideField(const Pose& pose, FieldBounds bounds = {});

// Return the nearest intersection of a pose's heading ray with the square
// field boundary. An invalid/outside pose has no meaningful intersection.
std::optional<double> distanceToWall(
    double x,
    double y,
    double theta,
    FieldBounds bounds = {});

std::optional<RangeMeasurement> predictRanges(
    const Pose& pose,
    FieldBounds bounds = {});

double measurementStandardDeviation(
    double expected_distance,
    const MclConfig& config);

std::vector<Particle> initializeParticles(
    const MclConfig& config,
    std::mt19937& rng);

Pose applyOdometry(const Pose& pose, const OdometryDelta& delta);

void motionUpdate(
    std::vector<Particle>& particles,
    const OdometryDelta& delta,
    const MclConfig& config,
    std::mt19937& rng);

void normalizeWeights(std::vector<Particle>& particles);

void measurementUpdate(
    std::vector<Particle>& particles,
    const RangeMeasurement& measurement,
    const MclConfig& config);

double effectiveSampleSize(const std::vector<Particle>& particles);

// Systematic (low-variance) resampling preserves the particle count while
// allocating samples according to the normalized particle weights.
std::vector<Particle> systematicResample(
    const std::vector<Particle>& particles,
    std::mt19937& rng);

Pose estimatePose(const std::vector<Particle>& particles);

class MonteCarloLocalization {
public:
    explicit MonteCarloLocalization(
        MclConfig config = {},
        std::uint32_t seed = static_cast<std::uint32_t>(std::random_device{}()));

    void initialize();
    void update(const OdometryDelta& delta, const RangeMeasurement& measurement);
    void motionUpdate(const OdometryDelta& delta);
    void measurementUpdate(const RangeMeasurement& measurement);
    void resample();

    [[nodiscard]] Pose estimatePose() const;
    [[nodiscard]] double effectiveSampleSize() const;
    [[nodiscard]] const std::vector<Particle>& particles() const;
    [[nodiscard]] const MclConfig& config() const;

private:
    MclConfig config_;
    std::mt19937 rng_;
    std::vector<Particle> particles_;
};

} // namespace mcl
