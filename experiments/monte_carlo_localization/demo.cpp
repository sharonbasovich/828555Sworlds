#include "mcl.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

namespace {

struct DemoOptions {
    std::string output_path = "mcl_trace.csv";
    std::size_t iterations = 30;
    // Fixed default keeps the checked-in synthetic trace reproducible.
    std::uint32_t seed = 82'858;
};

void printUsage(const char* program) {
    std::cout << "Usage: " << program
              << " [--output path] [--iterations count] [--seed value]\n";
}

std::size_t parseSize(const std::string& value, const char* option) {
    try {
        const auto parsed = std::stoull(value);
        if (parsed == 0) throw std::invalid_argument("zero");
        return static_cast<std::size_t>(parsed);
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string(option) + " requires a positive integer");
    }
}

std::uint32_t parseSeed(const std::string& value) {
    try {
        return static_cast<std::uint32_t>(std::stoul(value));
    } catch (const std::exception&) {
        throw std::invalid_argument("--seed requires an unsigned integer");
    }
}

DemoOptions parseOptions(int argc, char** argv) {
    DemoOptions options;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }

        if (index + 1 >= argc) {
            throw std::invalid_argument(argument + " requires a value");
        }

        const std::string value = argv[++index];
        if (argument == "--output") {
            options.output_path = value;
        } else if (argument == "--iterations") {
            options.iterations = parseSize(value, "--iterations");
        } else if (argument == "--seed") {
            options.seed = parseSeed(value);
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }

    return options;
}

double positionError(const mcl::Pose& first, const mcl::Pose& second) {
    return std::hypot(first.x - second.x, first.y - second.y);
}

mcl::RangeMeasurement addSyntheticNoise(
    const mcl::RangeMeasurement& expected,
    const mcl::MclConfig& config,
    std::mt19937& rng) {
    const auto noisyRange = [&](double range) {
        std::normal_distribution<double> noise(
            0.0,
            mcl::measurementStandardDeviation(range, config));
        return std::max(0.0, range + noise(rng));
    };

    return {
        noisyRange(expected.front),
        noisyRange(expected.left),
        noisyRange(expected.right),
    };
}

} // namespace

int main(int argc, char** argv) {
    try {
        const DemoOptions options = parseOptions(argc, argv);
        mcl::MclConfig config;
        mcl::MonteCarloLocalization localizer(config, options.seed);
        localizer.initialize();

        // This is synthetic ground truth for a host-side demonstration. It
        // does not claim to reproduce a recorded robot sensor trace.
        mcl::Pose truth{-38.0, -26.0, mcl::degreesToRadians(35.0)};
        const mcl::OdometryDelta odometry{
            1.2,
            0.15,
            mcl::degreesToRadians(1.5),
        };
        std::mt19937 sensor_rng(options.seed + 1);

        std::ofstream output(options.output_path);
        if (!output) {
            throw std::runtime_error("could not open output file: " + options.output_path);
        }

        output << "iteration,truth_x,truth_y,truth_theta_rad,estimate_x,estimate_y,"
                  "estimate_theta_rad,position_error,heading_error_rad,effective_sample_size\n";
        output << std::fixed << std::setprecision(6);

        mcl::Pose final_estimate{};

        for (std::size_t iteration = 1; iteration <= options.iterations; ++iteration) {
            truth = mcl::applyOdometry(truth, odometry);
            const auto expected_ranges = mcl::predictRanges(truth, config.field);
            if (!expected_ranges) {
                throw std::runtime_error("synthetic truth pose left the field");
            }

            localizer.update(
                odometry,
                addSyntheticNoise(*expected_ranges, config, sensor_rng));
            const mcl::Pose estimate = localizer.estimatePose();
            final_estimate = estimate;

            output << iteration << ','
                   << truth.x << ',' << truth.y << ',' << truth.theta << ','
                   << estimate.x << ',' << estimate.y << ',' << estimate.theta << ','
                   << positionError(truth, estimate) << ','
                   << std::abs(mcl::normalizeAngle(estimate.theta - truth.theta)) << ','
                   << localizer.effectiveSampleSize() << '\n';
        }

        std::cout << "MCL demo: synthetic " << config.particle_count
                  << "-particle run\n"
                  << "seed: " << options.seed << "\n"
                  << "iterations: " << options.iterations << "\n"
                  << "trace: " << options.output_path << "\n"
                  << "final position error: " << positionError(truth, final_estimate)
                  << " in\n"
                  << "final heading error: "
                  << mcl::radiansToDegrees(
                         std::abs(mcl::normalizeAngle(final_estimate.theta - truth.theta)))
                  << " deg\n";
    } catch (const std::exception& error) {
        std::cerr << "mcl_demo: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
