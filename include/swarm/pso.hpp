#pragma once

#include <random>
#include <vector>

#include "swarm/swarm_state.hpp"

namespace px_swarm::swarm {

struct Bounds {
    Vec3 min{};
    Vec3 max{};
};

struct PsoParams {
    int particles = 20;
    int iterations = 15;

    float w_inertia = 0.6f;
    float c_cognitive = 1.4f;
    float c_social = 1.4f;

    unsigned seed = 42;
};

/// Fitness terms weights for follower objective.
struct FitnessWeights {
    float w_form = 1.0f;     // closeness to desired position
    float w_coll = 3.0f;     // collision penalty
    float w_smooth = 0.2f;   // smoothness from prev target
};

struct PsoProblem {
    Vec3 desired{};                 // formation desired point
    std::vector<Vec3> neighbors{};  // neighbor positions
    float safe_radius_m = 2.0f;
    float cohesion_radius_m = 6.0f;

    std::optional<Vec3> prev_target{};
    FitnessWeights w{};
    Bounds bounds{};
};

class PsoOptimizer {
public:
    PsoOptimizer();

    Vec3 solve(const PsoParams& params, const PsoProblem& prob, float* best_fitness = nullptr);

private:
    std::mt19937 _rng;

    float _fitness(const Vec3& p, const PsoProblem& prob) const;

    static float _sq(float v) { return v * v; }
    static float _norm2(const Vec3& a, const Vec3& b);
    static Vec3 _clamp(const Vec3& p, const Bounds& b);
};

} // namespace px_swarm::swarm
