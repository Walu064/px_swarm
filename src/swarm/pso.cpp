#include "swarm/pso.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace px_swarm::swarm {

PsoOptimizer::PsoOptimizer() : _rng(42) {}

float PsoOptimizer::_norm2(const Vec3& a, const Vec3& b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx*dx + dy*dy + dz*dz;
}

Vec3 PsoOptimizer::_clamp(const Vec3& p, const Bounds& b)
{
    Vec3 out = p;
    out.x = std::min(std::max(out.x, b.min.x), b.max.x);
    out.y = std::min(std::max(out.y, b.min.y), b.max.y);
    out.z = std::min(std::max(out.z, b.min.z), b.max.z);
    return out;
}

float PsoOptimizer::_fitness(const Vec3& p, const PsoProblem& prob) const
{
    // 1) formation closeness
    float cost = prob.w.w_form * _norm2(p, prob.desired);

    // 2) collision penalty: hinge when closer than safe radius
    const float safe2 = prob.safe_radius_m * prob.safe_radius_m;
    const float cohesion2 = prob.cohesion_radius_m * prob.cohesion_radius_m;
    for (const auto& nb : prob.neighbors) {
        const float d2 = _norm2(p, nb);
        if (d2 < safe2) {
            const float d = std::sqrt(std::max(d2, 1e-6f));
            const float pen = (prob.safe_radius_m - d);
            cost += prob.w.w_coll * pen * pen;
        } else if (d2 > cohesion2) {
            const float d = std::sqrt(d2);
            const float pen = (d - prob.cohesion_radius_m);
            cost += 0.5f * prob.w.w_coll * pen * pen;
        }
    }

    // 3) smoothness from previous target
    if (prob.prev_target) {
        cost += prob.w.w_smooth * _norm2(p, *prob.prev_target);
    }

    return cost;
}

Vec3 PsoOptimizer::solve(const PsoParams& params, const PsoProblem& prob, float* best_fitness)
{
    _rng.seed(params.seed);

    std::uniform_real_distribution<float> ux(prob.bounds.min.x, prob.bounds.max.x);
    std::uniform_real_distribution<float> uy(prob.bounds.min.y, prob.bounds.max.y);
    std::uniform_real_distribution<float> uz(prob.bounds.min.z, prob.bounds.max.z);
    std::uniform_real_distribution<float> ur(0.f, 1.f);

    struct Particle {
        Vec3 x{};
        Vec3 v{};
        Vec3 best_x{};
        float best_f = std::numeric_limits<float>::infinity();
    };

    std::vector<Particle> ps(params.particles);

    // init
    Vec3 gbest_x{};
    float gbest_f = std::numeric_limits<float>::infinity();

    for (auto& p : ps) {
        p.x = Vec3{ux(_rng), uy(_rng), uz(_rng)};
        p.v = Vec3{0.f, 0.f, 0.f};
        p.best_x = p.x;
        p.best_f = _fitness(p.x, prob);

        if (p.best_f < gbest_f) {
            gbest_f = p.best_f;
            gbest_x = p.best_x;
        }
    }

    // iterate
    for (int it = 0; it < params.iterations; ++it) {
        for (auto& p : ps) {
            const float r1 = ur(_rng);
            const float r2 = ur(_rng);

            // velocity update
            p.v.x = params.w_inertia * p.v.x
                  + params.c_cognitive * r1 * (p.best_x.x - p.x.x)
                  + params.c_social    * r2 * (gbest_x.x - p.x.x);

            p.v.y = params.w_inertia * p.v.y
                  + params.c_cognitive * r1 * (p.best_x.y - p.x.y)
                  + params.c_social    * r2 * (gbest_x.y - p.x.y);

            p.v.z = params.w_inertia * p.v.z
                  + params.c_cognitive * r1 * (p.best_x.z - p.x.z)
                  + params.c_social    * r2 * (gbest_x.z - p.x.z);

            // position update
            p.x.x += p.v.x;
            p.x.y += p.v.y;
            p.x.z += p.v.z;

            p.x = _clamp(p.x, prob.bounds);

            const float f = _fitness(p.x, prob);
            if (f < p.best_f) {
                p.best_f = f;
                p.best_x = p.x;
                if (f < gbest_f) {
                    gbest_f = f;
                    gbest_x = p.x;
                }
            }
        }
    }

    if (best_fitness) {
        *best_fitness = gbest_f;
    }
    return gbest_x;
}

} // namespace px_swarm::swarm
