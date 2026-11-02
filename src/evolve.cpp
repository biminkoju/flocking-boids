// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Bimin Kiran Koju
#include "evolve.hpp"

#include "sim.hpp"

#include <raymath.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

namespace boids {
namespace {

// Genes under selection: the steering weights plus the radii that shape flock
// geometry. Everything else stays at its default so the search stays readable.
struct Genome {
    float separation, alignment, cohesion, follow;
    float separationRadius, alignmentRadius, cohesionRadius, followRadius;
    float maxSpeed, minSpeed;
};

float* gene(Genome& g, int i) {
    switch (i) {
        case 0: return &g.separation;
        case 1: return &g.alignment;
        case 2: return &g.cohesion;
        case 3: return &g.follow;
        case 4: return &g.separationRadius;
        case 5: return &g.alignmentRadius;
        case 6: return &g.cohesionRadius;
        case 7: return &g.followRadius;
        case 8: return &g.maxSpeed;
        default: return &g.minSpeed;
    }
}

void bounds(int i, float& lo, float& hi) {
    switch (i) {
        case 4: lo = 10.0f; hi = 80.0f;  break;
        case 5: lo = 20.0f; hi = 150.0f; break;
        case 6: lo = 20.0f; hi = 200.0f; break;
        case 7: lo = 80.0f; hi = 400.0f; break;
        case 8: lo = 120.0f; hi = 400.0f; break;
        case 9: lo = 20.0f;  hi = 180.0f; break;
        default: lo = 0.0f;  hi = 4.0f;   break;
    }
}

Genome fromDefault() {
    const Params p;
    Genome g{};
    g.separation = p.separation;  g.alignment = p.alignment;
    g.cohesion   = p.cohesion;    g.follow    = p.follow;
    g.separationRadius = p.separationRadius;
    g.alignmentRadius  = p.alignmentRadius;
    g.cohesionRadius   = p.cohesionRadius;
    g.followRadius     = p.followRadius;
    g.maxSpeed = p.maxSpeed;      g.minSpeed = p.minSpeed;
    return g;
}

void apply(const Genome& g, Params& p) {
    p.separation = g.separation;  p.alignment = g.alignment;
    p.cohesion   = g.cohesion;    p.follow    = g.follow;
    p.separationRadius = g.separationRadius;
    p.alignmentRadius  = g.alignmentRadius;
    p.cohesionRadius   = g.cohesionRadius;
    p.followRadius     = g.followRadius;
    p.maxSpeed = g.maxSpeed;      p.minSpeed = g.minSpeed;
}

// Fitness: goals eaten dominate, with polarisation as a secondary term so a
// degenerate "flock that never coheres" cannot win by luck.
float evaluate(const Genome& g, int frames) {
    Sim sim;
    apply(g, sim.params);
    sim.params.foodDrain   = 0.25f;
    sim.params.foodRadius  = 120.0f;
    sim.params.foodPull    = 0.3f;
    sim.foodEnabled = true;

    sim.setCount(300);
    sim.setLeaderCount(12);

    const float dt = 1.0f / 60.0f;
    float orderSum = 0.0f;
    for (int i = 0; i < frames; ++i) {
        sim.step(dt, false, Vector2{0.0f, 0.0f}, Vector2{0.0f, 0.0f});
        orderSum += sim.stats.order;
    }
    const float meanOrder = orderSum / (float)frames;
    return (float)sim.targetsEaten * 10.0f + meanOrder * 20.0f;
}

}  // namespace

int runEvolve(int generations, const char* outPath) {
    std::mt19937 rng{20260101u};
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);

    constexpr int kPop = 12;
    constexpr int kElite = 3;
    constexpr int kFrames = 240;   // ~4 s of simulated flock per evaluation

    // Seed the population from the defaults plus gaussian jitter.
    std::vector<Genome> pop(kPop);
    const Genome base = fromDefault();
    for (int i = 0; i < kPop; ++i) {
        pop[(size_t)i] = base;
        if (i == 0) continue;   // keep one untouched baseline in the pool
        for (int gi = 0; gi < 10; ++gi) {
            float lo, hi;
            bounds(gi, lo, hi);
            const float* b = gene(const_cast<Genome&>(base), gi);
            float* v = gene(pop[(size_t)i], gi);
            *v = Clamp(*b + (*b) * 0.4f * (unit(rng) * 2.0f - 1.0f), lo, hi);
        }
    }

    float bestFitness = -1e30f;
    Genome best = base;

    for (int gen = 0; gen < generations; ++gen) {
        std::vector<float> fit(kPop);
        for (int i = 0; i < kPop; ++i) {
            setRandomSeed(4242u);   // same scenario for everyone: fair comparison
            fit[(size_t)i] = evaluate(pop[(size_t)i], kFrames);
        }

        std::vector<int> order(kPop);
        for (int i = 0; i < kPop; ++i) order[(size_t)i] = i;
        std::sort(order.begin(), order.end(),
                  [&](int a, int b) { return fit[(size_t)a] > fit[(size_t)b]; });

        const float genBest = fit[(size_t)order[0]];
        float mean = 0.0f;
        for (float f : fit) mean += f;
        mean /= (float)kPop;

        if (genBest > bestFitness) {
            bestFitness = genBest;
            best = pop[(size_t)order[0]];
        }
        std::printf("gen %3d  best %7.2f  mean %7.2f\n", gen, (double)genBest, (double)mean);

        // Elitism, then breed the rest from tournament-selected parents.
        std::vector<Genome> next;
        next.reserve((size_t)kPop);
        for (int i = 0; i < kElite; ++i) next.push_back(pop[(size_t)order[(size_t)i]]);

        while ((int)next.size() < kPop) {
            auto pick = [&]() {
                const int a = (int)(unit(rng) * kPop) % kPop;
                const int b = (int)(unit(rng) * kPop) % kPop;
                return fit[(size_t)a] > fit[(size_t)b] ? a : b;
            };
            Genome child = pop[(size_t)pick()];
            const Genome other = pop[(size_t)pick()];
            for (int gi = 0; gi < 10; ++gi) {
                float lo, hi;
                bounds(gi, lo, hi);
                float* v = gene(child, gi);
                if (unit(rng) < 0.5f) *v = *gene(const_cast<Genome&>(other), gi);
                if (unit(rng) < 0.25f) {
                    const float span = hi - lo;
                    *v = Clamp(*v + span * 0.12f * (unit(rng) * 2.0f - 1.0f), lo, hi);
                }
            }
            next.push_back(child);
        }
        pop = std::move(next);
    }

    Params out;
    apply(best, out);
    FILE* fp = std::fopen(outPath, "wb");
    if (!fp) { std::printf("cannot write %s\n", outPath); return 1; }
    const std::string text = paramsToJson(out);
    std::fwrite(text.data(), 1, text.size(), fp);
    std::fclose(fp);

    std::printf("best fitness %.2f  ->  %s\n", (double)bestFitness, outPath);
    return 0;
}

}  // namespace boids
