// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Bimin Kiran Koju
#include "sim.hpp"

#include <raymath.h>

#include <algorithm>
#include <cmath>
#ifdef BOIDS_PARALLEL
#include <execution>
#endif
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <utility>

namespace boids {
namespace {

constexpr uint32_t kRecMagic = 0x4b4c4642u;  // "BFLK"

std::mt19937& rng() {
    static std::mt19937 gen{std::random_device{}()};
    return gen;
}

}  // namespace

void setRandomSeed(unsigned int seed) { rng().seed(seed); }

namespace {

float randRange(float lo, float hi) {
    return std::uniform_real_distribution<float>(lo, hi)(rng());
}

Vector2 limitVec(Vector2 v, float max) {
    const float len = Vector2Length(v);
    if (len > max && len > 0.0f) v = Vector2Scale(v, max / len);
    return v;
}

// Reynolds: desired velocity of length maxSpeed, minus current velocity.
Vector2 steerTowards(const Boid& b, Vector2 desired, const Params& p, float maxSpeed = -1.0f) {
    if (Vector2Length(desired) == 0.0f) return Vector2{0.0f, 0.0f};
    if (maxSpeed <= 0.0f) maxSpeed = p.maxSpeed;
    desired = Vector2Scale(Vector2Normalize(desired), maxSpeed);
    return limitVec(Vector2Subtract(desired, b.vel), p.maxForce);
}

// Energy scales the usable top speed: a spent boid can only coast.
float boidMaxSpeed(const Boid& b, const Params& p) {
    const float fatigue = p.energyEnabled ? 0.45f + 0.55f * Clamp(b.energy, 0.0f, 1.0f) : 1.0f;
    return p.maxSpeed * b.cruise * fatigue;
}

// Shortest offset from d in a wrapped world (identity when walls are used).
Vector2 wrapDelta(Vector2 d, const Params& p) {
    if (!p.periodicBounds) return d;
    const float w = (float)kScreenWidth;
    const float h = (float)kScreenHeight;
    if (d.x >  w * 0.5f) d.x -= w;
    if (d.x < -w * 0.5f) d.x += w;
    if (d.y >  h * 0.5f) d.y -= h;
    if (d.y < -h * 0.5f) d.y += h;
    return d;
}

float worldDist2(Vector2 a, Vector2 b, const Params& p) {
    const Vector2 d = wrapDelta(Vector2Subtract(b, a), p);
    return Vector2DotProduct(d, d);
}

// Global flow field: base direction from windAngleDeg, plus a sine sum over
// position and time weighted by turbulence / gust. Wavenumbers are integer
// multiples of 2*pi/screenSize so the field is seamless across the wrap seam.
Vector2 windAt(Vector2 pos, float t, const Params& p) {
    if (p.windStrength <= 0.0f) return Vector2{0.0f, 0.0f};

    const float kx = 2.0f * PI / (float)kScreenWidth;
    const float ky = 2.0f * PI / (float)kScreenHeight;
    const float turb = Clamp(p.windTurbulence, 0.0f, 1.0f);
    const float gust = std::max(0.0f, p.windGust);

    const float a = std::sin(pos.x * kx * 2.0f + t * gust) *
                    std::cos(pos.y * ky * 3.0f - t * gust * 0.7f);
    const float b = std::cos(pos.x * kx * 3.0f - t * gust * 0.9f) *
                    std::sin(pos.y * ky * 2.0f + t * gust * 1.3f);

    const float ang = p.windAngleDeg * (PI / 180.0f);
    const Vector2 base{std::cos(ang), std::sin(ang)};
    const Vector2 swirl{-base.y, base.x};

    Vector2 dir = Vector2Add(base, Vector2Scale(swirl, a * turb));
    dir = Vector2Add(dir, Vector2Scale(base, b * turb * 0.5f));
    const float len = Vector2Length(dir);
    if (len <= 1e-5f) return Vector2{0.0f, 0.0f};
    dir = Vector2Scale(dir, 1.0f / len);

    const float scale = p.windStrength * (1.0f + gust * 0.25f * std::sin(t * gust + 1.7f));
    return Vector2Scale(dir, std::max(0.0f, scale));
}

// Perpendicular pointing away from awayFrom.
Vector2 sideAway(const Boid& b, Vector2 awayFrom) {
    Vector2 side{-b.vel.y, b.vel.x};
    if (Vector2DotProduct(side, Vector2Subtract(b.pos, awayFrom)) < 0.0f)
        side = Vector2Negate(side);
    return Vector2Normalize(side);
}

// True when the neighbour offset d lies inside the forward field of view.
bool inFov(const Vector2 heading, Vector2 d, const Params& p, float dist) {
    if (p.fovCos >= 1.0f) return true;
    return Vector2DotProduct(heading, d) > p.fovCos * dist;
}

// Real tangent length from a point at dist to a circle of radius, as a
// fraction of dist (1 when the point is inside the circle).
float tangentRatio(float dist, float radius) {
    if (dist <= radius) return 1.0f;
    const float r = radius / dist;
    return std::sqrt(std::max(0.0f, 1.0f - r * r));
}

// ------------------------------------------------------------- JSON helpers

const char* findValue(const std::string& s, const char* key) {
    const std::string pat = std::string("\"") + key + "\"";
    const size_t k = s.find(pat);
    if (k == std::string::npos) return nullptr;
    const size_t c = s.find(':', k + pat.size());
    if (c == std::string::npos) return nullptr;
    return s.c_str() + c + 1;
}

void jsonFloat(const std::string& s, const char* key, float& dst) {
    if (const char* v = findValue(s, key)) dst = (float)std::atof(v);
}

void jsonInt(const std::string& s, const char* key, int& dst) {
    if (const char* v = findValue(s, key)) dst = std::atoi(v);
}

void jsonBool(const std::string& s, const char* key, bool& dst) {
    if (const char* v = findValue(s, key)) {
        const char* t = std::strstr(v, "true");
        const char* f = std::strstr(v, "false");
        if (t && (!f || t < f)) dst = true;
        else if (f) dst = false;
    }
}

std::string readTextFile(const char* path) {
    char* txt = LoadFileText(path);
    std::string out = txt ? txt : "";
    if (txt) UnloadFileText(txt);
    return out;
}

bool writeTextFile(const char* path, const std::string& text) {
    std::string copy = text;
    return SaveFileText(path, copy.data());
}

}  // namespace

Vector2 Boid::heading() const {
    const float l = Vector2Length(vel);
    return l > 0.0f ? Vector2Scale(vel, 1.0f / l) : Vector2{1.0f, 0.0f};
}

// ----------------------------------------------------------------- Params IO

std::string paramsToJson(const Params& p) {
    char b[4096];
    std::snprintf(b, sizeof(b),
        "{\n"
        "  \"separation\": %.4f,\n  \"alignment\": %.4f,\n  \"cohesion\": %.4f,\n"
        "  \"obstacle\": %.4f,\n  \"predator\": %.4f,\n  \"target\": %.4f,\n"
        "  \"follow\": %.4f,\n"
        "  \"separationRadius\": %.2f,\n  \"alignmentRadius\": %.2f,\n"
        "  \"cohesionRadius\": %.2f,\n  \"followRadius\": %.2f,\n"
        "  \"obstacleLook\": %.2f,\n  \"predatorRadius\": %.2f,\n"
        "  \"targetRadius\": %.2f,\n  \"boundsMargin\": %.2f,\n"
        "  \"boundsWeight\": %.2f,\n"
        "  \"maxSpeed\": %.2f,\n  \"minSpeed\": %.2f,\n  \"maxForce\": %.2f,\n"
        "  \"forceBudget\": %.2f,\n  \"predatorSpeed\": %.2f,\n"
        "  \"predatorForce\": %.2f,\n  \"predatorHunt\": %.2f,\n"
        "  \"predatorRest\": %.2f,\n  \"catchRadius\": %.2f,\n"
        "  \"energyDrain\": %.4f,\n  \"energyRecover\": %.4f,\n"
        "  \"energyPanicDrain\": %.4f,\n"
        "  \"mouseRadius\": %.2f,\n"
        "  \"mousePush\": %.2f,\n  \"fovCos\": %.4f,\n"
        "  \"periodicBounds\": %s,\n  \"hardWalls\": %s,\n"
        "  \"topological\": %s,\n  \"topoK\": %d,\n"
        "  \"distanceWeighted\": %s,\n  \"energyEnabled\": %s,\n"
        "  \"occlusion\": %s,\n  \"multiSpecies\": %s,\n  \"speciesCount\": %d,\n"
        "  \"crossSpecies\": %.4f,\n"
        "  \"windStrength\": %.2f,\n  \"windAngleDeg\": %.2f,\n"
        "  \"windTurbulence\": %.4f,\n  \"windGust\": %.4f,\n"
        "  \"foodDrain\": %.4f,\n  \"foodRadius\": %.2f,\n"
        "  \"foodPull\": %.4f,\n  \"huntPressure\": %.4f\n"
        "}\n",
        p.separation, p.alignment, p.cohesion, p.obstacle, p.predator, p.target, p.follow,
        p.separationRadius, p.alignmentRadius, p.cohesionRadius, p.followRadius,
        p.obstacleLook, p.predatorRadius, p.targetRadius, p.boundsMargin, p.boundsWeight,
        p.maxSpeed, p.minSpeed, p.maxForce, p.forceBudget, p.predatorSpeed, p.predatorForce,
        p.predatorHunt, p.predatorRest, p.catchRadius,
        p.energyDrain, p.energyRecover, p.energyPanicDrain,
        p.mouseRadius, p.mousePush, p.fovCos,
        p.periodicBounds ? "true" : "false",
        p.hardWalls ? "true" : "false",
        p.topological ? "true" : "false", p.topoK,
        p.distanceWeighted ? "true" : "false",
        p.energyEnabled ? "true" : "false",
        p.occlusion ? "true" : "false",
        p.multiSpecies ? "true" : "false", p.speciesCount,
        (double)p.crossSpecies,
        p.windStrength, p.windAngleDeg, p.windTurbulence, p.windGust,
        p.foodDrain, p.foodRadius, p.foodPull, p.huntPressure);
    return b;
}

void paramsFromJson(const std::string& text, Params& p) {
    Params q = p;
    jsonFloat(text, "separation", q.separation);
    jsonFloat(text, "alignment", q.alignment);
    jsonFloat(text, "cohesion", q.cohesion);
    jsonFloat(text, "obstacle", q.obstacle);
    jsonFloat(text, "predator", q.predator);
    jsonFloat(text, "target", q.target);
    jsonFloat(text, "follow", q.follow);
    jsonFloat(text, "separationRadius", q.separationRadius);
    jsonFloat(text, "alignmentRadius", q.alignmentRadius);
    jsonFloat(text, "cohesionRadius", q.cohesionRadius);
    jsonFloat(text, "followRadius", q.followRadius);
    jsonFloat(text, "obstacleLook", q.obstacleLook);
    jsonFloat(text, "predatorRadius", q.predatorRadius);
    jsonFloat(text, "targetRadius", q.targetRadius);
    jsonFloat(text, "boundsMargin", q.boundsMargin);
    jsonFloat(text, "boundsWeight", q.boundsWeight);
    jsonFloat(text, "maxSpeed", q.maxSpeed);
    jsonFloat(text, "minSpeed", q.minSpeed);
    jsonFloat(text, "maxForce", q.maxForce);
    jsonFloat(text, "forceBudget", q.forceBudget);
    jsonFloat(text, "predatorSpeed", q.predatorSpeed);
    jsonFloat(text, "predatorForce", q.predatorForce);
    jsonFloat(text, "predatorHunt", q.predatorHunt);
    jsonFloat(text, "predatorRest", q.predatorRest);
    jsonFloat(text, "catchRadius", q.catchRadius);
    jsonFloat(text, "energyDrain", q.energyDrain);
    jsonFloat(text, "energyRecover", q.energyRecover);
    jsonFloat(text, "energyPanicDrain", q.energyPanicDrain);
    jsonBool(text, "periodicBounds", q.periodicBounds);
    jsonBool(text, "hardWalls", q.hardWalls);
    jsonBool(text, "topological", q.topological);
    jsonInt(text, "topoK", q.topoK);
    jsonBool(text, "distanceWeighted", q.distanceWeighted);
    jsonBool(text, "energyEnabled", q.energyEnabled);
    jsonBool(text, "occlusion", q.occlusion);
    jsonBool(text, "multiSpecies", q.multiSpecies);
    jsonInt(text, "speciesCount", q.speciesCount);
    q.speciesCount = std::max(2, std::min(q.speciesCount, kSpeciesMax));
    jsonFloat(text, "crossSpecies", q.crossSpecies);
    jsonFloat(text, "windStrength", q.windStrength);
    jsonFloat(text, "windAngleDeg", q.windAngleDeg);
    jsonFloat(text, "windTurbulence", q.windTurbulence);
    jsonFloat(text, "windGust", q.windGust);
    jsonFloat(text, "foodDrain", q.foodDrain);
    jsonFloat(text, "foodRadius", q.foodRadius);
    jsonFloat(text, "foodPull", q.foodPull);
    jsonFloat(text, "huntPressure", q.huntPressure);
    q.foodPull = Clamp(q.foodPull, 0.0f, 1.0f);
    q.foodDrain = std::max(0.0f, q.foodDrain);
    q.windAngleDeg = std::fmod(std::fmod(q.windAngleDeg, 360.0f) + 360.0f, 360.0f);
    q.windTurbulence = Clamp(q.windTurbulence, 0.0f, 1.0f);
    q.topoK = std::max(1, std::min(q.topoK, kTopoKMax));
    q.topoKf = (float)q.topoK;
    jsonFloat(text, "mouseRadius", q.mouseRadius);
    jsonFloat(text, "mousePush", q.mousePush);
    jsonFloat(text, "fovCos", q.fovCos);
    p = q;
}

bool loadAppState(const char* path, AppState& s) {
    const std::string text = readTextFile(path);
    if (text.empty()) return false;
    paramsFromJson(text, s.params);
    jsonBool(text, "panelOpen", s.panelOpen);
    jsonInt(text, "boidCount", s.boidCount);
    jsonInt(text, "leaderCount", s.leaderCount);
    jsonBool(text, "trails", s.trails);
    jsonBool(text, "autoLeaders", s.autoLeaders);
    jsonBool(text, "kills", s.kills);
    jsonBool(text, "windEnabled", s.windEnabled);
    jsonBool(text, "foodEnabled", s.foodEnabled);
    jsonInt(text, "tool", s.tool);
    jsonFloat(text, "brushRadius", s.brushRadius);
    return true;
}

bool saveAppState(const char* path, const AppState& s) {
    std::string text = paramsToJson(s.params);
    text.resize(text.size() - 2);  // drop the trailing "\n}"
    char b[512];
    std::snprintf(b, sizeof(b),
                  ",\n  \"panelOpen\": %s,\n  \"boidCount\": %d,\n  \"leaderCount\": %d,\n"
                  "  \"trails\": %s,\n  \"autoLeaders\": %s,\n  \"kills\": %s,\n"
                  "  \"tool\": %d,\n  \"brushRadius\": %.2f,\n"
                  "  \"windEnabled\": %s,\n  \"foodEnabled\": %s\n}\n",
                  s.panelOpen ? "true" : "false", s.boidCount, s.leaderCount,
                  s.trails ? "true" : "false", s.autoLeaders ? "true" : "false",
                  s.kills ? "true" : "false", s.tool, (double)s.brushRadius,
                  s.windEnabled ? "true" : "false",
                  s.foodEnabled ? "true" : "false");
    text += b;
    return writeTextFile(path, text);
}

std::vector<Preset> presets() {
    std::vector<Preset> out;
    const Params p;

    out.push_back({PresetDefault, "Default", p, false, false, false});

    Params tight = p;
    tight.cohesion = 2.6f;
    tight.separation = 1.4f;
    tight.alignment = 1.4f;
    tight.cohesionRadius = 90.0f;
    tight.alignmentRadius = 50.0f;
    tight.maxSpeed = 180.0f;
    out.push_back({PresetBall, "Tight ball", tight, false, false, false});

    Params stream = p;
    stream.alignment = 2.8f;
    stream.cohesion = 0.35f;
    stream.separation = 2.0f;
    stream.fovCos = -0.55f;
    stream.maxSpeed = 270.0f;
    stream.alignmentRadius = 90.0f;
    stream.cohesionRadius = 70.0f;
    out.push_back({PresetStream, "Streaming", stream, false, false, false});

    Params panic = p;
    panic.predator = 4.0f;
    panic.separation = 2.6f;
    panic.alignment = 2.0f;
    panic.cohesion = 1.4f;
    panic.maxSpeed = 300.0f;
    panic.minSpeed = 110.0f;
    panic.predatorRadius = 240.0f;
    panic.predatorSpeed = 210.0f;
    panic.follow = 0.6f;
    // Panic actually turns predation on; without that it was only a weight
    // tweak with nothing to panic about.
    panic.huntPressure = 0.5f;
    panic.energyEnabled = true;
    out.push_back({PresetPanic, "Panic", panic, true, false, false});

    // Murmuration: the Ballerini setup - topological neighbours in a wrapped
    // world, so the flock streams and folds instead of bunching at the walls.
    Params murm = p;
    murm.topological = true;
    murm.topoK = 7;
    murm.topoKf = 7.0f;
    murm.periodicBounds = true;
    murm.distanceWeighted = true;
    murm.alignment = 1.6f;
    murm.cohesion = 1.1f;
    murm.separation = 1.9f;
    murm.fovCos = -0.5f;
    murm.maxSpeed = 240.0f;
    out.push_back({PresetMurmuration, "Murmuration", murm, false, false, false});

    // Forage: objectives deplete and the flock must keep choosing new ones.
    Params forage = p;
    forage.foodPull = 0.35f;
    forage.foodDrain = 0.12f;
    forage.foodRadius = 90.0f;
    forage.target = 2.2f;
    forage.follow = 1.1f;
    out.push_back({PresetForage, "Forage", forage, false, false, true});

    // Contest: two flocks that refuse to mix, competing for the same space.
    Params contest = p;
    contest.multiSpecies = true;
    contest.speciesCount = 2;
    contest.crossSpecies = 0.0f;
    contest.separation = 2.4f;
    contest.cohesion = 1.3f;
    out.push_back({PresetContest, "Contest", contest, false, false, false});

    // Breeze: a turbulent, gusting flow field with strong flocking, so the
    // flock visibly holds shape while being pushed around.
    Params breeze = p;
    breeze.windStrength = 120.0f;
    breeze.windAngleDeg = 20.0f;
    breeze.windTurbulence = 0.7f;
    breeze.windGust = 1.2f;
    breeze.alignment = 1.6f;
    breeze.cohesion = 1.2f;
    out.push_back({PresetBreeze, "Breeze", breeze, false, true, false});

    return out;
}

// -------------------------------------------------------------------- Recorder

void Recorder::capture(const Sim& sim) {
    // Remember the params of the first captured frame: replays must draw with
    // the radii/speeds the run used, not with the defaults.
    if (frames.empty()) params = sim.params;

    Frame f;
    const size_t n = sim.boids.size();
    f.pos.resize(n);
    f.vel.resize(n);
    f.species.resize(n);
    f.flags.resize(n);
    f.energy.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const Boid& b = sim.boids[i];
        f.pos[i]     = b.pos;
        f.vel[i]     = b.vel;
        f.species[i] = (uint8_t)Clamp(b.species, 0, kSpeciesMax - 1);
        f.flags[i]   = b.leader ? 1u : 0u;
        f.energy[i]  = b.energy;
    }
    f.predators = sim.predators;
    f.targets   = sim.targets;
    f.multiSpecies  = sim.params.multiSpecies;
    f.energyEnabled = sim.params.energyEnabled;
    frames.push_back(std::move(f));
}

bool Recorder::save(const char* path) const {
    FILE* fp = std::fopen(path, "wb");
    if (!fp) return false;
    const uint32_t magic = kRecMagic;
    const uint32_t version = 3;   // v3 adds the params header (JSON)
    const uint32_t count = (uint32_t)frames.size();
    std::fwrite(&magic, sizeof(magic), 1, fp);
    std::fwrite(&version, sizeof(version), 1, fp);
    std::fwrite(&count, sizeof(count), 1, fp);

    // v3 header: the Params in force, as JSON. Length-prefixed so the header
    // can grow later without breaking the reader.
    {
        const std::string pj = paramsToJson(params);
        const uint32_t len = (uint32_t)pj.size();
        std::fwrite(&len, sizeof(len), 1, fp);
        std::fwrite(pj.data(), 1, len, fp);
    }
    for (const Frame& f : frames) {
        const uint32_t nb = (uint32_t)f.pos.size();
        const uint32_t np = (uint32_t)f.predators.size();
        std::fwrite(&nb, sizeof(nb), 1, fp);
        std::fwrite(&np, sizeof(np), 1, fp);
        if (nb) {
            std::fwrite(f.pos.data(), sizeof(Vector2), nb, fp);
            std::fwrite(f.vel.data(), sizeof(Vector2), nb, fp);
        }
        for (const Predator& pr : f.predators) {
            std::fwrite(&pr.pos, sizeof(Vector2), 1, fp);
            std::fwrite(&pr.vel, sizeof(Vector2), 1, fp);
        }

        // v2 block. Explicit sizes so a truncated file is detectable.
        const uint32_t nt = (uint32_t)f.targets.size();
        const uint8_t  ms = f.multiSpecies ? 1u : 0u;
        const uint8_t  ee = f.energyEnabled ? 1u : 0u;
        std::fwrite(&ms, sizeof(ms), 1, fp);
        std::fwrite(&ee, sizeof(ee), 1, fp);
        std::fwrite(&nt, sizeof(nt), 1, fp);
        if (nb) {
            std::fwrite(f.species.data(), 1, nb, fp);
            std::fwrite(f.flags.data(), 1, nb, fp);
            std::fwrite(f.energy.data(), sizeof(float), nb, fp);
        }
        for (const Target& t : f.targets) {
            const uint8_t active = t.active ? 1u : 0u;
            std::fwrite(&t.pos, sizeof(Vector2), 1, fp);
            std::fwrite(&t.amount, sizeof(float), 1, fp);
            std::fwrite(&active, sizeof(active), 1, fp);
        }
    }
    std::fclose(fp);
    return true;
}

bool Recorder::load(const char* path) {
    FILE* fp = std::fopen(path, "rb");
    if (!fp) return false;
    uint32_t magic = 0, version = 0, count = 0;
    if (std::fread(&magic, sizeof(magic), 1, fp) != 1 || magic != kRecMagic ||
        std::fread(&version, sizeof(version), 1, fp) != 1 ||
        std::fread(&count, sizeof(count), 1, fp) != 1) {
        std::fclose(fp);
        return false;
    }

    // v3 header: parameters of the recorded run. v1/v2 files keep the defaults.
    params = Params{};
    if (version >= 3) {
        uint32_t len = 0;
        if (std::fread(&len, sizeof(len), 1, fp) != 1 || len > (1u << 20)) {
            std::fclose(fp);
            return false;
        }
        std::string pj(len, '\0');
        if (len && std::fread(&pj[0], 1, len, fp) != len) {
            std::fclose(fp);
            return false;
        }
        paramsFromJson(pj, params);
    }

    frames.clear();
    frames.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t nb = 0, np = 0;
        if (std::fread(&nb, sizeof(nb), 1, fp) != 1 ||
            std::fread(&np, sizeof(np), 1, fp) != 1) break;
        Frame f;
        f.pos.resize(nb);
        f.vel.resize(nb);
        bool ok = true;
        if (nb) {
            if (std::fread(f.pos.data(), sizeof(Vector2), nb, fp) != nb) ok = false;
            if (ok && std::fread(f.vel.data(), sizeof(Vector2), nb, fp) != nb) ok = false;
        }
        if (ok) {
            f.predators.resize(np);
            for (uint32_t k = 0; k < np && ok; ++k) {
                if (std::fread(&f.predators[k].pos, sizeof(Vector2), 1, fp) != 1 ||
                    std::fread(&f.predators[k].vel, sizeof(Vector2), 1, fp) != 1) ok = false;
                f.predators[k].active = true;
            }
        }

        // v2 block: absent in older files, where the frame keeps its defaults
        // and replays with the legacy look.
        if (ok && version >= 2) {
            uint8_t  ms = 0, ee = 0;
            uint32_t nt = 0;
            if (std::fread(&ms, sizeof(ms), 1, fp) != 1 ||
                std::fread(&ee, sizeof(ee), 1, fp) != 1 ||
                std::fread(&nt, sizeof(nt), 1, fp) != 1) {
                ok = false;
            } else {
                f.multiSpecies  = ms != 0;
                f.energyEnabled = ee != 0;
                if (nb) {
                    f.species.resize(nb);
                    f.flags.resize(nb);
                    f.energy.resize(nb);
                    if (std::fread(f.species.data(), 1, nb, fp) != nb ||
                        std::fread(f.flags.data(), 1, nb, fp) != nb ||
                        std::fread(f.energy.data(), sizeof(float), nb, fp) != nb)
                        ok = false;
                }
                if (ok) {
                    f.targets.resize(nt);
                    for (uint32_t k = 0; k < nt && ok; ++k) {
                        uint8_t active = 0;
                        if (std::fread(&f.targets[k].pos, sizeof(Vector2), 1, fp) != 1 ||
                            std::fread(&f.targets[k].amount, sizeof(float), 1, fp) != 1 ||
                            std::fread(&active, sizeof(active), 1, fp) != 1) ok = false;
                        f.targets[k].active = active != 0;
                    }
                }
            }
        }
        if (!ok) break;
        frames.push_back(std::move(f));
    }
    std::fclose(fp);
    index = 0;
    playing = false;
    return !frames.empty();
}

// ------------------------------------------------------------------------ Sim

Sim::Sim() { reset(250); }

int Sim::speciesN() const {
    if (!params.multiSpecies) return 1;
    return std::max(2, std::min(params.speciesCount, kSpeciesMax));
}

// World == window. When the window changes size, wrap (in toroidal mode) or clamp
// (otherwise) every position back inside the new bounds, and re-wrap obstacle and
// target radii so nothing ends up buried in a wall. Rebuilding the grid is not
// needed: it is rebuilt every step anyway.
void Sim::onWorldResized() {
    const float w = (float)kScreenWidth;
    const float h = (float)kScreenHeight;
    if (w <= 1.0f || h <= 1.0f) return;

    auto fix = [&](Vector2& p) {
        if (params.periodicBounds) {
            p.x = std::fmod(std::fmod(p.x, w) + w, w);
            p.y = std::fmod(std::fmod(p.y, h) + h, h);
        } else {
            p.x = Clamp(p.x, 0.0f, w);
            p.y = Clamp(p.y, 0.0f, h);
        }
    };

    for (Boid& b : boids) fix(b.pos);
    for (Predator& pr : predators) fix(pr.pos);
    for (Target& t : targets) {
        fix(t.pos);
        t.pos.x = Clamp(t.pos.x, 0.0f, std::max(0.0f, w - 1.0f));
        t.pos.y = Clamp(t.pos.y, 0.0f, std::max(0.0f, h - 1.0f));
    }
    for (Obstacle& o : obstacles) {
        const float r = std::min(o.radius, std::min(w, h) * 0.5f);
        o.radius = r;
        o.pos.x = Clamp(o.pos.x, r, std::max(r, w - r));
        o.pos.y = Clamp(o.pos.y, r, std::max(r, h - r));
    }

    // Wind wavenumbers are quantised to the screen size, so the field is still
    // seamless at the new dimensions; nothing to rebuild there.
    leaderTimer = 1e9f;   // leadership depends on distances: re-score next step
}

void Sim::redistributeSpecies() {
    const int n = speciesN();
    for (size_t i = 0; i < boids.size(); ++i)
        boids[i].species = (n > 1) ? (int)(i % (size_t)n) : 0;
}

void Sim::reset(int n) {
    n = std::max(0, std::min(n, kBoidMax));
    boids.clear();
    boids.reserve(n);
    for (int i = 0; i < n; ++i) {
        Boid b;
        b.pos = {randRange(0, (float)kScreenWidth), randRange(0, (float)kScreenHeight)};
        const float a = randRange(0, 2.0f * PI);
        const float s = randRange(params.minSpeed, params.maxSpeed);
        b.vel = {std::cos(a) * s, std::sin(a) * s};
        b.cruise = randRange(0.75f, 1.25f);
        b.energy = 1.0f;
        b.species = params.multiSpecies ? (i % speciesN()) : 0;
        boids.push_back(b);
    }

    obstacles.clear();
    obstacles.push_back({{400.0f, 260.0f}, 44.0f});
    obstacles.push_back({{860.0f, 540.0f}, 60.0f});
    obstacles.push_back({{640.0f, 140.0f}, 30.0f});

    targets.assign(1, Target{{640.0f, 400.0f}, false});

    predators.assign(1, Predator{});
    predators[0].pos = {1100.0f, 700.0f};
    predators[0].vel = {-params.predatorSpeed, 0.0f};
    predators[0].hunting = true;
    predators[0].timer = params.predatorHunt;
    predators[0].active = false;

    for (Predator& pr : predators) pr.kills = 0;
    killCount = 0;
    targetsEaten = 0;
    simTime   = 0.0f;
    stats.clearHistory();

    leaderIdx.clear();
    leaderTimer = 0.0f;
    updateLeaders();
}

void Sim::addBoids(int n) {
    if (n <= 0) return;
    boids.reserve(boids.size() + n);
    for (int i = 0; i < n; ++i) {
        Boid b;
        Boid src;
        if (boids.empty()) {
            src.pos = {kScreenWidth * 0.5f, kScreenHeight * 0.5f};
            src.vel = {params.maxSpeed, 0.0f};
        } else {
            src = boids[(size_t)(randRange(0, (float)boids.size())) % boids.size()];
        }
        b.pos = Vector2Add(src.pos, {randRange(-40, 40), randRange(-40, 40)});
        b.vel = src.vel;
        b.cruise = randRange(0.75f, 1.25f);
        b.energy = 1.0f;
        b.species = params.multiSpecies ? (i % speciesN()) : 0;
        b.leader = false;
        boids.push_back(b);
    }
}

void Sim::removeBoids(int n) {
    for (int i = 0; i < n && !boids.empty(); ++i) boids.pop_back();
}

int Sim::spawnBoidsAt(Vector2 center, float radius, int n) {
    const int room = kBoidMax - (int)boids.size();
    n = std::max(0, std::min(n, room));
    if (n <= 0) return 0;
    boids.reserve(boids.size() + (size_t)n);
    for (int i = 0; i < n; ++i) {
        Boid b;
        const float a = randRange(0.0f, 2.0f * PI);
        const float r = radius * std::sqrt(randRange(0.0f, 1.0f));  // uniform in the disc
        b.pos = {center.x + std::cos(a) * r, center.y + std::sin(a) * r};
        if (params.periodicBounds) {
            b.pos.x = std::fmod(b.pos.x + (float)kScreenWidth, (float)kScreenWidth);
            b.pos.y = std::fmod(b.pos.y + (float)kScreenHeight, (float)kScreenHeight);
        } else {
            b.pos.x = Clamp(b.pos.x, 0.0f, (float)kScreenWidth);
            b.pos.y = Clamp(b.pos.y, 0.0f, (float)kScreenHeight);
        }
        const float va = randRange(0.0f, 2.0f * PI);
        const float vs = randRange(params.minSpeed, params.maxSpeed);
        b.vel = {std::cos(va) * vs, std::sin(va) * vs};
        b.cruise = randRange(0.75f, 1.25f);
        b.energy = 1.0f;
        // Painted boids join a random flock so you can top up any species.
        b.species = params.multiSpecies
            ? std::min(speciesN() - 1, (int)randRange(0.0f, (float)speciesN())) : 0;
        boids.push_back(b);
    }
    leaderTimer = 1e9f;   // refresh leadership on the next step
    return n;
}

int Sim::eraseBoidsAt(Vector2 center, float radius) {
    const float r2 = radius * radius;
    const size_t before = boids.size();
    boids.erase(std::remove_if(boids.begin(), boids.end(),
                               [&](const Boid& b) {
                                   return worldDist2(b.pos, center, params) <= r2;
                               }),
                boids.end());
    const int removed = (int)(before - boids.size());
    if (removed > 0) {
        for (Boid& b : boids) b.leader = false;
        leaderTimer = 1e9f;
    }
    return removed;
}

void Sim::setCount(int n) {
    n = std::max(0, std::min(n, kBoidMax));
    const int cur = (int)boids.size();
    if (n > cur)      addBoids(n - cur);
    else if (n < cur) removeBoids(cur - n);
}

void Sim::setLeaderCount(int n) {
    leaderCount = std::max(0, std::min(n, std::min(kLeaderMax, (int)boids.size())));
    if (!autoLeaders) {
        for (size_t i = 0; i < boids.size(); ++i)
            boids[i].leader = (int)i < leaderCount;
    }
    leaderTimer = 1e9f;   // force a fitness refresh on the next step
}

const Target* Sim::nearestTargetTo(Vector2 p) const {
    const Target* best = nullptr;
    float bd = 1e30f;
    for (const Target& t : targets) {
        if (!t.active) continue;
        const float d2 = worldDist2(t.pos, p, params);
        if (d2 < bd) { bd = d2; best = &t; }
    }
    return best;
}

void Sim::updateLeaders() {
    if (leaderCount <= 0) {
        for (Boid& b : boids) b.leader = false;
        leaderIdx.clear();
        return;
    }
    if (!autoLeaders) {
        for (size_t i = 0; i < boids.size(); ++i)
            boids[i].leader = (int)i < leaderCount;
        return;
    }

    Vector2 centroid{0, 0};
    for (const Boid& b : boids) centroid = Vector2Add(centroid, b.pos);
    if (!boids.empty()) centroid = Vector2Scale(centroid, 1.0f / (float)boids.size());

    std::vector<std::pair<float, int>> score;
    score.reserve(boids.size());
    for (size_t i = 0; i < boids.size(); ++i) {
        const Target* t = nearestTargetTo(boids[i].pos);
        const Vector2 anchor = t ? t->pos : centroid;
        score.push_back({worldDist2(boids[i].pos, anchor, params), (int)i});
    }
    const int k = std::min(leaderCount, (int)score.size());
    std::partial_sort(score.begin(), score.begin() + k, score.end());

    leaderIdx.clear();
    for (Boid& b : boids) b.leader = false;
    for (int i = 0; i < k; ++i) {
        boids[(size_t)score[(size_t)i].second].leader = true;
        leaderIdx.push_back(score[(size_t)i].second);
    }
}

// -------------------------------------------------------------- spatial grid

void Sim::buildGrid() {
    // Fine grid: sized by the local rules only; the (long-range) follow rule
    // is handled by scanning the leaders directly in gatherNeighbors.
    const float maxR = std::max(std::max(params.separationRadius, params.alignmentRadius),
                                params.cohesionRadius);
    grid.cell = std::max(16.0f, maxR);
    // No padding column/row: in wrapped mode the lookup indices wrap modulo
    // these counts, so every boid has a full 3x3 neighbourhood.
    grid.cols = std::max(1, (int)std::ceil((float)kScreenWidth / grid.cell));
    grid.rows = std::max(1, (int)std::ceil((float)kScreenHeight / grid.cell));
    const int cells = grid.cols * grid.rows;

    grid.counts.assign((size_t)cells, 0);
    grid.starts.assign((size_t)cells + 1, 0);
    grid.items.assign(boids.size(), 0);

    auto cellOf = [&](Vector2 p) {
        const int cx = std::max(0, std::min(grid.cols - 1, (int)(p.x / grid.cell)));
        const int cy = std::max(0, std::min(grid.rows - 1, (int)(p.y / grid.cell)));
        return cy * grid.cols + cx;
    };
    for (const Boid& b : boids) ++grid.counts[(size_t)cellOf(b.pos)];
    for (int c = 0; c < cells; ++c) grid.starts[(size_t)c + 1] = grid.starts[(size_t)c] + grid.counts[(size_t)c];
    std::vector<int> cursor(grid.starts.begin(), grid.starts.end() - 1);
    for (size_t i = 0; i < boids.size(); ++i) {
        const int c = cellOf(boids[i].pos);
        grid.items[(size_t)cursor[(size_t)c]++] = (int)i;
    }
}

// Hot path: called once per neighbour pair, so the common "occlusion off" case
// is filtered by the caller (see `bool occl = params.occlusion && !obstacles
// .empty()` in gatherNeighbors) and this function assumes it is enabled.
// Cost is O(obstacles) per neighbour pair. Measured with --bench 300 at 1200
// boids: baseline 3.38 ms/frame, occlusion on 4.00, topological 4.35,
// both 5.07. So occlusion costs ~18% and topo ~29% at the default cap of
// kObstacleMax = 12. Raising kObstacleMax or kBoidMax materially should be
// re-measured; a per-cell visibility cache is the next step if it ever matters.
bool Sim::occluded(Vector2 a, Vector2 b) const {
    if (obstacles.empty()) return false;
    const Vector2 ab = Vector2Subtract(b, a);
    const float len2 = Vector2DotProduct(ab, ab);
    if (len2 <= 1e-6f) return false;
    for (const Obstacle& o : obstacles) {
        const Vector2 ao = Vector2Subtract(o.pos, a);
        const float t = Clamp(Vector2DotProduct(ao, ab) / len2, 0.0f, 1.0f);
        const Vector2 closest = Vector2Add(a, Vector2Scale(ab, t));
        if (Vector2DistanceSqr(closest, o.pos) < o.radius * o.radius) return true;
    }
    return false;
}

Sim::NeighborData Sim::gatherNeighbors(const Boid& b, size_t self, const Context& ctx) const {
    NeighborData nb;
    const Params& p = params;

    const float sepR2 = p.separationRadius * p.separationRadius;
    const float aliR2 = p.alignmentRadius * p.alignmentRadius;
    const float cohR2 = p.cohesionRadius * p.cohesionRadius;
    const float maxR = std::sqrt(std::max(std::max(sepR2, aliR2), cohR2));
    const Vector2 h = b.heading();

    // Topological mode replaces the alignment/cohesion radius test with the K
    // nearest neighbours (Ballerini et al. 2008), so candidates are collected
    // first and only the best K influence alignment and cohesion.
    const int topoK = p.topological ? std::max(1, std::min(p.topoK, kTopoKMax)) : 0;
    // Hoisted out of occluded(): the disabled case must not pay a call per pair.
    const bool occl = p.occlusion && !obstacles.empty();
    std::vector<std::pair<float, int>> near;
    if (topoK > 0) near.reserve(64);

    auto visit = [&](const Boid& o, size_t j) {
        if (j == self) return;
        const Vector2 d = wrapDelta(Vector2Subtract(o.pos, b.pos), p);
        const float dist2 = Vector2DotProduct(d, d);
        if (dist2 > maxR * maxR) return;

        // Separation is a purely local rule and must not depend on which
        // alignment/cohesion mode is active, so it runs before the topological
        // early-return. Skipping it made boids overlap in topo mode.
        if (dist2 > 0.0f && dist2 < sepR2) {
            nb.sepSum = Vector2Add(nb.sepSum, Vector2Scale(d, -1.0f / dist2));
            ++nb.sepCount;
        }
        if (topoK > 0) { near.push_back({dist2, (int)j}); return; }

        const float dist = std::sqrt(dist2);
        const bool fov = inFov(h, d, p, dist);
        // Obstacles block sight when occlusion is on: a neighbour behind a rock
        // cannot be followed, which is what makes hiding behind obstacles real.
        if (occl && fov && occluded(b.pos, o.pos)) return;
        // Distance-weighted modes scale each contribution by 1/(1+d) so near
        // neighbours dominate instead of every neighbour counting equally.
        const float w = p.distanceWeighted ? 1.0f / (1.0f + dist) : 1.0f;

        const int sp = (o.species >= 0 && o.species < kSpeciesMax) ? o.species : 0;
        if (dist2 < aliR2 && fov) {
            nb.aliSum = Vector2Add(nb.aliSum, Vector2Scale(o.vel, w));
            nb.aliSumS[sp] = Vector2Add(nb.aliSumS[sp], Vector2Scale(o.vel, w));
            nb.aliW  += w;
            nb.aliWS[sp] += w;
            ++nb.aliCount;
            ++nb.aliCountS[sp];
        }
        if (dist2 < cohR2 && fov) {
            nb.cohSum = Vector2Add(nb.cohSum, Vector2Scale(d, w));
            nb.cohSumS[sp] = Vector2Add(nb.cohSumS[sp], Vector2Scale(d, w));
            nb.cohW  += w;
            nb.cohWS[sp] += w;
            ++nb.cohCount;
            ++nb.cohCountS[sp];
        }
    };

    if (grid.cols <= 0) {
        for (size_t j = 0; j < boids.size(); ++j) visit(boids[j], j);
    } else {

    const int cx0 = (int)std::floor((b.pos.x - maxR) / grid.cell);
    const int cx1 = (int)std::floor((b.pos.x + maxR) / grid.cell);
    const int cy0 = (int)std::floor((b.pos.y - maxR) / grid.cell);
    const int cy1 = (int)std::floor((b.pos.y + maxR) / grid.cell);
    auto cellIndex = [&](int cx, int cy) -> long {
        if (p.periodicBounds) {
            cx = ((cx % grid.cols) + grid.cols) % grid.cols;
            cy = ((cy % grid.rows) + grid.rows) % grid.rows;
        } else {
            if (cx < 0 || cy < 0 || cx >= grid.cols || cy >= grid.rows) return -1;
        }
        return (long)(cy * grid.cols + cx);
    };
    for (int cy = cy0; cy <= cy1; ++cy) {
        for (int cx = cx0; cx <= cx1; ++cx) {
            const long ci = cellIndex(cx, cy);
            if (ci < 0) continue;
            const size_t c = (size_t)ci;
            for (int k = grid.starts[c]; k < grid.starts[c + 1]; ++k) {
                const int j = grid.items[(size_t)k];
                visit(boids[(size_t)j], (size_t)j);
            }
        }
    }
    }

    if (topoK > 0 && !near.empty()) {
        const int k = std::min(topoK, (int)near.size());
        std::partial_sort(near.begin(), near.begin() + k, near.end());
        for (int i = 0; i < k; ++i) {
            const Boid& o = boids[(size_t)near[(size_t)i].second];
            const Vector2 d = wrapDelta(Vector2Subtract(o.pos, b.pos), p);
            const float dist = std::sqrt(near[(size_t)i].first);
            if (!inFov(h, d, p, dist)) continue;
            if (occl && occluded(b.pos, o.pos)) continue;
            const float w = p.distanceWeighted ? 1.0f / (1.0f + dist) : 1.0f;
            const int sp = (o.species >= 0 && o.species < kSpeciesMax) ? o.species : 0;
            nb.aliSum = Vector2Add(nb.aliSum, Vector2Scale(o.vel, w));
            nb.aliSumS[sp] = Vector2Add(nb.aliSumS[sp], Vector2Scale(o.vel, w));
            nb.aliW  += w;
            nb.aliWS[sp] += w;
            ++nb.aliCount;
            ++nb.aliCountS[sp];
            nb.cohSum = Vector2Add(nb.cohSum, Vector2Scale(d, w));
            nb.cohSumS[sp] = Vector2Add(nb.cohSumS[sp], Vector2Scale(d, w));
            nb.cohW  += w;
            nb.cohWS[sp] += w;
            ++nb.cohCount;
            ++nb.cohCountS[sp];
        }
    }

    // Follow rule: scan the (few) leaders directly - no need for a coarse grid.
    const float folR2 = p.followRadius * p.followRadius;
    for (const Boid* o : ctx.leadersPtr) {
        const Vector2 d = wrapDelta(Vector2Subtract(o->pos, b.pos), p);
        const float dist2 = Vector2DotProduct(d, d);
        if (dist2 >= folR2) continue;
        if (!inFov(h, d, p, std::sqrt(dist2))) continue;
        if (occl && occluded(b.pos, o->pos)) continue;
        nb.folSum = Vector2Add(nb.folSum, d);
        ++nb.folCount;
    }
    return nb;
}

Sim::Context Sim::buildContext() const {
    Context ctx;
    for (const Boid& b : boids) {
        if (!b.leader) continue;
        ctx.leaderCenter = Vector2Add(ctx.leaderCenter, b.pos);
        ctx.leaderVel = Vector2Add(ctx.leaderVel, b.vel);
        ctx.leadersPtr.push_back(&b);
        ++ctx.leaders;
    }
    if (ctx.leaders > 0) {
        ctx.leaderCenter = Vector2Scale(ctx.leaderCenter, 1.0f / (float)ctx.leaders);
        ctx.leaderVel = Vector2Scale(ctx.leaderVel, 1.0f / (float)ctx.leaders);
    }
    for (const Predator& p : predators)
        if (p.active) ctx.predators.push_back(&p);
    return ctx;
}

// --------------------------------------------------------------- force model

void Sim::accumulateForces(const Boid& b, const NeighborData& nb, const Context& ctx,
                           bool pointerActive, Vector2 pointer, Vector2 pointerVel,
                           ForceBreakdown& out) const {
    const Params& p = params;
    out = ForceBreakdown{};

    // Shared, prioritised acceleration budget: each behaviour is applied in
    // priority order against what is left, so low-priority rules degrade
    // gracefully instead of drowning out survival behaviour.
    float remaining = std::max(0.0f, p.forceBudget) * p.maxForce;
    auto add = [&](Vector2& slot, Vector2 steer, float weight) {
        slot = Vector2{0, 0};
        if (weight <= 0.0f || remaining <= 0.0f) return;
        Vector2 f = Vector2Scale(steer, weight);
        const float len = Vector2Length(f);
        if (len <= 0.0f) return;
        if (len > remaining) f = Vector2Scale(f, remaining / len);
        slot = f;
        remaining -= Vector2Length(f);
        out.total = Vector2Add(out.total, f);
    };

    // 1. Predator avoidance: radial escape, tangential slide, panic burst.
    {
        Vector2 steer{0, 0};
        for (const Predator* pr : ctx.predators) {
            const Vector2 toB = wrapDelta(Vector2Subtract(b.pos, pr->pos), p);
            const float dist = Vector2Length(toB);
            if (dist <= 0.0f || dist >= p.predatorRadius) continue;
            const float t = 1.0f - dist / p.predatorRadius;
            const Vector2 away = Vector2Scale(toB, 1.0f / dist);
            const Vector2 tang = sideAway(b, pr->pos);
            const Vector2 desired = Vector2Add(away, Vector2Scale(tang, 0.6f));
            steer = Vector2Add(steer, Vector2Scale(
                steerTowards(b, desired, p, p.maxSpeed * 1.25f), 0.4f + t));
            steer = Vector2Add(steer, Vector2Scale(b.vel, 0.9f * t));
        }
        add(out.predator, limitVec(steer, p.maxForce), p.predator);
    }

    // 2. Obstacle avoidance: radial push plus the real tangent length.
    if (p.obstacle > 0.0f && !obstacles.empty()) {
        Vector2 steer{0, 0};
        for (const Obstacle& o : obstacles) {
            const Vector2 toB = wrapDelta(Vector2Subtract(b.pos, o.pos), p);
            const float dist = Vector2Length(toB);
            const float reach = o.radius + p.obstacleLook;
            if (dist >= reach || dist == 0.0f) continue;
            const float t = 1.0f - dist / reach;
            const Vector2 away = Vector2Scale(toB, 1.0f / dist);
            const Vector2 tang = sideAway(b, o.pos);
            const float ratio = tangentRatio(dist, o.radius);
            const Vector2 desired = Vector2Add(Vector2Scale(away, 1.0f),
                                               Vector2Scale(tang, 0.8f * ratio));
            steer = Vector2Add(steer, Vector2Scale(steerTowards(b, desired, p),
                                                   (0.35f + t) / (float)obstacles.size()));
        }
        add(out.obstacle, limitVec(steer, p.maxForce), p.obstacle);
    }

    // 3. Separation.
    if (nb.sepCount > 0)
        add(out.separation,
            steerTowards(b, Vector2Scale(nb.sepSum, 1.0f / (float)nb.sepCount), p), p.separation);

    // 4. Alignment.
    // Multi-flock: alignment/cohesion prefer the boid's own species. With
    // crossSpecies > 0 a fraction of the other species is blended back in.
    const int   sp    = (b.species >= 0 && b.species < kSpeciesMax) ? b.species : 0;
    const bool  mixed = p.multiSpecies;
    const float cross = Clamp(p.crossSpecies, 0.0f, 1.0f);
    auto blend = [&](const Vector2* all, const Vector2* byS) {
        if (!mixed) return *all;
        Vector2 v = byS[sp];
        if (cross > 0.0f)
            v = Vector2Add(v, Vector2Scale(Vector2Subtract(*all, byS[sp]), cross));
        return v;
    };
    // The normalisation denominator must follow the same rule as the sums, or
    // a segregated flock would divide by every boid in sight.
    auto blendW = [&](float allW, const float* wS) {
        if (!mixed) return allW;
        const float own = wS[sp];
        return cross > 0.0f ? own + (allW - own) * cross : own;
    };
    auto blendN = [&](int allN, const int* nS) {
        if (!mixed) return allN;
        const int own = nS[sp];
        return cross > 0.0f ? own + (int)((float)(allN - own) * cross) : own;
    };

    const float aliWall = blendW(nb.aliW, nb.aliWS);
    const int   aliNall = blendN(nb.aliCount, nb.aliCountS);
    if (nb.aliCount > 0 && aliNall > 0) {
        const float denom = (p.distanceWeighted || p.topological) && aliWall > 0.0f
            ? aliWall : (float)aliNall;
        add(out.alignment,
            steerTowards(b, Vector2Scale(blend(&nb.aliSum, nb.aliSumS), 1.0f / denom), p),
            p.alignment);
    }

    // 5. Cohesion.
    const float cohWall = blendW(nb.cohW, nb.cohWS);
    const int   cohNall = blendN(nb.cohCount, nb.cohCountS);
    if (nb.cohCount > 0 && cohNall > 0) {
        // cohSum holds wrapped offsets, so the centroid is already relative.
        const float denom = (p.distanceWeighted || p.topological) && cohWall > 0.0f
            ? cohWall : (float)cohNall;
        add(out.cohesion,
            steerTowards(b, Vector2Scale(blend(&nb.cohSum, nb.cohSumS), 1.0f / denom), p),
            p.cohesion);
    }

    // 6. Target seeking (leaders only, nearest active target).
    // Leaders seek objectives at full strength; followers are pulled weakly so
    // a flock with no leaders can still forage, and so the objective reaches
    // the whole flock rather than only its front runners.
    const float seek = b.leader ? 1.0f : Clamp(p.foodPull, 0.0f, 1.0f);
    if (p.target > 0.0f && seek > 0.0f) {
        if (const Target* t = nearestTargetTo(b.pos)) {
            const Vector2 toT = wrapDelta(Vector2Subtract(t->pos, b.pos), p);
            if (Vector2Length(toT) > p.targetRadius * 0.25f)
                add(out.target, steerTowards(b, toT, p), p.target * seek);
        }
    }

    // 7. Leadership / following.
    if (p.follow > 0.0f) {
        if (!b.leader && nb.folCount > 0) {
            const Vector2 c = Vector2Scale(nb.folSum, 1.0f / (float)nb.folCount);
            add(out.follow, steerTowards(b, c, p), p.follow);   // folSum is already relative
        } else if (b.leader && ctx.leaders > 0) {
            Vector2 steer = Vector2Scale(
                steerTowards(b, Vector2Subtract(ctx.leaderCenter, b.pos), p), 0.35f);
            if (Vector2Length(ctx.leaderVel) > 0.0f)
                steer = Vector2Add(steer, Vector2Scale(steerTowards(b, ctx.leaderVel, p), 0.4f));
            add(out.follow, limitVec(steer, p.maxForce), p.follow);
        }
    }

    // 8. Boundary avoidance (skipped entirely in the wrapped world).
    if (!p.periodicBounds) {
        Vector2 turn{0, 0};
        const float m = p.boundsMargin;
        if (b.pos.x < m)                 turn.x += 1.0f - b.pos.x / m;
        if (b.pos.x > kScreenWidth - m)  turn.x -= 1.0f - ((float)kScreenWidth - b.pos.x) / m;
        if (b.pos.y < m)                 turn.y += 1.0f - b.pos.y / m;
        if (b.pos.y > kScreenHeight - m) turn.y -= 1.0f - ((float)kScreenHeight - b.pos.y) / m;
        if (turn.x != 0.0f || turn.y != 0.0f)
            add(out.bounds, steerTowards(b, turn, p), p.boundsWeight);
    }

    // 9. Mouse nudge (lowest priority).
    if (pointerActive) {
        const Vector2 toB = wrapDelta(Vector2Subtract(b.pos, pointer), p);
        const float dist = Vector2Length(toB);
        if (dist > 0.0f && dist < p.mouseRadius) {
            const float falloff = 1.0f - dist / p.mouseRadius;
            const Vector2 away = Vector2Scale(toB, 1.0f / dist);
            Vector2 steer = Vector2Scale(away, p.mousePush * falloff * falloff * p.maxForce);
            steer = Vector2Add(steer, Vector2Scale(pointerVel, 0.55f * falloff));
            add(out.mouse, limitVec(steer, p.maxForce), 1.0f);
        }
    }

    // 10. Global wind / flow field (absolute lowest priority: survival and
    // flocking always spend the budget first, so this only shapes the drift).
    if (windEnabled && p.windStrength > 0.0f) {
        const Vector2 w = windAt(b.pos, simTime, p);
        add(out.wind, limitVec(w, p.maxForce), 1.0f);
    }
}

// ------------------------------------------------------------------ predators

void Sim::updatePredator(Predator& pr, float dt) {
    const Params& p = params;

    pr.timer -= dt;
    if (pr.timer <= 0.0f) {
        pr.hunting = !pr.hunting;
        pr.timer = std::max(0.1f, pr.hunting ? p.predatorHunt : p.predatorRest);
    }

    Vector2 targetVel = pr.vel;
    if (pr.hunting) {
        const Boid* prey = nullptr;
        float best = 1e30f;
        for (const Boid& b : boids) {
            const float d2 = worldDist2(b.pos, pr.pos, p);
            if (d2 < best) { best = d2; prey = &b; }
        }
        if (prey) {
            const Vector2 toPrey = wrapDelta(Vector2Subtract(prey->pos, pr.pos), p);
            if (Vector2Length(toPrey) > 0.0f)
                targetVel = Vector2Scale(Vector2Normalize(toPrey), p.predatorSpeed);
        }
        targetVel = Vector2Rotate(targetVel, randRange(-0.6f, 0.6f) * dt * 4.0f);
    } else {
        // Retreat from the flock while stamina recovers.
        Vector2 centroid{0, 0};
        for (const Boid& b : boids) centroid = Vector2Add(centroid, b.pos);
        if (!boids.empty()) centroid = Vector2Scale(centroid, 1.0f / (float)boids.size());
        const Vector2 away = Vector2Subtract(pr.pos, centroid);
        if (Vector2Length(away) > 1.0f) {
            targetVel = Vector2Scale(Vector2Normalize(away), p.predatorSpeed * 0.55f);
            targetVel = Vector2Rotate(targetVel, randRange(-1.0f, 1.0f) * dt);
        }
    }

    const float rate = p.predatorSpeed > 0.0f
        ? std::min(1.0f, p.predatorForce / p.predatorSpeed * dt) : 1.0f;
    pr.vel = Vector2Add(pr.vel, Vector2Scale(Vector2Subtract(targetVel, pr.vel), rate));
    const float sp = Vector2Length(pr.vel);
    const float cap = p.predatorSpeed * (pr.hunting ? 1.0f : 0.6f);
    if (sp > cap && sp > 0.0f) pr.vel = Vector2Scale(pr.vel, cap / sp);
    pr.pos = Vector2Add(pr.pos, Vector2Scale(pr.vel, dt));

    if (p.periodicBounds) {
        if (pr.pos.x < 0.0f)           pr.pos.x += (float)kScreenWidth;
        if (pr.pos.x >= kScreenWidth)  pr.pos.x -= (float)kScreenWidth;
        if (pr.pos.y < 0.0f)           pr.pos.y += (float)kScreenHeight;
        if (pr.pos.y >= kScreenHeight) pr.pos.y -= (float)kScreenHeight;
    } else if (p.hardWalls) {
        if (pr.pos.x < 0 || pr.pos.x > kScreenWidth)  pr.vel.x *= -1;
        if (pr.pos.y < 0 || pr.pos.y > kScreenHeight) pr.vel.y *= -1;
        pr.pos.x = Clamp(pr.pos.x, 0.0f, (float)kScreenWidth);
        pr.pos.y = Clamp(pr.pos.y, 0.0f, (float)kScreenHeight);
    } else {
        pr.pos.x = Clamp(pr.pos.x, 0.0f, (float)kScreenWidth);
        pr.pos.y = Clamp(pr.pos.y, 0.0f, (float)kScreenHeight);
    }
}

// Foraging: every boid within foodRadius of an active target eats from it. The
// target shrinks as it is consumed, and once empty it deactivates (and is
// optionally replaced elsewhere) so the flock must choose a new objective.
// Leadership is re-scored because it keys off the nearest target.
// One dial for predator lethality. Rationale: at default settings a single
// predator almost never catches anything, so the predation demo cannot show a
// population cycle. This maps [0,1] onto the three parameters that actually
// matter. It OVERRIDES them (see Params::huntPressure) - disable by setting the
// pressure below zero and the individual sliders take over again.
void Sim::setHuntPressure(bool on) {
    if (on == huntPressureActive()) return;   // already in the requested state

    if (on) {
        // Stash the manual pair so it can come back when pressure is released.
        manualPredatorSpeed = params.predatorSpeed;
        manualCatchRadius   = params.catchRadius;
        params.huntPressure = 0.35f;
    } else {
        params.huntPressure = -1.0f;
        if (manualPredatorSpeed >= 0.0f) params.predatorSpeed = manualPredatorSpeed;
        if (manualCatchRadius   >= 0.0f) params.catchRadius   = manualCatchRadius;
        manualPredatorSpeed = -1.0f;
        manualCatchRadius   = -1.0f;
    }
}

void Sim::applyHuntPressure() {
    const float h = params.huntPressure;
    if (h < 0.0f) return;
    const float t = Clamp(h, 0.0f, 1.0f);

    params.predatorSpeed = 120.0f + 200.0f * t;   // 120 .. 320 px/s
    params.catchRadius   = 10.0f + 30.0f * t;     // 10 .. 40 px

    const int want = 1 + (int)std::lround(3.0f * t);   // 1 .. 4 predators
    int active = 0;
    for (const Predator& pr : predators) if (pr.active) ++active;

    if (want > active) {
        for (int i = 0; i < want - active; ++i) {
            bool found = false;
            for (Predator& pr : predators) {
                if (pr.active) continue;
                pr.pos     = {randRange(60.0f, (float)kScreenWidth - 60.0f),
                              randRange(60.0f, (float)kScreenHeight - 60.0f)};
                pr.vel     = {params.predatorSpeed, 0.0f};
                pr.hunting = true;
                pr.timer   = params.predatorHunt;
                pr.active  = true;
                found = true;
                break;
            }
            if (!found && predators.size() < (size_t)kPredatorMax) {
                Predator pr;
                pr.pos     = {60.0f, (float)kScreenHeight * 0.5f};
                pr.vel     = {params.predatorSpeed, 0.0f};
                pr.hunting = true;
                pr.timer   = params.predatorHunt;
                pr.active  = true;
                predators.push_back(pr);
            }
        }
    } else if (want < active) {
        int toKill = active - want;
        for (int i = (int)predators.size() - 1; i >= 0 && toKill > 0; --i)
            if (predators[i].active) { predators[i].active = false; --toKill; }
    }
}

void Sim::updateFood(float dt) {
    if (!foodEnabled || boids.empty()) return;

    // Foraging needs something to eat. Targets are not persisted, and reset()
    // leaves the initial one inactive, so stock a target on first use.
    bool anyActive = false;
    for (const Target& t : targets) if (t.active && t.amount > 0.0f) anyActive = true;
    if (!anyActive) {
        Target fresh;
        fresh.pos    = {kScreenWidth * 0.5f, kScreenHeight * 0.5f};
        fresh.active = true;
        fresh.amount = 1.0f;
        if (targets.empty()) {
            targets.push_back(fresh);
        } else {
            targets[0] = fresh;
            for (size_t i = 1; i < targets.size(); ++i) targets[i].active = false;
        }
    }

    const float r2 = params.foodRadius * params.foodRadius;
    bool anyEaten = false;

    for (Target& t : targets) {
        if (!t.active) continue;
        int eaters = 0;
        for (const Boid& b : boids)
            if (worldDist2(b.pos, t.pos, params) <= r2) ++eaters;
        if (eaters == 0) continue;

        t.amount -= params.foodDrain * (float)eaters * dt;
        if (t.amount > 0.0f) continue;

        t.amount = 0.0f;
        t.active = false;
        ++targetsEaten;
        anyEaten = true;

        if (!foodRespawn) continue;
        // Respawn away from the flock so reaching it is a real trip.
        Target fresh;
        for (int tries = 0; tries < 12; ++tries) {
            fresh.pos = {randRange(60.0f, (float)kScreenWidth - 60.0f),
                         randRange(60.0f, (float)kScreenHeight - 60.0f)};
            float nearest = 1e30f;
            for (const Boid& b : boids)
                nearest = std::min(nearest, worldDist2(b.pos, fresh.pos, params));
            if (nearest > 200.0f * 200.0f) break;
        }
        fresh.amount = 1.0f;
        fresh.active = true;

        // Reuse the slot we just emptied (or any other inactive one) rather
        // than letting the vector grow without bound.
        bool reused = false;
        for (Target& slot : targets) {
            if (!slot.active) { slot = fresh; reused = true; break; }
        }
        if (!reused && targets.size() < (size_t)kTargetMax) targets.push_back(fresh);
    }

    if (anyEaten) leaderTimer = 1e9f;   // objectives changed: re-score leadership
}

void Sim::updatePredators(float dt) {
    for (Predator& pr : predators)
        if (pr.active) updatePredator(pr, dt);
}

// Lethal predation: a hunting predator eats one boid within catchRadius, then
// stops hunting and digests for predatorRest seconds (it retreats meanwhile).
// At most one kill per predator per step keeps the cycle readable and stops a
// predator parked inside the flock from vacuuming up every boid in a few frames.
void Sim::resolvePredation() {
    if (!kills || predators.empty() || boids.empty()) return;

    const float r2 = params.catchRadius * params.catchRadius;
    bool removedAny = false;

    for (Predator& pr : predators) {
        if (!pr.active || !pr.hunting) continue;

        size_t best = boids.size();
        float  bestD2 = r2;
        for (size_t i = 0; i < boids.size(); ++i) {
            const float d2 = worldDist2(boids[i].pos, pr.pos, params);
            if (d2 <= bestD2) { bestD2 = d2; best = i; }
        }
        if (best == boids.size()) continue;

        boids.erase(boids.begin() + (long)best);
        removedAny = true;
        ++pr.kills;
        ++killCount;

        pr.hunting = false;
        pr.timer   = std::max(0.1f, params.predatorRest);
    }

    if (removedAny) updateLeaders();
}

// ---------------------------------------------------------------------- stats

void Sim::updateStats() {
    // Reset only the scalar readouts. Assigning Stats{} would also wipe the
    // rolling chart history (hist / histCount / histHead) every single frame.
    stats.avgSpeed = 0.0f;
    stats.order    = 0.0f;
    stats.nnDist   = 0.0f;
    if (boids.empty()) return;

    Vector2 headingSum{0, 0};
    float speedSum = 0.0f;
    float nnSum = 0.0f;
    int nnCount = 0;

    // Fine radius for the nearest-neighbour probe (matches the grid cells);
    // isolated boids fall back to a full scan, which only happens when the
    // flock is sparse enough that the grid pass was already cheap.
    const float maxR = std::max(std::max(params.separationRadius, params.alignmentRadius),
                                params.cohesionRadius);

    for (size_t i = 0; i < boids.size(); ++i) {
        const Boid& b = boids[i];
        headingSum = Vector2Add(headingSum, b.heading());
        speedSum += Vector2Length(b.vel);

        float best = 1e30f;
        if (grid.cols > 0) {
            const int cx0 = std::max(0, std::min(grid.cols - 1, (int)((b.pos.x - maxR) / grid.cell)));
            const int cx1 = std::max(0, std::min(grid.cols - 1, (int)((b.pos.x + maxR) / grid.cell)));
            const int cy0 = std::max(0, std::min(grid.rows - 1, (int)((b.pos.y - maxR) / grid.cell)));
            const int cy1 = std::max(0, std::min(grid.rows - 1, (int)((b.pos.y + maxR) / grid.cell)));
            for (int cy = cy0; cy <= cy1; ++cy) {
                for (int cx = cx0; cx <= cx1; ++cx) {
                    const size_t c = (size_t)(cy * grid.cols + cx);
                    for (int k = grid.starts[c]; k < grid.starts[c + 1]; ++k) {
                        const int j = grid.items[(size_t)k];
                        if ((size_t)j == i) continue;
                        const float d = worldDist2(b.pos, boids[(size_t)j].pos, params);
                        if (d < best) best = d;
                    }
                }
            }
        }
        if (best < 1e29f) {
            nnSum += std::sqrt(best);
            ++nnCount;
        } else {
            for (size_t j = 0; j < boids.size(); ++j) {
                if (j == i) continue;
                const float d = worldDist2(b.pos, boids[j].pos, params);
                if (d < best) best = d;
            }
            if (best < 1e29f) {
                nnSum += std::sqrt(best);
                ++nnCount;
            }
        }
    }

    stats.avgSpeed = speedSum / (float)boids.size();
    stats.order = Vector2Length(headingSum) / (float)boids.size();
    stats.nnDist = nnCount > 0 ? nnSum / (float)nnCount : 0.0f;
}

// ----------------------------------------------------------------------- step

void Sim::step(float dt, bool pointerActive, Vector2 pointer, Vector2 pointerVel) {
    if (dt <= 0.0f) return;

    const Params& p = params;

    simTime += dt;

    applyHuntPressure();
    updatePredators(dt);
    resolvePredation();
    updateFood(dt);

    leaderTimer += dt;
    if (autoLeaders && leaderTimer >= 1.0f) {
        leaderTimer = 0.0f;
        updateLeaders();
    }

    buildGrid();
    const Context ctx = buildContext();

    const size_t n = boids.size();
    accel.assign(n, Vector2{0, 0});

    // Force evaluation is a pure function of (boid, neighbours, context), so it
    // parallelises cleanly: each iteration writes only accel[i] and owns its
    // own ForceBreakdown. The RNG is never touched in here (it is not
    // thread-safe); if that ever changes this must stay serial.
    //
    // NOTE: `fb` used to be declared outside this loop and reused. Under
    // par_unseq that is a data race, so it is now per-iteration.
#ifdef BOIDS_PARALLEL
    std::for_each(std::execution::par_unseq,
                  accel.begin(), accel.end(),
                  [&](Vector2& out) {
                      const size_t i = (size_t)(&out - accel.data());
                      ForceBreakdown fb;
                      const NeighborData nb = gatherNeighbors(boids[i], i, ctx);
                      accumulateForces(boids[i], nb, ctx, pointerActive, pointer,
                                       pointerVel, fb);
                      out = fb.total;
                  });
#else
    for (size_t i = 0; i < n; ++i) {
        const Boid& b = boids[i];
        ForceBreakdown fb;
        const NeighborData nb = gatherNeighbors(b, i, ctx);
        accumulateForces(b, nb, ctx, pointerActive, pointer, pointerVel, fb);
        accel[i] = fb.total;
    }
#endif

    for (size_t i = 0; i < n; ++i) {
        Boid& b = boids[i];
        const float maxSpeed = boidMaxSpeed(b, p);

        b.vel = Vector2Add(b.vel, Vector2Scale(accel[i], dt));

        const float speed = Vector2Length(b.vel);
        if (speed > maxSpeed)                        b.vel = Vector2Scale(b.vel, maxSpeed / speed);
        else if (speed < p.minSpeed && speed > 0.0f) b.vel = Vector2Scale(b.vel, p.minSpeed / speed);

        b.pos = Vector2Add(b.pos, Vector2Scale(b.vel, dt));

        if (p.periodicBounds) {
            // Toroidal world: cross a seam and come out the other side.
            if (b.pos.x < 0.0f)            b.pos.x += (float)kScreenWidth;
            if (b.pos.x >= kScreenWidth)   b.pos.x -= (float)kScreenWidth;
            if (b.pos.y < 0.0f)            b.pos.y += (float)kScreenHeight;
            if (b.pos.y >= kScreenHeight)  b.pos.y -= (float)kScreenHeight;
        } else if (p.hardWalls) {
            // Hard containment: bounce off the walls rather than wrap.
            if (b.pos.x < 0)             { b.pos.x = 0;             b.vel.x = std::fabs(b.vel.x); }
            if (b.pos.x > kScreenWidth)  { b.pos.x = kScreenWidth;  b.vel.x = -std::fabs(b.vel.x); }
            if (b.pos.y < 0)             { b.pos.y = 0;             b.vel.y = std::fabs(b.vel.y); }
            if (b.pos.y > kScreenHeight) { b.pos.y = kScreenHeight; b.vel.y = -std::fabs(b.vel.y); }
        }

        // Stamina: drains with speed and predator pressure, recovers when slow.
        if (p.energyEnabled) {
            float rate = (speed > 0.75f * maxSpeed) ? -p.energyDrain : p.energyRecover;
            for (const Predator* pr : ctx.predators) {
                if (Vector2DistanceSqr(b.pos, pr->pos) <
                    p.predatorRadius * p.predatorRadius) { rate -= p.energyPanicDrain; break; }
            }
            b.energy = Clamp(b.energy + rate * dt, 0.0f, 1.0f);
        } else {
            b.energy = 1.0f;
        }
    }

    updateStats();

    // Rolling chart history: subsampled so the window spans ~60 s, not ~10 s.
    if (++stats.sampleTick < Stats::kSampleEvery) return;
    stats.sampleTick = 0;

    Stats::Sample s;
    s.order    = stats.order;
    s.avgSpeed = stats.avgSpeed;
    s.nnDist   = stats.nnDist;
    s.alive    = (int)boids.size();
    s.kills    = killCount;
    if (params.multiSpecies) {
        for (const Boid& b : boids)
            if (b.species >= 0 && b.species < kSpeciesMax) ++s.species[b.species];
    }
    stats.pushSample(s);
}

// ----------------------------------------------------------------------- draw

namespace {

Color targetColor(int i) {
    static const Color palette[4] = {
        {120, 255, 170, 255}, {130, 200, 255, 255}, {255, 210, 120, 255}, {230, 160, 255, 255}};
    return palette[i % 4];
}

void drawBoidShape(const Boid& b, const Params& p) {
    const Vector2 dir = b.heading();
    const Vector2 side{-dir.y, dir.x};
    const float size = b.leader ? 11.0f : 8.0f;
    const Vector2 tip   = Vector2Add(b.pos, Vector2Scale(dir, size));
    const Vector2 base  = Vector2Add(b.pos, Vector2Scale(dir, -size * 0.62f));
    const Vector2 left  = Vector2Add(base, Vector2Scale(side,  size * 0.38f));
    const Vector2 right = Vector2Add(base, Vector2Scale(side, -size * 0.38f));

    Color c;
    if (b.leader) {
        c = Color{255, 205, 90, 255};   // leaders stand out in gold
    } else if (p.multiSpecies && b.species > 0 && b.species < kSpeciesMax) {
        // Each extra flock gets its own distinct hue, brightened with speed so
        // it still reads as one organism in motion.
        const Color hue = speciesColor(b.species);
        const float t = Clamp(Vector2Length(b.vel) / std::max(1.0f, p.maxSpeed), 0.0f, 1.0f);
        const float k = 0.55f + 0.45f * t;
        c = Color{(unsigned char)Clamp(hue.r * k, 0.0f, 255.0f),
                  (unsigned char)Clamp(hue.g * k, 0.0f, 255.0f),
                  (unsigned char)Clamp(hue.b * k, 0.0f, 255.0f), 255};
        if (p.energyEnabled) {
            const float e = Clamp(b.energy, 0.0f, 1.0f);
            const Color spent{90, 90, 110, 255};
            c = Color{(unsigned char)(spent.r + (c.r - spent.r) * e),
                      (unsigned char)(spent.g + (c.g - spent.g) * e),
                      (unsigned char)(spent.b + (c.b - spent.b) * e), 255};
        }
    } else {
        const float t = Clamp(Vector2Length(b.vel) / std::max(1.0f, p.maxSpeed), 0.0f, 1.0f);
        c = Color{(unsigned char)(40 + 200 * t),
                  (unsigned char)(190 - 120 * t),
                  (unsigned char)(220 - 60 * t), 255};
        if (p.energyEnabled) {
            // Tired boids fade toward a cold grey so exhaustion is visible.
            const float e = Clamp(b.energy, 0.0f, 1.0f);
            const Color spent{90, 90, 110, 255};
            c = Color{(unsigned char)(spent.r + (c.r - spent.r) * e),
                      (unsigned char)(spent.g + (c.g - spent.g) * e),
                      (unsigned char)(spent.b + (c.b - spent.b) * e), 255};
        }
    }
    DrawTriangle(tip, right, left, c);
}

void drawPredatorShape(const Predator& pr, const Params& p, float catchRadius = 0.0f) {
    DrawCircleLinesV(pr.pos, p.predatorRadius, Fade(RED, pr.hunting ? 0.18f : 0.07f));
    if (catchRadius > 0.0f && pr.hunting)
        DrawCircleLinesV(pr.pos, catchRadius, Color{255, 90, 90, 255});
    const Vector2 dir = Vector2Length(pr.vel) > 0.0f ? Vector2Normalize(pr.vel) : Vector2{1, 0};
    const Vector2 side{-dir.y, dir.x};
    const Vector2 tip = Vector2Add(pr.pos, Vector2Scale(dir, 18.0f));
    const Vector2 a = Vector2Add(pr.pos, Vector2Add(Vector2Scale(dir, -12.0f), Vector2Scale(side, 11.0f)));
    const Vector2 c = Vector2Add(pr.pos, Vector2Add(Vector2Scale(dir, -12.0f), Vector2Scale(side, -11.0f)));
    DrawTriangle(tip, c, a, pr.hunting ? Color{230, 60, 60, 255} : Color{150, 90, 110, 255});
    if (!pr.hunting) DrawCircleLinesV(pr.pos, 6.0f, Fade(SKYBLUE, 0.85f));
}

}  // namespace

void Sim::draw() const {
    // Wind field background: a sparse arrow grid so the drift is visible
    // instead of the flock just mysteriously sliding sideways.
    if (windEnabled && params.windStrength > 0.0f) {
        constexpr float kStep = 120.0f;
        for (float gy = kStep * 0.5f; gy < (float)kScreenHeight; gy += kStep) {
            for (float gx = kStep * 0.5f; gx < (float)kScreenWidth; gx += kStep) {
                const Vector2 w = windAt(Vector2{gx, gy}, simTime, params);
                const float len = Vector2Length(w);
                if (len <= 1e-4f) continue;
                const Vector2 d = Vector2Scale(w, 1.0f / len);
                const float sc = std::min(22.0f, len * 0.25f);
                const Vector2 a{gx, gy};
                const Vector2 b = Vector2Add(a, Vector2Scale(d, sc));
                const Color c = Fade(Color{150, 200, 255, 255},
                                     0.18f + 0.30f * (sc / 22.0f));
                DrawLineV(a, b, c);
                const Vector2 n{-d.y, d.x};
                DrawTriangle(Vector2Add(b, Vector2Scale(d, 4.0f)),
                             Vector2Add(b, Vector2Scale(n, 3.0f)),
                             Vector2Subtract(b, Vector2Scale(n, 3.0f)), c);
            }
        }
    }

    for (const Obstacle& o : obstacles) {
        DrawCircleV(o.pos, o.radius, Color{52, 58, 74, 255});
        DrawCircleLinesV(o.pos, o.radius, Color{120, 132, 160, 255});
        DrawCircleLinesV(o.pos, o.radius + params.obstacleLook,
                         Fade(Color{120, 132, 160, 255}, 0.18f));
    }

    for (size_t i = 0; i < targets.size(); ++i) {
        if (!targets[i].active) continue;
        const Color c = targetColor((int)i);
        DrawCircleLinesV(targets[i].pos, params.targetRadius, Fade(c, 0.20f));
        DrawCircleLinesV(targets[i].pos, params.targetRadius * 0.5f, Fade(c, 0.32f));
        DrawCircleV(targets[i].pos, 8.0f, c);

        if (foodEnabled) {
            // Eating radius plus a fill that shrinks as the food is consumed.
            DrawCircleLinesV(targets[i].pos, params.foodRadius, Fade(c, 0.30f));
            const float a = Clamp(targets[i].amount, 0.0f, 1.0f);
            if (a > 0.0f) DrawCircleV(targets[i].pos, 8.0f + 10.0f * a, Fade(c, 0.55f));
            DrawRectangle((int)targets[i].pos.x - 12, (int)targets[i].pos.y - 20, 24, 4,
                          Color{30, 34, 44, 255});
            DrawRectangle((int)targets[i].pos.x - 12, (int)targets[i].pos.y - 20,
                          (int)(24.0f * a), 4, c);
        }
    }

    for (const Predator& pr : predators)
        if (pr.active)
            drawPredatorShape(pr, params, kills ? params.catchRadius : 0.0f);

    for (const Boid& b : boids) drawBoidShape(b, params);
}

void Sim::drawDebug(Vector2 mouse) const {
    if (boids.empty()) return;
    const Params& p = params;

    size_t best = 0;
    float bestD = 1e30f;
    for (size_t i = 0; i < boids.size(); ++i) {
        const float d = Vector2DistanceSqr(boids[i].pos, mouse);
        if (d < bestD) { bestD = d; best = i; }
    }
    const Boid& b = boids[best];

    DrawCircleLinesV(b.pos, p.separationRadius, Fade(Color{255, 120, 90, 255}, 0.55f));
    DrawCircleLinesV(b.pos, p.alignmentRadius, Fade(Color{110, 210, 255, 255}, 0.55f));
    DrawCircleLinesV(b.pos, p.cohesionRadius, Fade(Color{140, 235, 150, 255}, 0.55f));
    DrawCircleLinesV(b.pos, p.followRadius, Fade(Color{255, 205, 90, 255}, 0.30f));

    // Field-of-view cone around the inspected boid's heading.
    if (p.fovCos < 1.0f && p.alignmentRadius > 1.0f) {
        const Vector2 h = b.heading();
        const float ang = std::acos(Clamp(p.fovCos, -1.0f, 1.0f));
        const float base = std::atan2(h.y, h.x);
        const float r = p.alignmentRadius;
        const Vector2 a = Vector2Add(b.pos, Vector2{r * std::cos(base - ang), r * std::sin(base - ang)});
        const Vector2 c = Vector2Add(b.pos, Vector2{r * std::cos(base + ang), r * std::sin(base + ang)});
        DrawTriangle(b.pos, c, a, Fade(Color{110, 210, 255, 255}, 0.10f));
    }

    const Context ctx = buildContext();
    const NeighborData nb = gatherNeighbors(b, best, ctx);
    ForceBreakdown fb;
    accumulateForces(b, nb, ctx, false, Vector2{0, 0}, Vector2{0, 0}, fb);

    auto arrow = [&](Vector2 v, Color c) {
        if (Vector2Length(v) < 0.001f) return;
        const Vector2 tip = Vector2Add(b.pos, v);
        DrawLineV(b.pos, tip, c);
        const Vector2 d = Vector2Normalize(v);
        const Vector2 n{-d.y, d.x};
        const Vector2 back = Vector2Subtract(tip, Vector2Scale(d, 8.0f));
        DrawTriangle(tip, Vector2Add(back, Vector2Scale(n, 4.0f)),
                     Vector2Add(back, Vector2Scale(n, -4.0f)), c);
    };
    const float s = 0.28f;
    arrow(Vector2Scale(fb.separation, s), Color{255, 120, 90, 255});
    arrow(Vector2Scale(fb.alignment, s),  Color{110, 210, 255, 255});
    arrow(Vector2Scale(fb.cohesion, s),   Color{140, 235, 150, 255});
    arrow(Vector2Scale(fb.obstacle, s),   Color{200, 170, 255, 255});
    arrow(Vector2Scale(fb.predator, s),   Color{255, 90, 90, 255});
    arrow(Vector2Scale(fb.target, s),     Color{120, 255, 170, 255});
    arrow(Vector2Scale(fb.follow, s),     Color{255, 205, 90, 255});
    arrow(Vector2Scale(fb.bounds, s),     Color{160, 160, 170, 255});
    arrow(Vector2Scale(fb.wind, s),       Color{150, 200, 255, 255});
    arrow(Vector2Scale(fb.total, s),      RAYWHITE);

    const Vector2 box{b.pos.x + 16.0f, b.pos.y - 62.0f};
    DrawRectangleV(box, Vector2{220.0f, 56.0f}, Fade(BLACK, 0.55f));
    DrawText(TextFormat("|a| %.0f  |v| %.0f", Vector2Length(fb.total), Vector2Length(b.vel)),
             (int)box.x + 6, (int)box.y + 6, 14, RAYWHITE);
    DrawText(TextFormat("leaders near: %d", nb.folCount),
             (int)box.x + 6, (int)box.y + 28, 14, LIGHTGRAY);
}

void drawFrame(const Frame& f, const Params& params) {
    Params p = params;   // recorded params, so radii/speeds match the run
    p.multiSpecies  = f.multiSpecies;
    p.energyEnabled = f.energyEnabled;

    for (size_t i = 0; i < f.pos.size(); ++i) {
        Boid b;
        b.pos = f.pos[i];
        b.vel = f.vel[i];
        if (i < f.species.size()) b.species = f.species[i];
        if (i < f.flags.size())   b.leader  = (f.flags[i] & 1u) != 0;
        if (i < f.energy.size())  b.energy  = f.energy[i];
        drawBoidShape(b, p);
    }
    for (const Predator& pr : f.predators)
        drawPredatorShape(pr, p);
}

}  // namespace boids
