// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Bimin Kiran Koju
#pragma once

#include <raylib.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace boids {

// Window size doubles as the world size, so these are mutable: resizing the
// window (or going fullscreen) resizes the world. C++17 inline variables keep
// every existing use site working without threading a parameter through ~50 calls.
inline int kScreenWidth  = 1280;
inline int kScreenHeight = 800;
constexpr float kPanelW       = 336.0f;
constexpr float kPanelY       = 12.0f;

// Panel x depends on the live window width, so it is a function, not a constant.
inline float panelX() { return (float)kScreenWidth - kPanelW - 12.0f; }

// Panel height follows the window too, leaving a small margin top and bottom.
inline float panelH() { return (float)kScreenHeight - 2.0f * kPanelY; }

constexpr int kBoidMin     = 1;
constexpr int kBoidMax     = 1200;
constexpr int kLeaderMin   = 0;
constexpr int kLeaderMax   = 60;
constexpr int kPredatorMax = 8;
constexpr int kTargetMax   = 8;
constexpr int kObstacleMax = 12;
constexpr int kTopoKMax    = 32;   // upper bound for the topological neighbour count

constexpr const char* kStatePath     = "boids_state.json";
constexpr const char* kRecordingPath = "recording.bin";

struct Obstacle {
    Vector2 pos{};
    float   radius = 36.0f;
};

constexpr int kSpeciesMax = 6;   // up to six competing flocks

// Distinct, well-separated hues so six flocks stay readable on screen.
inline Color speciesColor(int i) {
    static const Color palette[kSpeciesMax] = {
        {110, 235, 255, 255},   // cyan
        {255, 120, 190, 255},   // pink
        {150, 245, 120, 255},   // lime
        {255, 190, 80, 255},    // amber
        {190, 150, 255, 255},   // violet
        {255, 120, 110, 255},   // coral
    };
    return palette[((i % kSpeciesMax) + kSpeciesMax) % kSpeciesMax];
}

struct Boid {
    Vector2 pos{};
    Vector2 vel{};
    float   cruise = 1.0f;   // per-boid preferred-speed multiplier
    float   energy = 1.0f;   // 0..1 stamina; only used when Params::energyEnabled
    int     species = 0;     // 0 / 1, only meaningful when Params::multiSpecies
    bool    leader = false;

    Vector2 heading() const;
};

struct Target {
    Vector2 pos{};
    bool    active = false;
    // Remaining food, 1 = full, 0 = eaten. Only used when Sim::foodEnabled.
    float   amount = 1.0f;
};

struct Predator {
    Vector2 pos{};
    Vector2 vel{};
    bool    hunting = true;
    float   timer   = 0.0f;   // seconds left in the current hunt/rest phase
    int     kills   = 0;      // prey caught so far (lethal mode)
    bool    active  = false;
};

// Every steering weight plus every previously hardcoded magic number. Radii
// are in pixels, speeds in px/s, forces in px/s^2, times in seconds.
struct Params {
    float separation = 1.8f;
    float alignment  = 1.0f;
    float cohesion   = 0.9f;
    float obstacle   = 2.5f;
    float predator   = 3.5f;
    float target     = 1.2f;
    float follow     = 1.5f;

    float separationRadius = 30.0f;
    float alignmentRadius  = 60.0f;
    float cohesionRadius   = 60.0f;
    float followRadius     = 260.0f;
    float obstacleLook     = 110.0f;
    float predatorRadius   = 170.0f;
    float targetRadius     = 220.0f;
    float boundsMargin     = 90.0f;
    float boundsWeight     = 3.0f;

    float maxSpeed    = 220.0f;
    float minSpeed    = 70.0f;
    float maxForce    = 420.0f;
    float forceBudget = 2.0f;   // shared acceleration budget, multiple of maxForce

    float predatorSpeed = 150.0f;
    float predatorForce = 260.0f;
    float predatorHunt  = 6.0f;
    float predatorRest  = 4.0f;
    float catchRadius   = 14.0f;   // kill radius while lethal predation is on

    // Single derived knob for predator lethality: drives predatorSpeed,
    // catchRadius and the number of active predators. It overrides those three
    // while >= 0, so one dial reliably produces a population cycle.
    float huntPressure = -1.0f;   // <0 = disabled (individual sliders win)

    float mouseRadius = 130.0f;
    float mousePush   = 9.0f;

    // World shape: wrap (toroidal) beats the classic bouncing walls because
    // the flock can stream across a seam without noticing it.
    bool periodicBounds = false;   // wrap positions, no wall force, toroidal distances
    bool hardWalls      = true;    // clamp + reflect at the screen edge

    // Neighbour interaction. Metric = hard radius cut (classic boids).
    // Topological = the K nearest neighbours feed alignment and cohesion
    // (Ballerini et al. 2008 - real starlings interact topologically).
    bool  topological      = false;
    int   topoK            = 7;
    float topoKf           = 7.0f;   // slider mirror of topoK (weightSlider edits floats)
    bool  distanceWeighted = false;  // weight neighbours by 1/(1+d)

    // Per-boid stamina. Drains with speed and predator pressure, recovers when
    // coasting; low energy lowers the boid's effective top speed.
    bool  energyEnabled    = false;
    float energyDrain      = 0.07f;  // per second at full speed
    float energyRecover    = 0.18f;  // per second while coasting
    float energyPanicDrain = 0.25f;  // extra per second while a predator is near

    float fovCos = -0.3f;   // cos of the half-angle; at least 1 disables the FOV

    // Obstacles block sight: neighbours hidden behind a disc are ignored, so a
    // flock can genuinely hide behind a rock instead of seeing through it.
    bool occlusion = false;

    // Global wind / flow field: the lowest-priority steering force, so survival
    // and flocking always win when the acceleration budget is tight. The field
    // is a sine sum sampled per boid position, so different boids feel different
    // drift. Wavenumbers are quantised to the screen size, which keeps the field
    // seamless when periodicBounds wraps the world.
    float windStrength   = 60.0f;  // px/s^2 at full scale (same units as maxForce)
    float windAngleDeg   = 0.0f;   // base direction in degrees, 0 = +x
    float windTurbulence = 0.0f;   // 0..1 spatial variation
    float windGust       = 0.0f;   // 0..2 temporal variation rate

    // Foraging: boids inside foodRadius eat the target, which shrinks and then
    // disappears, forcing the flock to pick a new objective.
    float foodDrain  = 0.05f;   // amount per second per eating boid
    float foodRadius = 60.0f;   // how close a boid must be to eat
    // Target seeking strength for FOLLOWERS, as a fraction of a leader's pull.
    // 0 restores the old "only leaders seek" behaviour; leaders are always 1.
    float foodPull   = 0.25f;

    // Two competing flocks. Alignment/cohesion only see same-species neighbours;
    // separation always applies across species. crossSpecies in [0,1] blends the
    // other species back into alignment/cohesion (0 = fully segregated).
    bool  multiSpecies = false;
    int   speciesCount = 2;    // 2..kSpeciesMax flocks when multiSpecies is on
    float crossSpecies = 0.0f;
};

struct Stats {
    float avgSpeed = 0.0f;
    float order    = 0.0f;   // |mean heading|, 1 = perfectly polarised
    float nnDist   = 0.0f;   // mean nearest-neighbour distance

    // Rolling history for the charts tab (one sample per simulation step).
    static constexpr int kHistoryCap = 600;
    struct Sample {
        float order    = 0.0f;
        float avgSpeed = 0.0f;
        float nnDist   = 0.0f;
        int   alive    = 0;
        int   kills    = 0;
        int   species[kSpeciesMax]{};   // live count per species (multi-flock chart)
    };
    Sample hist[kHistoryCap]{};
    int    histCount = 0;    // total samples ever pushed (saturating at cap)
    int    histHead  = 0;    // index of the next slot to write

    // One sample is stored every kSampleEvery steps, so a 600-sample window
    // covers ~60 s of simulation at 60 fps instead of ~10 s.
    static constexpr int kSampleEvery = 6;
    int sampleTick = 0;      // steps since the last stored sample

    void pushSample(const Sample& s) {
        hist[histHead] = s;
        histHead = (histHead + 1) % kHistoryCap;
        if (histCount < kHistoryCap) ++histCount;
    }
    // 0 = oldest retained sample, histCount-1 = newest.
    const Sample& sample(int i) const {
        const int start = (histHead - histCount + kHistoryCap * 2) % kHistoryCap;
        return hist[(start + i) % kHistoryCap];
    }
    void clearHistory() { histCount = 0; histHead = 0; sampleTick = 0; }
};

// Per-force breakdown, used by the debug overlay.
struct ForceBreakdown {
    Vector2 separation{};
    Vector2 alignment{};
    Vector2 cohesion{};
    Vector2 obstacle{};
    Vector2 predator{};
    Vector2 target{};
    Vector2 follow{};
    Vector2 bounds{};
    Vector2 mouse{};
    Vector2 wind{};
    Vector2 total{};
};

struct Sim;

// One recorded frame of boid + predator positions.
struct Frame {
    std::vector<Vector2>  pos;
    std::vector<Vector2>  vel;
    std::vector<Predator> predators;

    // v2 fields: species, leadership, stamina and objectives. v1 files still
    // load; the extras just come back empty and draw as defaults.
    std::vector<uint8_t> species;
    std::vector<uint8_t> flags;     // bit0 = leader
    std::vector<float>   energy;
    std::vector<Target>  targets;
    bool multiSpecies  = false;
    bool energyEnabled = false;
};

// Records positions each frame and plays them back.
struct Recorder {
    bool               recording = false;
    bool               playing   = false;
    std::vector<Frame> frames;
    int                index = 0;

    // Parameters in force when the recording started. Written once in the file
    // header so replays draw with the radii/speeds the run actually used rather
    // than with the defaults.
    Params             params{};

    void capture(const Sim& sim);
    bool save(const char* path) const;
    bool load(const char* path);
};

enum PresetId {
    PresetDefault, PresetBall, PresetStream, PresetPanic,
    PresetMurmuration, PresetForage, PresetContest, PresetBreeze,
    PresetCount
};

// A scenario, not just a weight tweak: the runtime toggles live on Sim, so a
// preset carries them too, otherwise selecting it would leave the flock in the
// wrong mode (e.g. foraging with no food enabled).
struct Preset {
    PresetId    id;
    const char* name;
    Params      params;
    bool        kills       = false;
    bool        windEnabled = false;
    bool        foodEnabled = false;
};

struct AppState {
    Params params;
    bool   panelOpen   = true;
    int    boidCount   = 250;
    int    leaderCount = 0;
    bool   trails      = false;
    bool   autoLeaders = true;
    bool   kills       = false;
    bool   windEnabled = false;
    bool   foodEnabled = false;
    int    tool        = 0;      // 0 nudge, 1 paint boids, 2 erase boids
    float  brushRadius = 70.0f;
};

// Seed the shared RNG. Needed to compare runs bit-for-bit, e.g. checking that
// parallel force evaluation produces exactly the serial result.
void setRandomSeed(unsigned int seed);

std::string         paramsToJson(const Params& p);
void                paramsFromJson(const std::string& text, Params& p);
bool                loadAppState(const char* path, AppState& s);
bool                saveAppState(const char* path, const AppState& s);
std::vector<Preset> presets();

struct Sim {
    std::vector<Boid>     boids;
    std::vector<Obstacle> obstacles;
    std::vector<Target>   targets;
    std::vector<Predator> predators;
    Params                params;

    bool paused = false;
    bool trails = false;
    bool debug  = false;

    // Lethal predation: a hunting predator that gets within catchRadius of a
    // boid removes it and then digests (rests) for predatorRest seconds.
    bool kills     = false;
    int  killCount = 0;   // prey eaten since the last reset

    // Wind is a runtime toggle, not physics, so it lives here rather than in
    // Params. simTime drives the gust term and is dt-accumulated (not wall
    // clock) so headless runs and recordings replay identically.
    bool  windEnabled = false;
    float simTime     = 0.0f;

    // Foraging mode: targets carry food that boids consume. A consumed target
    // goes inactive and, if foodRespawn is on, a fresh one appears elsewhere so
    // the demo keeps generating objectives.
    bool foodEnabled   = false;
    bool foodRespawn   = true;
    int  targetsEaten  = 0;   // objectives fully consumed since the last reset

    int  leaderCount = 0;
    bool autoLeaders = true;   // leaders chosen by fitness, refreshed periodically

    Stats stats;

    Sim();

    // Clamped number of flocks currently in play (1 when multi-species is off).
    int  speciesN() const;
    // Re-deals species ids across the live flock, keeping the split even.
    void redistributeSpecies();

    // The world is the window: after a resize, everything that may now lie
    // outside the bounds is folded back in. Called once per size change.
    void onWorldResized();

    // True while huntPressure is driving predatorSpeed / catchRadius, i.e. those
    // two individual sliders are overridden and have no effect.
    bool huntPressureActive() const { return params.huntPressure >= 0.0f; }

    // Enables/disables hunt pressure, saving and restoring the manual
    // predatorSpeed / catchRadius pair so turning it off returns the sliders to
    // the values they had before pressure took them over.
    void setHuntPressure(bool on);

    void reset(int n);
    void addBoids(int n);
    void removeBoids(int n);
    // Paint tools: spawn fresh boids inside a disc, or kill every boid inside it.
    // Returns the number of boids actually created / removed.
    int  spawnBoidsAt(Vector2 center, float radius, int n);
    int  eraseBoidsAt(Vector2 center, float radius);
    void setCount(int n);
    void setLeaderCount(int n);

    void step(float dt, bool pointerActive, Vector2 pointer, Vector2 pointerVel);
    void draw() const;
    void drawDebug(Vector2 mouse) const;

    const Target* nearestTargetTo(Vector2 p) const;

private:
    struct Grid {
        int              cols = 0;
        int              rows = 0;
        float            cell = 64.0f;
        std::vector<int> counts;
        std::vector<int> starts;
        std::vector<int> items;
    };

    struct NeighborData {
        Vector2 sepSum{};
        Vector2 aliSum{};
        Vector2 cohSum{};
        Vector2 folSum{};
        float   aliW = 0.0f;   // accumulated neighbour weights (distance-weighted modes)
        float   cohW = 0.0f;
        // Per-species accumulators (multi-flock mode). Index by Boid::species.
        Vector2 aliSumS[kSpeciesMax]{};
        Vector2 cohSumS[kSpeciesMax]{};
        float   aliWS[kSpeciesMax]{};
        float   cohWS[kSpeciesMax]{};
        int     aliCountS[kSpeciesMax]{};
        int     cohCountS[kSpeciesMax]{};
        int     sepCount = 0;
        int     aliCount = 0;
        int     cohCount = 0;
        int     folCount = 0;
    };

    struct Context {
        Vector2                     leaderCenter{};
        Vector2                     leaderVel{};
        int                         leaders = 0;
        std::vector<const Boid*>    leadersPtr;
        std::vector<const Predator*> predators;
    };

    Grid                 grid;
    std::vector<Vector2> accel;
    std::vector<int>     leaderIdx;
    float                leaderTimer = 0.0f;

    void buildGrid();
    // Seps/ali/coh come from the fine grid; followers check the (few)
    // leaders directly so followRadius does not inflate the grid cells.
    NeighborData gatherNeighbors(const Boid& b, size_t self, const Context& ctx) const;
    Context buildContext() const;

    void updateStats();
    // True when the segment a->b passes through an obstacle disc.
    bool occluded(Vector2 a, Vector2 b) const;
    void updateLeaders();
    void updatePredators(float dt);
    void updatePredator(Predator& p, float dt);
    void resolvePredation();
    void updateFood(float dt);
    // Applies Params::huntPressure to speed / catch radius / predator count.
    void applyHuntPressure();

    // Manual values stashed when hunt pressure is switched on (-1 = none).
    float manualPredatorSpeed = -1.0f;
    float manualCatchRadius   = -1.0f;

    void accumulateForces(const Boid& b, const NeighborData& nb, const Context& ctx,
                          bool pointerActive, Vector2 pointer, Vector2 pointerVel,
                          ForceBreakdown& out) const;
};

void drawFrame(const Frame& f, const Params& p);

}  // namespace boids
