// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Bimin Kiran Koju
// Advanced Boids / Flocking simulation - C++17 + raylib
//
// Behaviours (each with a live weight slider in the panel):
//   Separation        - steer away from close neighbours
//   Alignment         - match the average neighbour heading
//   Cohesion          - steer toward the neighbour centroid
//   Obstacle          - radial push + tangential slide around round obstacles
//   Predator          - radial escape + tangential slide + panic speed-up
//   Target            - leaders home in on the nearest target point
//   Follow            - followers chase the leader centroid; leaders stay together
// Plus boundary avoidance, speed regulation (per-boid cruise speed),
// mouse nudging, multiple targets / predators and leader selection by fitness.
//
// Usage:
//   boids                      interactive window
//   boids --headless N         run N frames without a window, print stats
//   boids --record PATH        interactive, start recording into PATH

#include "evolve.hpp"
#include "selftest.hpp"
#include "sim.hpp"
#include "ui.hpp"

#include <raylib.h>
#include <raymath.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

using namespace boids;

constexpr int kMaxObstacles = 12;

int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

void activateFirstTarget(Sim& sim, Vector2 mouse) {
    if (sim.targets.empty()) sim.targets.push_back(Target{});
    if (sim.targets.size() < (size_t)kTargetMax) {
        Target t;
        t.pos    = mouse;
        t.active = true;
        t.amount = 1.0f;
        sim.targets.push_back(t);
    } else {
        // Reuse the first slot but restock it so it is edible again.
        sim.targets[0].pos    = mouse;
        sim.targets[0].active = true;
        sim.targets[0].amount = 1.0f;
    }
}

void activateFirstPredator(Sim& sim, Vector2 mouse) {
    Predator p;
    p.pos     = mouse;
    p.vel     = {sim.params.predatorSpeed, 0.0f};
    p.hunting = true;
    p.timer   = sim.params.predatorHunt;
    p.active  = true;
    if (sim.predators.size() < (size_t)kPredatorMax) {
        sim.predators.push_back(p);
    } else {
        sim.predators[0] = p;
    }
}

// Measure the simulation step cost. Reports ms/frame for a small matrix of the
// switches that change per-pair work, so performance claims are measurements.
int runBench(int frames, int boids) {
    // Deterministic seed: without this the RNG is random_device-seeded and two
    // runs cannot be compared at all, which makes the parallel/serial check
    // below impossible.
    setRandomSeed(12345u);
    struct Case { const char* name; bool occl; bool topo; };
    const Case cases[] = {
        {"baseline           ", false, false},
        {"occlusion          ", true,  false},
        {"topological        ", false, true},
        {"occlusion+topo     ", true,  true},
    };

    for (const Case& c : cases) {
        AppState state;
        loadAppState(kStatePath, state);

        Sim sim;
        sim.params           = state.params;
        sim.params.occlusion = c.occl;
        sim.params.topological = c.topo;
        sim.setCount(boids);
        sim.setLeaderCount(20);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 30; ++i)   // warm up: caches, first-touch allocations
            sim.step(dt, false, Vector2{0.0f, 0.0f}, Vector2{0.0f, 0.0f});

        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < frames; ++i)
            sim.step(dt, false, Vector2{0.0f, 0.0f}, Vector2{0.0f, 0.0f});
        const auto t1 = std::chrono::steady_clock::now();

        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::printf("%s  %8.3f ms/frame  (%d boids, %d frames)\n",
                    c.name, ms / (double)frames, boids, frames);
    }
    return 0;
}

int runHeadless(int frames) {
    // Deterministic so two builds (e.g. parallel vs serial force evaluation)
    // can be compared bit-for-bit via the digest below.
    setRandomSeed(12345u);

    AppState state;
    loadAppState(kStatePath, state);

    Sim sim;
    sim.params      = state.params;
    sim.trails      = state.trails;
    sim.autoLeaders = state.autoLeaders;
    sim.kills       = state.kills;
    sim.windEnabled = state.windEnabled;
    sim.foodEnabled = state.foodEnabled;
    sim.setCount(clampInt(state.boidCount, kBoidMin, kBoidMax));
    sim.setLeaderCount(clampInt(state.leaderCount, kLeaderMin, kLeaderMax));

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < frames; ++i) sim.step(dt, false, Vector2{0.0f, 0.0f}, Vector2{0.0f, 0.0f});

    std::printf("frames %d\n", frames);
    std::printf("boids %d\n", (int)sim.boids.size());
    std::printf("avg speed %.3f\n", (double)sim.stats.avgSpeed);
    std::printf("order %.4f\n", (double)sim.stats.order);
    std::printf("mean nn %.3f\n", (double)sim.stats.nnDist);
    // Full-precision digest of the simulation state, for exact A/B comparison.
    {
        double h = 0.0;
        for (const Boid& b : sim.boids)
            h += (double)b.pos.x * 1.0 + (double)b.pos.y * 3.0 +
                 (double)b.vel.x * 7.0 + (double)b.vel.y * 11.0;
        std::printf("digest %.9f\n", h);
    }
    std::printf("kills %s  eaten %d\n", sim.kills ? "on" : "off", sim.killCount);




    state.params      = sim.params;
    state.boidCount   = (int)sim.boids.size();
    state.leaderCount = sim.leaderCount;
    state.trails      = sim.trails;
    state.autoLeaders = sim.autoLeaders;
    state.kills       = sim.kills;
    state.windEnabled = sim.windEnabled;
    state.foodEnabled = sim.foodEnabled;
    saveAppState(kStatePath, state);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    using namespace boids;

    int         headlessFrames = -1;
    std::string recordPath;
    bool        selftest = false;
    int         benchFrames = -1;
    int         benchBoids  = 1200;
    int         evolveGens  = -1;
    std::string evolveOut   = "evolved_params.json";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--headless") == 0 && i + 1 < argc) {
            headlessFrames = std::max(0, atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--record") == 0 && i + 1 < argc) {
            recordPath = argv[++i];
        } else if (std::strcmp(argv[i], "--selftest") == 0) {
            selftest = true;
        } else if (std::strcmp(argv[i], "--bench") == 0 && i + 1 < argc) {
            benchFrames = std::max(1, atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--boids") == 0 && i + 1 < argc) {
            benchBoids = std::max(1, atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--evolve") == 0 && i + 1 < argc) {
            evolveGens = std::max(1, atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            evolveOut = argv[++i];
        }
    }

    if (selftest) return runSelfTest();
    if (benchFrames > 0) return runBench(benchFrames, benchBoids);
    if (evolveGens > 0) return runEvolve(evolveGens, evolveOut.c_str());
    if (headlessFrames >= 0) return runHeadless(headlessFrames);

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(kScreenWidth, kScreenHeight, "Boids - advanced flocking");
    SetTargetFPS(60);
    SetWindowMinSize(640, 400);
    kScreenWidth  = GetScreenWidth();
    kScreenHeight = GetScreenHeight();
    bool fullscreen = false;
    // raylib quits on ESC by default, which would fight "ESC leaves fullscreen".
    SetExitKey(KEY_NULL);

    Sim sim;
    Ui  ui;

    // ------------------------------------------------------------ persistence
    AppState state;
    if (loadAppState(kStatePath, state)) {
        sim.params       = state.params;
        ui.panelOpen     = state.panelOpen;
        sim.trails       = state.trails;
        sim.autoLeaders  = state.autoLeaders;
        sim.kills        = state.kills;
        sim.windEnabled  = state.windEnabled;
        sim.foodEnabled  = state.foodEnabled;
        ui.tool          = clampInt(state.tool, 0, 2);
        ui.brushRadius   = state.brushRadius < 5.0f ? 70.0f : state.brushRadius;
        sim.setCount(clampInt(state.boidCount, kBoidMin, kBoidMax));
        sim.setLeaderCount(clampInt(state.leaderCount, kLeaderMin, kLeaderMax));
    }

    int boidCount   = (int)sim.boids.size();
    int leaderCount = sim.leaderCount;

    // Camera: the sim works in world space, which equals screen space at zoom 1
    // and zero offset. Zoom/pan only change this matrix.
    Camera2D camera{};
    camera.offset = {0.0f, 0.0f};
    camera.target = {0.0f, 0.0f};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
    bool panning = false;
    bool spacePanned = false;   // this space hold panned, so it must not pause
    Vector2 panAnchor{};

    Recorder rec;
    if (!recordPath.empty()) {
        rec.recording = true;
        rec.frames.clear();
    }

    while (!WindowShouldClose()) {
        const float dt = std::min(GetFrameTime(), 1.0f / 30.0f);
        const bool replaying = rec.playing;

        // ------------------------------------------------- window / world size
        // F11 toggles borderless fullscreen; Esc leaves fullscreen (and only then
        // quits). The world IS the window, so any size change is published to the
        // sim before anything else this frame runs.
        if (IsKeyPressed(KEY_F11)) {
            fullscreen = !fullscreen;
            ToggleBorderlessWindowed();
        }
        if (fullscreen && IsKeyPressed(KEY_ESCAPE) && !rec.playing) {
            fullscreen = false;
            ToggleBorderlessWindowed();
        }
        {
            const int winW = GetScreenWidth();
            const int winH = GetScreenHeight();
            if (winW != kScreenWidth || winH != kScreenHeight) {
                kScreenWidth  = winW;
                kScreenHeight = winH;
                sim.onWorldResized();
            }
        }

        // ------------------------------------------------------ camera
        // Wheel zooms toward the cursor; space+LMB drag pans. Both run before
        // any world-space hit testing so the mouse maps correctly.
        if (!ui.isEditing()) {
            const Vector2 wheelAt = GetMousePosition();
            // The settings panel scrolls with the same wheel event, so the
            // camera must ignore it while the cursor is over the panel.
            const bool wheelOverPanel =
                ui.panelOpen && CheckCollisionPointRec(wheelAt, panelRect(ui));
            const float wheel = wheelOverPanel ? 0.0f : GetMouseWheelMove();
            if (wheel != 0.0f) {
                const float before = camera.zoom;
                const float zoomed = std::max(0.15f, std::min(8.0f,
                                          before * std::pow(1.12f, wheel)));
                if (zoomed != before) {
                    const Vector2 worldAt = GetScreenToWorld2D(wheelAt, camera);
                    camera.zoom = zoomed;
                    const Vector2 after = GetWorldToScreen2D(worldAt, camera);
                    camera.offset = Vector2Subtract(camera.offset,
                                                    Vector2Subtract(after, wheelAt));
                }
            }
            if (IsKeyPressed(KEY_ZERO)) { camera.zoom = 1.0f; camera.offset = {0.0f, 0.0f}; }
        }
        const bool spaceHeld = IsKeyDown(KEY_SPACE);
        const bool overPanelForPan =
            CheckCollisionPointRec(GetMousePosition(), panelRect(ui));
        if (spaceHeld && IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !ui.isEditing() &&
            !overPanelForPan) {
            if (!panning) { panning = true; panAnchor = GetMousePosition(); }
            const Vector2 now = GetMousePosition();
            camera.offset = Vector2Add(camera.offset, Vector2Subtract(now, panAnchor));
            panAnchor = now;
            spacePanned = true;
        } else {
            panning = false;
        }

        // ------------------------------------------------------ keyboard
        if (!ui.isEditing()) {
            // Pause on space *release* so a space-held pan does not toggle it.
            if (IsKeyReleased(KEY_SPACE)) {
                if (!spacePanned) sim.paused = !sim.paused;
                spacePanned = false;
            }
            if (IsKeyPressed(KEY_R))     sim.reset(boidCount);
            if (IsKeyPressed(KEY_T))     sim.trails = !sim.trails;
            if (IsKeyPressed(KEY_P)) {
                if (sim.predators.empty()) sim.predators.push_back(Predator{});
                sim.predators[0].active = !sim.predators[0].active;
            }
            if (IsKeyPressed(KEY_G)) {
                if (sim.targets.empty()) sim.targets.push_back(Target{});
                sim.targets[0].active = !sim.targets[0].active;
            }
            if (IsKeyPressed(KEY_D)) sim.debug  = !sim.debug;
            if (IsKeyPressed(KEY_K)) sim.kills  = !sim.kills;   // lethal predation
            if (IsKeyPressed(KEY_O)) sim.params.periodicBounds   = !sim.params.periodicBounds;
            if (IsKeyPressed(KEY_N)) sim.params.topological      = !sim.params.topological;
            if (IsKeyPressed(KEY_E)) sim.params.energyEnabled    = !sim.params.energyEnabled;
            if (IsKeyPressed(KEY_W)) sim.params.distanceWeighted = !sim.params.distanceWeighted;
            if (IsKeyPressed(KEY_B)) ui.tool = (ui.tool + 1) % 3;   // cycle brush tool
            if (IsKeyPressed(KEY_X)) sim.params.occlusion    = !sim.params.occlusion;
            if (IsKeyPressed(KEY_V)) sim.windEnabled         = !sim.windEnabled;
            if (IsKeyPressed(KEY_J)) sim.foodEnabled         = !sim.foodEnabled;
            if (IsKeyPressed(KEY_H)) sim.setHuntPressure(!sim.huntPressureActive());
            if (IsKeyPressed(KEY_M)) sim.params.multiSpecies = !sim.params.multiSpecies;
            if (IsKeyPressed(KEY_F)) {
                sim.autoLeaders = !sim.autoLeaders;
                sim.setLeaderCount(leaderCount);
            }

            if (IsKeyPressed(KEY_F5)) {
                rec.recording = !rec.recording;
                if (rec.recording) { rec.frames.clear(); rec.index = 0; rec.playing = false; }
            }
            if (IsKeyPressed(KEY_F6)) rec.save(recordPath.empty() ? kRecordingPath : recordPath.c_str());
            if (IsKeyPressed(KEY_F9)) {
                if (rec.load(recordPath.empty() ? kRecordingPath : recordPath.c_str()) && !rec.frames.empty()) {
                    rec.playing   = true;
                    rec.recording = false;
                    rec.index     = 0;
                    sim.paused    = true;
                }
            }

            if (rec.playing && !rec.frames.empty()) {
                const int last = (int)rec.frames.size() - 1;
                if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_PERIOD))  rec.index = std::min(last, rec.index + 1);
                if (IsKeyPressed(KEY_LEFT)  || IsKeyPressed(KEY_COMMA))   rec.index = std::max(0, rec.index - 1);
                if (IsKeyPressed(KEY_HOME)) rec.index = 0;
                if (IsKeyPressed(KEY_END))  rec.index = last;
            }
            if (rec.playing && IsKeyPressed(KEY_ESCAPE)) {
                rec.playing = false;
                sim.paused  = false;
            }
        }

        // ------------------------------------------------------ mouse
        const Vector2 screenMouse = GetMousePosition();
        const Vector2 mouse       = GetScreenToWorld2D(screenMouse, camera);
        const Vector2 mouseDelta  = Vector2Scale(GetMouseDelta(), 1.0f / camera.zoom);
        const Rectangle panel    = panelRect(ui);
        // The panel is drawn in screen space, so hit-test it with the raw cursor.
        const bool overPanel     = CheckCollisionPointRec(screenMouse, panel);
        const bool shift         = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        const bool ctrl          = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);

        if (!overPanel && !ui.isEditing() && !replaying) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                if (shift) activateFirstTarget(sim, mouse);
                else       activateFirstTarget(sim, mouse);   // right-click also places
            }
            if (IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) {
                activateFirstPredator(sim, mouse);
            }
        }

        // Shift / Ctrl + left click on the canvas: add / remove an obstacle.
        if (!overPanel && !ui.isEditing() && !replaying &&
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && ui.activeSlider < 0) {
            if (shift) {
                if ((int)sim.obstacles.size() < kMaxObstacles && ui.dragTarget < 0 && ui.dragObstacle < 0) {
                    Obstacle o;
                    o.pos    = mouse;
                    o.radius = 36.0f;
                    sim.obstacles.push_back(o);
                }
            } else if (ctrl) {
                for (size_t i = 0; i < sim.obstacles.size(); ++i) {
                    if (CheckCollisionPointCircle(mouse, sim.obstacles[i].pos, sim.obstacles[i].radius)) {
                        sim.obstacles.erase(sim.obstacles.begin() + (long)i);
                        ui.dragObstacle = -1;
                        break;
                    }
                }
            }
        }

        const bool handleDrag =
            !ui.isEditing() && !replaying && updateCanvasHandles(sim, ui, panel, mouse);

        // Paint / erase brush: continuous while the left button is held. Runs a
        // few interpolated stamps per frame so a fast drag leaves no gaps.
        const bool brushing = !overPanel && !handleDrag && !ui.isEditing() && !replaying &&
                              !shift && !ctrl && !spaceHeld && ui.tool != 0 &&
                              IsMouseButtonDown(MOUSE_BUTTON_LEFT);
        if (brushing) {
            const Vector2 prev = Vector2Subtract(mouse, mouseDelta);
            const float step = std::max(1.0f, ui.brushRadius * 0.4f);
            const float len  = Vector2Length(mouseDelta);
            const int stamps = std::max(1, (int)std::ceil(len / step));
            for (int s = 1; s <= stamps; ++s) {
                const Vector2 at = Vector2Add(prev, Vector2Scale(mouseDelta, (float)s / (float)stamps));
                if (ui.tool == 1) sim.spawnBoidsAt(at, ui.brushRadius, 6);
                else              sim.eraseBoidsAt(at, ui.brushRadius);
            }
            boidCount = (int)sim.boids.size();
        }

        const bool dragging = IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !overPanel &&
                              !handleDrag && !ui.isEditing() && !shift && !ctrl &&
                              !spaceHeld && ui.tool == 0;
        const Vector2 mouseVel = (dt > 0.0f) ? Vector2Scale(mouseDelta, 1.0f / dt) : Vector2{0, 0};

        // ------------------------------------------------------ simulate
        if (!sim.paused && !replaying) {
            sim.step(dt, dragging, mouse, mouseVel);
            if (rec.recording) {
                rec.capture(sim);
                if (!recordPath.empty()) rec.save(recordPath.c_str());
            }

            // Predation removes boids, so keep the panel counter in sync with
            // the sim instead of letting the next setCount() resurrect prey.
            if (sim.kills) boidCount = (int)sim.boids.size();
        }

        // ------------------------------------------------------ draw
        BeginDrawing();
        if (sim.trails) {
            DrawRectangle(0, 0, kScreenWidth, kScreenHeight, Fade(BLACK, 0.15f));
        } else {
            ClearBackground(Color{12, 16, 24, 255});
        }

        BeginMode2D(camera);
        if (replaying) {
            if (!rec.frames.empty()) {
                rec.index = std::min(std::max(0, rec.index), (int)rec.frames.size() - 1);
                drawFrame(rec.frames[(size_t)rec.index], rec.params);
            }
        } else {
            sim.draw();
            drawHandleHighlights(sim, ui);
            if (sim.debug) sim.drawDebug(mouse);
        }

        if (dragging) {
            DrawCircleLinesV(mouse, sim.params.mouseRadius, Fade(Color{255, 210, 120, 255}, 0.55f));
            DrawCircleLinesV(mouse, sim.params.mouseRadius * 0.5f, Fade(Color{255, 210, 120, 255}, 0.30f));
            DrawCircleV(mouse, 5.0f, Fade(Color{255, 220, 150, 255}, 0.9f));
        }

        if (!overPanel && !replaying && ui.tool != 0) {
            const Color brush = ui.tool == 1 ? Color{140, 235, 150, 255} : Color{255, 110, 110, 255};
            DrawCircleLinesV(mouse, ui.brushRadius, Fade(brush, 0.85f));
            DrawCircleV(mouse, 3.0f, Fade(brush, 0.85f));
        }
        EndMode2D();

        if (!overPanel && !replaying && ui.tool != 0) {
            const Color brush = ui.tool == 1 ? Color{140, 235, 150, 255} : Color{255, 110, 110, 255};
            DrawText(ui.tool == 1 ? "paint" : "erase",
                     (int)screenMouse.x + 8, (int)screenMouse.y - 20, 12, Fade(brush, 0.85f));
        }

        if (!replaying) drawOffscreenMarkers(sim, camera);

        if (camera.zoom != 1.0f || camera.offset.x != 0.0f || camera.offset.y != 0.0f)
            DrawText(TextFormat("zoom %.2fx  (0 resets)", (double)camera.zoom),
                     14, (int)kScreenHeight - 86.0f, 14, Fade(Color{160, 200, 255, 255}, 0.8f));

        drawHud(sim, rec.recording, replaying ? rec.index : -1, (int)rec.frames.size());
        drawPanel(sim, ui, boidCount, leaderCount, dt, replaying);

        sim.setCount(boidCount);
        if (sim.leaderCount != leaderCount) sim.setLeaderCount(leaderCount);

        EndDrawing();
    }

    CloseWindow();

    if (rec.recording && rec.frames.size() > 0)
        rec.save(recordPath.empty() ? kRecordingPath : recordPath.c_str());

    state.params      = sim.params;
    state.panelOpen   = ui.panelOpen;
    state.boidCount   = boidCount;
    state.leaderCount = leaderCount;
    state.trails      = sim.trails;
    state.autoLeaders = sim.autoLeaders;
    state.kills       = sim.kills;
    state.windEnabled = sim.windEnabled;
    state.foodEnabled = sim.foodEnabled;
    state.tool        = ui.tool;
    state.brushRadius = ui.brushRadius;
    saveAppState(kStatePath, state);

    return 0;
}
