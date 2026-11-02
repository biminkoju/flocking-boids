// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Bimin Kiran Koju
#pragma once

#include "sim.hpp"

namespace boids {

struct Ui {
    int   activeSlider = -1;   // id of the slider being dragged, -1 = none
    int   activeTab    = 0;    // 0 = steering weights, 1 = physics/radii
    int   heldButton   = 0;    // -1 / +1 while a counter button is held
    float repeatTimer  = 0.0f;

    bool panelOpen = true;     // collapsible control panel

    // Canvas tool: 0 = nudge boids, 1 = paint new boids, 2 = erase boids.
    int   tool        = 0;
    float brushRadius = 70.0f;

    // Panel body scrolling: pixel offset from the top, and the measured height
    // of the body content from the previous frame (used to clamp the scroll).
    float scroll   = 0.0f;
    float contentH = 0.0f;

    // Canvas handle dragging (target / obstacles); -1 = none.
    int  dragTarget     = -1;
    int  dragObstacle   = -1;
    bool hoverTarget    = false;
    int  hoverObstacle  = -1;

    // Inline numeric text editing (0 = none, 1 = boids, 2 = leaders).
    int  editing = 0;
    char buf[8]  = {0};
    int  len     = 0;
    Rectangle editRect{};      // bounds of the field being edited

    bool isEditing() const { return editing != 0; }
};

// Current panel bounds (respects the collapsed state).
Rectangle panelRect(const Ui& ui);

// Draws the whole control panel and applies its input to `sim`.
// `boidCount` / `leaderCount` are mirrored back so keyboard shortcuts stay in sync.
void drawPanel(Sim& sim, Ui& ui, int& boidCount, int& leaderCount, float dt,
               bool replaying);

// Hit-tests and drags the targets / obstacles on the canvas.
// Returns true while a handle is being dragged (callers should suppress nudging).
bool updateCanvasHandles(Sim& sim, Ui& ui, Rectangle panel, Vector2 mouse);

// Draws hover / drag highlights for the targets and obstacles.
void drawHandleHighlights(const Sim& sim, const Ui& ui);

// Bottom-left live stats + key hints; `replayIndex` = -1 while not replaying.
void drawHud(const Sim& sim, bool recording, int replayIndex, int frameCount);

// Edge arrows for targets / predators outside the current camera view.
// Screen-space: call after EndMode2D().
void drawOffscreenMarkers(const Sim& sim, Camera2D camera);

}  // namespace boids
