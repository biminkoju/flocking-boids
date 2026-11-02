// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Bimin Kiran Koju
#include "ui.hpp"

#include <raymath.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace boids {
namespace {

constexpr float kHeaderH = 42.0f;
constexpr float kRowH    = 34.0f;
constexpr float kRowH2   = 32.0f;

// Generic slider with a per-row range and arrow-key fine control while it is
// the active slider (1 percent of range, 0.1 percent with Shift).
bool weightSlider(Rectangle bar, float& value, Ui& ui, int id, Color fill,
                  float lo, float hi) {
    const Vector2 m = GetMousePosition();
    const Rectangle hit{bar.x - 6.0f, bar.y - 12.0f, bar.width + 12.0f, bar.height + 24.0f};
    const float span = std::max(1e-4f, hi - lo);

    bool changed = false;
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(m, hit))
        ui.activeSlider = id;
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && ui.activeSlider == id)
        ui.activeSlider = -1;

    if (ui.activeSlider == id && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        const float t = Clamp((m.x - bar.x) / bar.width, 0.0f, 1.0f);
        value = lo + t * span;
        changed = true;
    }

    if (ui.activeSlider == id) {
        const float step = span * (IsKeyDown(KEY_LEFT_SHIFT) ? 0.001f : 0.01f);
        if (IsKeyPressed(KEY_LEFT))  { value -= step; changed = true; }
        if (IsKeyPressed(KEY_RIGHT)) { value += step; changed = true; }
        value = Clamp(value, lo, hi);
    }

    const float t = Clamp((value - lo) / span, 0.0f, 1.0f);
    DrawRectangleRounded(bar, 0.5f, 8, Color{38, 44, 58, 255});
    if (t > 0.0f)
        DrawRectangleRounded(Rectangle{bar.x, bar.y, bar.width * t, bar.height}, 0.5f, 8, fill);

    const Vector2 knob{bar.x + bar.width * t, bar.y + bar.height * 0.5f};
    const bool hot = ui.activeSlider == id || CheckCollisionPointRec(m, hit);
    DrawCircleV(knob, hot ? 8.0f : 6.5f, RAYWHITE);
    DrawCircleV(knob, hot ? 4.5f : 3.5f, fill);
    return changed;
}

int counter(Rectangle minusBtn, Rectangle plusBtn, int value, int lo, int hi,
            Ui& ui, float dt, int step) {
    const Vector2 m = GetMousePosition();
    const bool overMinus = CheckCollisionPointRec(m, minusBtn);
    const bool overPlus  = CheckCollisionPointRec(m, plusBtn);

    int delta = 0;
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (overMinus)     { delta = -1; ui.heldButton = -1; ui.repeatTimer = 0.35f; }
        else if (overPlus) { delta =  1; ui.heldButton =  1; ui.repeatTimer = 0.35f; }
        else               { ui.heldButton =  0; }
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) ui.heldButton = 0;

    if (ui.heldButton != 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        ui.repeatTimer -= dt;
        if (ui.repeatTimer <= 0.0f) {
            delta = ui.heldButton * step;
            ui.repeatTimer = 0.06f;
        }
    }

    auto drawButton = [&](Rectangle r, const char* label, bool hover, bool enabled) {
        const Color bg = !enabled ? Color{30, 34, 44, 255}
                                  : (hover ? Color{70, 84, 108, 255} : Color{48, 56, 74, 255});
        DrawRectangleRounded(r, 0.25f, 6, bg);
        const int fs = 22;
        const int tw = MeasureText(label, fs);
        DrawText(label, (int)(r.x + (r.width - tw) * 0.5f),
                 (int)(r.y + (r.height - fs) * 0.5f), fs, enabled ? RAYWHITE : GRAY);
    };
    drawButton(minusBtn, "-", overMinus, value > lo);
    drawButton(plusBtn,  "+", overPlus,  value < hi);

    return Clamp(value + delta, lo, hi);
}

// Editable integer field: click to type digits, Enter or click-away commits,
// Esc cancels. Returns the (possibly updated) value.
int numberField(Rectangle box, int value, int id, Ui& ui, int lo, int hi) {
    const Vector2 m = GetMousePosition();
    const bool hover = CheckCollisionPointRec(m, box);
    const bool pressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    if (ui.editing == id) {
        int c;
        while ((c = GetCharPressed()) != 0) {
            if (c >= '0' && c <= '9' && ui.len < 4) {
                ui.buf[ui.len++] = (char)c;
                ui.buf[ui.len] = '\0';
            }
        }
        if (IsKeyPressed(KEY_BACKSPACE) && ui.len > 0) ui.buf[--ui.len] = '\0';

        bool commit = IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER);
        if (pressed && !hover) commit = true;

        if (commit) {
            if (ui.len > 0) value = Clamp(atoi(ui.buf), lo, hi);
            ui.editing = 0;
            ui.len = 0;
            ui.buf[0] = '\0';
        } else if (IsKeyPressed(KEY_ESCAPE)) {
            ui.editing = 0;
            ui.len = 0;
            ui.buf[0] = '\0';
        }
    } else if (pressed && hover) {
        ui.editing = id;
        const int n = snprintf(ui.buf, sizeof(ui.buf), "%d", value);
        ui.len = std::min(n, 4);
        ui.editRect = box;
    }

    const bool active = ui.editing == id;
    DrawRectangleRounded(box, 0.2f, 6, Color{30, 36, 48, 255});
    if (active)     DrawRectangleRoundedLines(box, 0.2f, 6, Color{120, 180, 255, 255});
    else if (hover) DrawRectangleRoundedLines(box, 0.2f, 6, Color{80, 92, 116, 255});

    const char* txt = active ? ui.buf : TextFormat("%d", value);
    const int fs = 18;
    const int tw = MeasureText(txt, fs);
    const int tx = (int)(box.x + (box.width - tw) * 0.5f);
    const int ty = (int)(box.y + (box.height - fs) * 0.5f);
    DrawText(txt, tx, ty, fs, active ? Color{160, 210, 255, 255} : RAYWHITE);

    if (active && !IsKeyDown(KEY_BACKSPACE) && ((int)(GetTime() * 2.0) % 2 == 0))
        DrawRectangle(tx + tw + 1, ty, 2, fs, Color{160, 210, 255, 255});

    return value;
}

// Small on/off pill. Returns true when it was clicked this frame.
bool togglePill(Rectangle r, const char* label, bool on) {
    const Vector2 m = GetMousePosition();
    const bool hover = CheckCollisionPointRec(m, r);
    const Color bg = on ? (hover ? Color{64, 140, 96, 255} : Color{48, 118, 80, 255})
                        : (hover ? Color{70, 78, 96, 255}  : Color{48, 56, 74, 255});
    DrawRectangleRounded(r, 0.3f, 6, bg);
    const int fs = 15;
    const int tw = MeasureText(label, fs);
    DrawText(label, (int)(r.x + (r.width - tw) * 0.5f),
             (int)(r.y + (r.height - fs) * 0.5f), fs, on ? RAYWHITE : LIGHTGRAY);
    return hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

bool textButton(Rectangle r, const char* label, bool hot) {
    const Vector2 m = GetMousePosition();
    const bool hover = CheckCollisionPointRec(m, r);
    const Color bg = hover ? Color{78, 94, 122, 255} : Color{48, 56, 74, 255};
    DrawRectangleRounded(r, 0.28f, 6, bg);
    const int fs = 14;
    const int tw = MeasureText(label, fs);
    DrawText(label, (int)(r.x + (r.width - tw) * 0.5f),
             (int)(r.y + (r.height - fs) * 0.5f), fs, hot ? RAYWHITE : LIGHTGRAY);
    return hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

}  // namespace

namespace {

// Shared hover cursor for the charts tab: -1 when the mouse is not over a plot.
// Computed once in drawCharts so every panel shows the same vertical rule.
int chartCursor = -1;

// One line-chart panel: title, current value, and the recent history scaled to
// its own min/max. `pick` selects the series from a Stats sample.
void chartPanel(const Sim& sim, float x, float y, float w, float h, const char* title,
                Color color, float (*pick)(const Stats::Sample&), bool integer) {
    const Stats& st = sim.stats;
    DrawText(title, (int)x, (int)y, 13, LIGHTGRAY);

    const Rectangle plot{x, y + 16.0f, w, h - 20.0f};
    DrawRectangleRec(plot, Color{24, 29, 38, 255});
    DrawRectangleLinesEx(plot, 1.0f, Color{44, 52, 68, 255});

    if (st.histCount < 2) {
        DrawText("collecting...", (int)plot.x + 6, (int)plot.y + 4, 12, DARKGRAY);
        return;
    }

    float lo = 1e30f, hi = -1e30f, last = 0.0f;
    for (int i = 0; i < st.histCount; ++i) {
        const float v = pick(st.sample(i));
        lo = std::min(lo, v);
        hi = std::max(hi, v);
        last = v;
    }
    if (hi - lo < 1e-4f) { lo -= 0.5f; hi += 0.5f; }

    const float span = hi - lo;
    auto toY = [&](float v) { return plot.y + plot.height - (v - lo) / span * plot.height; };

    float sum = 0.0f;
    Vector2 prev{};
    for (int i = 0; i < st.histCount; ++i) {
        const float v = pick(st.sample(i));
        sum += v;
        const Vector2 pt{plot.x + plot.width * ((float)i / (float)(st.histCount - 1)), toY(v)};
        if (i > 0) DrawLineV(prev, pt, color);
        prev = pt;
    }
    const float mean = sum / (float)st.histCount;

    // Shared time cursor: a vertical rule plus the sampled value at the cursor.
    bool atCursor = false;
    if (chartCursor >= 0 && chartCursor < st.histCount) {
        const float cx = plot.x + plot.width *
                         ((float)chartCursor / (float)(st.histCount - 1));
        DrawLineV(Vector2{cx, plot.y}, Vector2{cx, plot.y + plot.height},
                  Fade(RAYWHITE, 0.35f));
        const float v = pick(st.sample(chartCursor));
        DrawCircleV(Vector2{cx, toY(v)}, 3.0f, color);
        last = v;
        atCursor = true;
    }

    const char* txt = integer ? TextFormat("%d", (int)(last + 0.5f))
                              : TextFormat("%.2f", (double)last);
    DrawText(txt, (int)(x + w - MeasureText(txt, 13)), (int)y, 13,
             atCursor ? color : RAYWHITE);

    // Footer: mean over the window, and the peak when a cursor is active.
    const char* stat = integer
        ? TextFormat("mean %d  peak %d", (int)(mean + 0.5f), (int)(hi + 0.5f))
        : TextFormat("mean %.2f  peak %.2f", (double)mean, (double)hi);
    DrawText(stat, (int)plot.x + 4, (int)(plot.y + plot.height - 14), 10,
             Fade(RAYWHITE, 0.42f));
}

// One panel holding a line per species, so the flocks can be compared directly.
void chartSpeciesPanel(const Sim& sim, float x, float y, float w, float h) {
    const Stats& st = sim.stats;
    const int n = sim.speciesN();

    DrawText("boids per species", (int)x, (int)y, 13, LIGHTGRAY);
    const Rectangle plot{x, y + 16.0f, w, h - 20.0f};
    DrawRectangleRec(plot, Color{24, 29, 38, 255});
    DrawRectangleLinesEx(plot, 1.0f, Color{44, 52, 68, 255});

    if (st.histCount < 2) {
        DrawText("collecting...", (int)plot.x + 6, (int)plot.y + 4, 12, DARKGRAY);
        return;
    }

    float hi = 1.0f;
    for (int i = 0; i < st.histCount; ++i) {
        const Stats::Sample& s = st.sample(i);
        for (int k = 0; k < n; ++k) hi = std::max(hi, (float)s.species[k]);
    }
    auto toY = [&](float v) { return plot.y + plot.height - v / hi * plot.height; };

    for (int k = 0; k < n; ++k) {
        const Color c = speciesColor(k);
        Vector2 prev{};
        for (int i = 0; i < st.histCount; ++i) {
            const Vector2 pt{plot.x + plot.width * ((float)i / (float)(st.histCount - 1)),
                             toY((float)st.sample(i).species[k])};
            if (i > 0) DrawLineV(prev, pt, c);
            prev = pt;
        }
        const int live = st.sample(st.histCount - 1).species[k];
        const char* lbl = TextFormat("%d", live);
        DrawText(lbl, (int)(x + w - MeasureText(lbl, 12)) - k * 22, (int)y, 12, c);
    }
}

void drawCharts(const Sim& sim, Rectangle view, float x, float y) {
    const float h = 62.0f;
    const float w = view.width - 32.0f;

    // Resolve the shared cursor once, using the same x-mapping as the panels.
    chartCursor = -1;
    {
        const Stats& st = sim.stats;
        const Vector2 m = GetMousePosition();
        const float plotTop = y + 16.0f;
        const int panels = sim.params.multiSpecies ? 6 : 5;
        const float plotsBottom = y + (float)(panels - 1) * 62.0f + 62.0f;
        if (st.histCount >= 2 && m.x >= x && m.x <= x + w &&
            m.y >= plotTop && m.y <= plotsBottom) {
            const float t = Clamp((m.x - x) / w, 0.0f, 1.0f);
            chartCursor = (int)(t * (float)(st.histCount - 1) + 0.5f);
        }
    }

    auto order    = [](const Stats::Sample& s) { return s.order; };
    auto speed    = [](const Stats::Sample& s) { return s.avgSpeed; };
    auto nn       = [](const Stats::Sample& s) { return s.nnDist; };
    auto alive    = [](const Stats::Sample& s) { return (float)s.alive; };
    auto kills    = [](const Stats::Sample& s) { return (float)s.kills; };

    chartPanel(sim, x, y,          w, h, "order (polarisation)", Color{140, 235, 150, 255}, order,    false);
    chartPanel(sim, x, y +  62.0f, w, h, "avg speed",            Color{110, 210, 255, 255}, speed,    false);
    chartPanel(sim, x, y + 124.0f, w, h, "mean nn distance",     Color{255, 205, 90, 255},  nn,       false);
    chartPanel(sim, x, y + 186.0f, w, h, "boids alive",          Color{200, 170, 255, 255}, alive,    true);
    chartPanel(sim, x, y + 248.0f, w, h, "prey eaten",           Color{255, 110, 110, 255}, kills,    true);
    if (sim.params.multiSpecies)
        chartSpeciesPanel(sim, x, y + 310.0f, w, h);
}

}  // namespace

Rectangle panelRect(const Ui& ui) {
    return ui.panelOpen
        ? Rectangle{kPanelX, kPanelY, kPanelW, kPanelH}
        : Rectangle{kPanelX, kPanelY, kPanelW, 44.0f};
}

void drawPanel(Sim& sim, Ui& ui, int& boidCount, int& leaderCount, float dt,
               bool replaying) {
    const Rectangle panel = panelRect(ui);
    const Vector2   m     = GetMousePosition();

    // Header (always visible, carries the collapse toggle).
    DrawRectangleRounded(panel, 0.05f, 12, Fade(Color{16, 20, 28, 255}, 0.93f));
    DrawRectangleRoundedLines(panel, 0.05f, 12, Color{52, 62, 84, 255});
    DrawText("FLOCKING", (int)panel.x + 16, (int)panel.y + 13, 19, RAYWHITE);

    const Rectangle collapse{panel.x + panel.width - 34.0f, panel.y + 10.0f, 24.0f, 24.0f};
    DrawText(ui.panelOpen ? "-" : "+", (int)collapse.x + 7, (int)collapse.y + 3, 17, LIGHTGRAY);
    if (CheckCollisionPointRec(m, collapse) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        ui.panelOpen = !ui.panelOpen;
    if (!ui.panelOpen) return;

    if (replaying) {
        DrawText("replay active - Esc to exit", (int)panel.x + 16, (int)panel.y + 48, 14,
                 Color{255, 210, 120, 255});
        return;
    }

    float y = panel.y + kHeaderH;
    const float x = panel.x;
    const float rowW = panel.width;

    // ------------------------------------------------------------- tabs
    const float tabW = (rowW - 32.0f) / 3.0f;
    for (int i = 0; i < 3; ++i) {
        const Rectangle t{x + 16.0f + i * tabW, y, tabW - 4.0f, 26.0f};
        const bool hot = ui.activeTab == i;
        DrawRectangleRounded(t, 0.25f, 6,
                             hot ? Color{70, 86, 112, 255} : Color{38, 44, 58, 255});
        const char* label = i == 0 ? "weights" : (i == 1 ? "radii" : "charts");
        const int fs = 14;
        DrawText(label, (int)(t.x + (t.width - MeasureText(label, fs)) * 0.5f),
                 (int)(t.y + (t.height - fs) * 0.5f), fs, hot ? RAYWHITE : LIGHTGRAY);
        if (CheckCollisionPointRec(m, t) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
            ui.activeTab != i) {
            ui.activeTab = i;
            ui.scroll    = 0.0f;   // each tab starts at the top
        }
    }
    y += 34.0f;

    // ------------------------------------------------ scrollable body
    // Everything below the tabs scrolls. The content height is measured from
    // the previous frame, so the clamp below always uses last frame's layout.
    const float bodyTop    = y;
    const float bodyBottom = panel.y + panel.height - 6.0f;
    const float viewH      = std::max(1.0f, bodyBottom - bodyTop);
    const Rectangle view{panel.x, bodyTop, panel.width, viewH};
    const float maxScroll  = std::max(0.0f, ui.contentH - viewH);

    if (CheckCollisionPointRec(m, view)) {
        const float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) ui.scroll -= wheel * 48.0f;
    }
    ui.scroll = Clamp(ui.scroll, 0.0f, maxScroll);

    BeginScissorMode((int)view.x, (int)view.y, (int)view.width, (int)view.height);
    y -= ui.scroll;

    // --------------------------------------------------------- counters
    auto counterRow = [&](const char* label, int& value, int lo, int hi, int step,
                          const char* hint) {
        const Rectangle minus{x + 96.0f,  y + 3.0f, 26.0f, 26.0f};
        const Rectangle field{x + 126.0f, y + 3.0f, 44.0f, 26.0f};
        const Rectangle plus{x + 174.0f,  y + 3.0f, 26.0f, 26.0f};
        DrawText(label, (int)x + 16, (int)y + 8, 15, LIGHTGRAY);
        value = counter(minus, plus, value, lo, hi, ui, dt, step);
        value = numberField(field, value, lo == 1 ? 1 : 2, ui, lo, hi);
        if (hint) DrawText(hint, (int)x + 210, (int)y + 8, 13, DARKGRAY);
        y += 33.0f;
    };

    counterRow("boids", boidCount, kBoidMin, kBoidMax, 10, "R reset");
    counterRow("leaders", leaderCount, kLeaderMin, kLeaderMax, 1, "");

    const Rectangle autoPill{x + 250.0f, y - 30.0f, 66.0f, 26.0f};
    const bool wantAuto = togglePill(autoPill, "auto F", sim.autoLeaders);
    if (wantAuto) {
        sim.autoLeaders = !sim.autoLeaders;
        sim.setLeaderCount(leaderCount);
    }

    // Predator / target rows reuse the counter widget but edit the sim directly.
    {
        int active = 0;
        for (const Predator& p : sim.predators) if (p.active) ++active;
        int next = counter({x + 96.0f, y + 3.0f, 26.0f, 26.0f},
                           {x + 174.0f, y + 3.0f, 26.0f, 26.0f},
                           active, 0, kPredatorMax, ui, dt, 1);
        next = numberField({x + 126.0f, y + 3.0f, 44.0f, 26.0f}, next, 3, ui, 0, kPredatorMax);
        DrawText("predators", (int)x + 16, (int)y + 8, 15, LIGHTGRAY);
        DrawText("P", (int)x + 210, (int)y + 8, 13, DARKGRAY);
        if (next > active) {
            bool found = false;
            for (auto& p : sim.predators) {
                if (p.active) continue;
                p.pos     = {120.0f, kScreenHeight * 0.5f};
                p.vel     = {sim.params.predatorSpeed, 0.0f};
                p.hunting = true;
                p.timer   = sim.params.predatorHunt;
                p.active  = true;
                found = true;
                break;
            }
            if (!found && sim.predators.size() < (size_t)kPredatorMax) {
                Predator p;
                p.pos     = {120.0f, kScreenHeight * 0.5f};
                p.vel     = {sim.params.predatorSpeed, 0.0f};
                p.hunting = true;
                p.timer   = sim.params.predatorHunt;
                p.active  = true;
                sim.predators.push_back(p);
            }
        } else if (next < active) {
            int toKill = active - next;
            for (int i = (int)sim.predators.size() - 1; i >= 0 && toKill > 0; --i)
                if (sim.predators[i].active) { sim.predators[i].active = false; --toKill; }
        }
        y += 33.0f;
    }
    {
        int active = 0;
        for (const Target& t : sim.targets) if (t.active) ++active;
        int next = counter({x + 96.0f, y + 3.0f, 26.0f, 26.0f},
                           {x + 174.0f, y + 3.0f, 26.0f, 26.0f},
                           active, 0, kTargetMax, ui, dt, 1);
        next = numberField({x + 126.0f, y + 3.0f, 44.0f, 26.0f}, next, 4, ui, 0, kTargetMax);
        DrawText("targets", (int)x + 16, (int)y + 8, 15, LIGHTGRAY);
        DrawText("G", (int)x + 210, (int)y + 8, 13, DARKGRAY);
        if (next > active) {
            bool found = false;
            for (auto& t : sim.targets)
                if (!t.active) { t.pos = {kScreenWidth * 0.5f, kScreenHeight * 0.5f}; t.active = true; found = true; break; }
            if (!found && sim.targets.size() < (size_t)kTargetMax) {
                Target t;
                t.pos    = {kScreenWidth * 0.5f, kScreenHeight * 0.5f};
                t.active = true;
                sim.targets.push_back(t);
            }
        } else if (next < active) {
            int toKill = active - next;
            for (int i = (int)sim.targets.size() - 1; i >= 0 && toKill > 0; --i)
                if (sim.targets[i].active) { sim.targets[i].active = false; --toKill; }
        }
        y += 38.0f;
    }

    const float barX = x + 16.0f;
    const float barW = rowW - 32.0f;

    if (ui.activeTab == 0) {
        // ---------------------------------------------------- weight sliders
        struct WeightDef { const char* name; float* value; Color color; int id; };
        const WeightDef defs[7] = {
            {"separation", &sim.params.separation, Color{255, 120, 90, 255}, 1},
            {"alignment",  &sim.params.alignment,  Color{110, 210, 255, 255}, 2},
            {"cohesion",   &sim.params.cohesion,   Color{140, 235, 150, 255}, 3},
            {"obstacle",   &sim.params.obstacle,   Color{200, 170, 255, 255}, 4},
            {"predator",   &sim.params.predator,   Color{255, 90, 90, 255},   5},
            {"target",     &sim.params.target,     Color{120, 255, 170, 255}, 6},
            {"follow",     &sim.params.follow,     Color{255, 205, 90, 255},  7},
        };
        for (const WeightDef& w : defs) {
            DrawText(w.name, (int)barX, (int)y, 14, LIGHTGRAY);
            const char* val = TextFormat("%.1f", (double)*w.value);
            DrawText(val, (int)(x + rowW - 16.0f - MeasureText(val, 14)), (int)y, 14, RAYWHITE);
            weightSlider({barX, y + 16.0f, barW, 8.0f}, *w.value, ui, w.id, w.color, 0.0f, 4.0f);
            y += 34.0f;
        }

        // ---------------------------------------------------------- presets
        y += 4.0f;
        DrawText("presets", (int)barX, (int)y, 13, DARKGRAY);
        y += 18.0f;
        const std::vector<Preset> list = presets();
        const float bw = (rowW - 32.0f - 8.0f) / 2.0f;
        for (size_t i = 0; i < list.size(); ++i) {
            const Rectangle r{x + 16.0f + (i % 2) * (bw + 8.0f), y + (i / 2) * 30.0f, bw, 26.0f};
            if (textButton(r, list[i].name, false)) {
                // A preset is a scenario: apply its params *and* its runtime
                // toggles, otherwise (e.g.) Forage would set food weights while
                // leaving foraging switched off.
                const Preset& pr = list[i];
                sim.params      = pr.params;
                sim.kills       = pr.kills;
                sim.windEnabled = pr.windEnabled;
                sim.foodEnabled = pr.foodEnabled;
                if (sim.params.multiSpecies) sim.redistributeSpecies();
            }
        }
        y += (((float)list.size() + 1.0f) * 0.5f) * 30.0f + 10.0f;   // two per row

        // -------------------------------------------------- lethal predation
        const Rectangle lethal{barX, y, 132.0f, 26.0f};
        if (togglePill(lethal, sim.kills ? "kills on K" : "kills off K", sim.kills))
            sim.kills = !sim.kills;
        DrawText(TextFormat("eaten %d", sim.killCount), (int)barX + 142, (int)y + 7, 13,
                 sim.kills ? Color{255, 150, 150, 255} : DARKGRAY);
        y += 34.0f;

        // --------------------------------------------- world / neighbour mode
        const float halfW = (rowW - 32.0f - 8.0f) / 2.0f;
        const Rectangle wrapR{barX, y, halfW, 26.0f};
        const Rectangle topoR{barX + halfW + 8.0f, y, halfW, 26.0f};
        if (togglePill(wrapR, "wrap O", sim.params.periodicBounds))
            sim.params.periodicBounds = !sim.params.periodicBounds;
        if (togglePill(topoR, "topo N", sim.params.topological))
            sim.params.topological = !sim.params.topological;
        y += 30.0f;

        const Rectangle energyR{barX, y, halfW, 26.0f};
        const Rectangle distR{barX + halfW + 8.0f, y, halfW, 26.0f};
        if (togglePill(energyR, "energy E", sim.params.energyEnabled))
            sim.params.energyEnabled = !sim.params.energyEnabled;
        if (togglePill(distR, "1/(1+d) W", sim.params.distanceWeighted))
            sim.params.distanceWeighted = !sim.params.distanceWeighted;
        y += 30.0f;

        const Rectangle occlR{barX, y, halfW, 26.0f};
        const Rectangle specR{barX + halfW + 8.0f, y, halfW, 26.0f};
        if (togglePill(occlR, "sight X", sim.params.occlusion))
            sim.params.occlusion = !sim.params.occlusion;
        if (togglePill(specR, "species M", sim.params.multiSpecies)) {
            sim.params.multiSpecies = !sim.params.multiSpecies;
            sim.redistributeSpecies();
        }
        y += 30.0f;

        // Wind and foraging are runtime toggles on the sim, not Params sliders.
        const Rectangle windR{barX, y, halfW, 26.0f};
        const Rectangle foodR{barX + halfW + 8.0f, y, halfW, 26.0f};
        if (togglePill(windR, "wind V", sim.windEnabled))
            sim.windEnabled = !sim.windEnabled;
        const Rectangle huntR{barX, y + 30.0f, halfW, 26.0f};
        const bool huntOn = sim.huntPressureActive();
        if (togglePill(huntR, "hunt H", huntOn))
            sim.setHuntPressure(!huntOn);
        DrawText(huntOn ? TextFormat("pressure %.0f%%", (double)(sim.params.huntPressure * 100.0f))
                        : "manual predators",
                 (int)barX + halfW + 12.0f, (int)y + 37.0f, 12,
                 huntOn ? Color{255, 150, 150, 255} : DARKGRAY);
        if (togglePill(foodR, "food J", sim.foodEnabled)) {
            sim.foodEnabled = !sim.foodEnabled;
            if (sim.foodEnabled) {
                // Entering foraging mode: make sure objectives are actually
                // stocked, otherwise nothing is edible.
                bool any = false;
                for (const Target& t : sim.targets) if (t.active) any = true;
                if (!any) {
                    if (sim.targets.empty()) sim.targets.push_back(Target{});
                    sim.targets[0].pos    = {kScreenWidth * 0.5f, kScreenHeight * 0.5f};
                    sim.targets[0].active = true;
                    sim.targets[0].amount = 1.0f;
                }
            }
        }
        DrawText(sim.foodEnabled ? TextFormat("eaten %d goals", sim.targetsEaten)
                                 : (sim.windEnabled ? "flow field on" : "flow field off"),
                 (int)barX, (int)y + 32, 12,
                 sim.foodEnabled ? Color{140, 235, 150, 255}
                                 : (sim.windEnabled ? Color{150, 200, 255, 255} : DARKGRAY));
        y += 76.0f;

        // Flock count: only meaningful in multi-species mode.
        DrawText("flocks", (int)barX, (int)y + 8, 13,
                 sim.params.multiSpecies ? LIGHTGRAY : DARKGRAY);
        {
            const Rectangle minus{barX + 56.0f, y + 3.0f, 24.0f, 24.0f};
            const Rectangle plus {barX + 112.0f, y + 3.0f, 24.0f, 24.0f};
            int n = std::max(2, std::min(sim.params.speciesCount, kSpeciesMax));
            if (sim.params.multiSpecies) {
                if (textButton(minus, "-", false) && n > 2) --n;
                if (textButton(plus,  "+", false) && n < kSpeciesMax) ++n;
            }
            if (n != sim.params.speciesCount) {
                sim.params.speciesCount = n;
                sim.redistributeSpecies();
            }
            const char* nv = TextFormat("%d", n);
            DrawText(nv, (int)(barX + 84.0f) - MeasureText(nv, 13) / 2, (int)y + 8, 13,
                     sim.params.multiSpecies ? RAYWHITE : DARKGRAY);
        }
        // Colour swatches: bright = in play, faded = available.
        for (int i = 0; i < kSpeciesMax; ++i) {
            const Color c = speciesColor(i);
            const Vector2 at{barX + 156.0f + i * 19.0f, y + 15.0f};
            const bool live = sim.params.multiSpecies && i < sim.params.speciesCount;
            DrawCircleV(at, 7.0f, live ? c : Fade(c, 0.22f));
        }
        y += 32.0f;

        DrawText("cross-species mix", (int)barX, (int)y, 13, LIGHTGRAY);
        const char* xv = TextFormat("%.0f%%", (double)(sim.params.crossSpecies * 100.0f));
        DrawText(xv, (int)(x + rowW - 16.0f - MeasureText(xv, 13)), (int)y, 13, RAYWHITE);
        weightSlider({barX, y + 15.0f, barW, 7.0f}, sim.params.crossSpecies, ui, 121,
                     Color{255, 160, 220, 255}, 0.0f, 1.0f);
        y += 34.0f;

        // ------------------------------------------------------- brush tools
        DrawText("canvas tool", (int)barX, (int)y, 13, DARKGRAY);
        y += 18.0f;
        {
            const char* names[3] = {"nudge", "paint", "erase"};
            const float tw3 = (rowW - 32.0f - 12.0f) / 3.0f;
            for (int i = 0; i < 3; ++i) {
                const Rectangle r{barX + i * (tw3 + 6.0f), y, tw3, 26.0f};
                if (textButton(r, names[i], ui.tool == i)) ui.tool = i;
            }
            y += 30.0f;
            DrawText("brush", (int)barX, (int)y, 13, LIGHTGRAY);
            const char* bv = TextFormat("%.0f px", (double)ui.brushRadius);
            DrawText(bv, (int)(x + rowW - 16.0f - MeasureText(bv, 13)), (int)y, 13, RAYWHITE);
            weightSlider({barX, y + 15.0f, barW, 7.0f}, ui.brushRadius, ui, 120,
                         Color{160, 230, 255, 255}, 20.0f, 200.0f);
            y += 34.0f;
        }

        // ---------------------------------------------------- bottom toggles
        const Rectangle trails{barX,                 y, 96.0f, 26.0f};
        const Rectangle pause {barX + 104.0f,        y, 96.0f, 26.0f};
        const Rectangle reset {barX + 208.0f,        y, rowW - 32.0f - 208.0f, 26.0f};
        if (togglePill(trails, "trails T", sim.trails))     sim.trails = !sim.trails;
        if (togglePill(pause,  "pause S", sim.paused))      sim.paused = !sim.paused;
        if (textButton(reset,  "reset R", false))           sim.reset(boidCount);
        (void)kRowH2;
    } else if (ui.activeTab == 2) {
        drawCharts(sim, view, barX, y);
        // Match the reserved extent to the panels actually drawn (the species
        // panel only exists in multi-species mode).
        y += (sim.params.multiSpecies ? 6.0f : 5.0f) * 62.0f + 10.0f;
    } else {
    // ------------------------------------------------- radii / speed sliders
    struct RadiusDef { const char* name; float* value; float lo; float hi; int id; bool oneDecimal; };
    const RadiusDef defs[27] = {
        {"separation radius", &sim.params.separationRadius, 5.0f,   80.0f,  101, false},
        {"alignment radius",  &sim.params.alignmentRadius,  10.0f,  150.0f, 102, false},
        {"cohesion radius",   &sim.params.cohesionRadius,   10.0f,  200.0f, 103, false},
        {"follow radius",     &sim.params.followRadius,     50.0f,  500.0f, 104, false},
        {"obstacle look",     &sim.params.obstacleLook,     20.0f,  250.0f, 105, false},
        {"predator radius",   &sim.params.predatorRadius,   40.0f,  400.0f, 106, false},
        {"target radius",     &sim.params.targetRadius,     40.0f,  500.0f, 107, false},
        {"bounds margin",     &sim.params.boundsMargin,     10.0f,  200.0f, 108, false},
        {"max speed",         &sim.params.maxSpeed,         40.0f,  500.0f, 109, false},
        {"min speed",         &sim.params.minSpeed,          0.0f,  200.0f, 110, false},
        {"max force",         &sim.params.maxForce,         100.0f, 1200.0f, 111, false},
        {"force budget",      &sim.params.forceBudget,      0.5f,   4.0f,   112, true},
        {"predator speed",    &sim.params.predatorSpeed,    20.0f,  400.0f, 113, false},
        {"hunt (s)",          &sim.params.predatorHunt,     1.0f,   20.0f,  114, true},
        {"rest (s)",          &sim.params.predatorRest,     0.0f,   20.0f,  115, true},
        {"catch radius",      &sim.params.catchRadius,      4.0f,   60.0f,  116, false},
        {"topo k",            &sim.params.topoKf,           1.0f,   32.0f,  117, false},
        {"energy drain",      &sim.params.energyDrain,      0.0f,   0.40f, 118, true},
        {"energy recover",    &sim.params.energyRecover,    0.0f,   0.60f, 119, true},
        {"wind strength",     &sim.params.windStrength,    0.0f,  400.0f, 122, false},
        {"wind angle (deg)",  &sim.params.windAngleDeg,    0.0f,  360.0f, 123, false},
        {"wind turbulence",   &sim.params.windTurbulence,  0.0f,    1.0f, 124, true},
        {"wind gust",         &sim.params.windGust,        0.0f,    2.0f, 125, true},
        {"food drain",        &sim.params.foodDrain,       0.005f,  0.30f, 126, true},
        {"food radius",       &sim.params.foodRadius,      10.0f,  200.0f, 127, false},
        {"food pull (followers)", &sim.params.foodPull,      0.0f,    1.0f, 128, true},
        {"hunt pressure",     &sim.params.huntPressure,   0.0f,    1.0f, 129, true},
    };
    // While hunt pressure drives them, the speed and catch-radius sliders are
    // read-only: they still show the live derived value, but dragging them would
    // lie about having any effect. Drawn dimmed with an "(auto)" tag.
    const bool hp = sim.huntPressureActive();
    for (const RadiusDef& d : defs) {
        const bool derived = hp && (d.id == 113 || d.id == 116);

        DrawText(d.name, (int)barX, (int)y, 13, derived ? DARKGRAY : LIGHTGRAY);
        const char* val = d.oneDecimal ? TextFormat("%.1f", (double)*d.value)
                                       : TextFormat("%.0f", (double)*d.value);
        DrawText(val, (int)(x + rowW - 16.0f - MeasureText(val, 13)), (int)y, 13,
                 derived ? Color{150, 150, 160, 255} : RAYWHITE);

        if (derived) {
            // Inert track, no knob, so it reads as locked.
            DrawRectangleRounded(Rectangle{barX, y + 15.0f, barW, 7.0f}, 0.5f, 8,
                                 Color{38, 44, 58, 255});
            const char* tag = "(auto)";
            DrawText(tag, (int)(barX + barW - MeasureText(tag, 10)), (int)y + 15, 10,
                     Color{140, 140, 150, 255});
        } else {
            weightSlider({barX, y + 15.0f, barW, 7.0f}, *d.value, ui, d.id,
                         Color{120, 180, 255, 255}, d.lo, d.hi);
        }
        y += 30.0f;
    }
    sim.params.topoK = std::max(1, std::min((int)(sim.params.topoKf + 0.5f), kTopoKMax));
    (void)kRowH;
    }

    // Remember how tall the body is so the next frame can clamp the scroll.
    ui.contentH = (y + ui.scroll) - bodyTop;
    EndScissorMode();

    // ------------------------------------------------------- scrollbar
    if (maxScroll > 0.0f) {
        constexpr float kBarW = 7.0f;
        const Rectangle track{panel.x + panel.width - kBarW - 5.0f, bodyTop + 2.0f,
                              kBarW, viewH - 4.0f};
        const float thumbH = std::max(28.0f, track.height * (viewH / ui.contentH));
        const float t      = ui.scroll / maxScroll;
        const Rectangle thumb{track.x, track.y + (track.height - thumbH) * t, kBarW, thumbH};

        DrawRectangleRounded(track, 0.5f, 6, Color{30, 35, 46, 255});
        const bool hot = CheckCollisionPointRec(m, track);
        DrawRectangleRounded(thumb, 0.5f, 6,
                             hot ? Color{130, 160, 210, 255} : Color{90, 106, 136, 255});

        // Click or drag anywhere on the track scrolls to that position.
        if (hot && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            const float rel = Clamp((m.y - track.y - thumbH * 0.5f) /
                                    std::max(1.0f, track.height - thumbH), 0.0f, 1.0f);
            ui.scroll = rel * maxScroll;
        }
    }
}

// ---------------------------------------------------------- canvas handles

int nearestTargetIndex(const Sim& sim, Vector2 m) {
    int best = -1;
    float bd = 1e30f;
    for (size_t i = 0; i < sim.targets.size(); ++i) {
        if (!sim.targets[i].active) continue;
        const float d = Vector2DistanceSqr(m, sim.targets[i].pos);
        if (d < bd) { bd = d; best = (int)i; }
    }
    return best;
}

bool updateCanvasHandles(Sim& sim, Ui& ui, Rectangle panel, Vector2 mouse) {
    ui.hoverTarget  = false;
    ui.hoverObstacle = -1;
    if (CheckCollisionPointRec(mouse, panel)) {
        ui.dragTarget = -1;
        ui.dragObstacle = -1;
        return false;
    }

    for (size_t i = 0; i < sim.targets.size(); ++i) {
        if (!sim.targets[i].active) continue;
        if (CheckCollisionPointCircle(mouse, sim.targets[i].pos, 14.0f))
            ui.hoverTarget = true;
    }
    for (int i = (int)sim.obstacles.size() - 1; i >= 0; --i)
        if (CheckCollisionPointCircle(mouse, sim.obstacles[i].pos, sim.obstacles[i].radius))
            ui.hoverObstacle = i;

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !sim.paused) {
        if (ui.hoverTarget) {
            const int idx = nearestTargetIndex(sim, mouse);
            if (idx >= 0) ui.dragTarget = idx;
        }
        if (ui.dragTarget < 0 && ui.hoverObstacle >= 0)
            ui.dragObstacle = ui.hoverObstacle;
    }
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        ui.dragTarget = -1;
        ui.dragObstacle = -1;
    }

    bool dragging = false;
    if (ui.dragTarget >= 0 && ui.dragTarget < (int)sim.targets.size() &&
        IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        sim.targets[(size_t)ui.dragTarget].pos = mouse;
        dragging = true;
    } else if (ui.dragObstacle >= 0 && ui.dragObstacle < (int)sim.obstacles.size() &&
               IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Obstacle& o = sim.obstacles[(size_t)ui.dragObstacle];
        o.pos.x = Clamp(mouse.x, o.radius, (float)kScreenWidth - o.radius);
        o.pos.y = Clamp(mouse.y, o.radius, (float)kScreenHeight - o.radius);
        dragging = true;
    }
    return dragging;
}

void drawHandleHighlights(const Sim& sim, const Ui& ui) {
    for (size_t i = 0; i < sim.targets.size(); ++i) {
        if (!sim.targets[i].active) continue;
        const bool hot = ui.dragTarget == (int)i ||
            (ui.hoverTarget && nearestTargetIndex(sim, GetMousePosition()) == (int)i);
        if (hot)
            DrawCircleLinesV(sim.targets[i].pos, 16.0f,
                             Fade(Color{120, 255, 170, 255}, 0.9f));
    }
    if (ui.hoverObstacle >= 0 || ui.dragObstacle >= 0) {
        const int i = ui.dragObstacle >= 0 ? ui.dragObstacle : ui.hoverObstacle;
        if (i >= 0 && i < (int)sim.obstacles.size())
            DrawCircleLinesV(sim.obstacles[i].pos,
                             sim.obstacles[i].radius + 5.0f,
                             Fade(Color{255, 210, 120, 255}, 0.85f));
    }
}

// ------------------------------------------------------- offscreen markers

void drawOffscreenMarkers(const Sim& sim, Camera2D camera) {
    const float pad = 22.0f;
    const Rectangle edge{pad, pad, (float)kScreenWidth - 2.0f * pad,
                         (float)kScreenHeight - 2.0f * pad};

    auto marker = [&](Vector2 world, Color c, const char* label) {
        const Vector2 s = GetWorldToScreen2D(world, camera);
        if (s.x >= edge.x && s.x <= edge.x + edge.width &&
            s.y >= edge.y && s.y <= edge.y + edge.height) return;   // on screen

        // Project the off-screen point onto the edge box, seen from the centre.
        const Vector2 ctr{edge.x + edge.width * 0.5f, edge.y + edge.height * 0.5f};
        const Vector2 to{ s.x - ctr.x, s.y - ctr.y };
        const float sx = std::fabs(to.x) > 1e-5f ? (edge.width * 0.5f) / std::fabs(to.x) : 1e30f;
        const float sy = std::fabs(to.y) > 1e-5f ? (edge.height * 0.5f) / std::fabs(to.y) : 1e30f;
        const float t = std::min(sx, sy);
        const Vector2 at{ctr.x + to.x * t, ctr.y + to.y * t};
        const Vector2 dir = Vector2Normalize(to);

        const Vector2 n{-dir.y, dir.x};
        const Vector2 tip = Vector2Add(at, Vector2Scale(dir, 9.0f));
        const Vector2 base = Vector2Subtract(at, Vector2Scale(dir, 6.0f));
        DrawTriangle(tip, Vector2Add(base, Vector2Scale(n, 7.0f)),
                     Vector2Subtract(base, Vector2Scale(n, 7.0f)), c);

        const int tw = MeasureText(label, 11);
        DrawText(label, (int)(at.x - tw * 0.5f), (int)(at.y - 20.0f), 11, Fade(c, 0.9f));
    };

    for (const Target& t : sim.targets)
        if (t.active) marker(t.pos, Color{120, 255, 170, 255}, "T");
    for (const Predator& p : sim.predators)
        if (p.active) marker(p.pos, Color{255, 90, 90, 255}, "P");
}

// ----------------------------------------------------------------------- hud

void drawHud(const Sim& sim, bool recording, int replayIndex, int frameCount) {
    float y = (float)kScreenHeight - 66.0f;
    const char* s1 = TextFormat(
        "boids %d   avg speed %.1f   order %.3f   nn %.1f px   eaten %d%s",
        (int)sim.boids.size(), (double)sim.stats.avgSpeed,
        (double)sim.stats.order, (double)sim.stats.nnDist, sim.killCount,
        sim.kills ? "   [kills on]" : "");
    DrawText(s1, 14, (int)y, 16, RAYWHITE);

    const char* s2 =
        "space pause    t trails    d debug    f auto-leaders    k kills    r reset\n"
        "o wrap    n topo    e energy    w 1/(1+d)    x sight    m species    v wind    j food    h hunt    b brush\n"
        "LMB drag nudge    RMB target    MMB predator    shift+LMB obstacle+    ctrl+LMB obstacle-";
    DrawText(s2, 14, (int)y + 22, 13, Fade(RAYWHITE, 0.55f));

    const char* s3 = "F5 record    F6 save    F9 load    , / . scrub";
    DrawText(s3, 14, (int)y + 40, 13, Fade(RAYWHITE, 0.45f));

    if (recording && ((int)(GetTime() * 2.0) % 2 == 0)) {
        const char* rec = "REC";
        DrawText(rec, 14, 14, 20, Color{255, 80, 80, 255});
        DrawCircle(10, 24, 5, Color{255, 80, 80, 255});
    }
    if (replayIndex >= 0)
        DrawText(TextFormat("replay %d / %d", replayIndex, frameCount), 40, 14, 20,
                 Color{255, 210, 120, 255});
}

}  // namespace boids
