// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Bimin Kiran Koju
#include "selftest.hpp"

#include "sim.hpp"

#include <cmath>
#include <cstdio>

namespace boids {
namespace {

int gFailures = 0;

void fail(const char* what) {
    std::printf("FAIL %s\n", what);
    ++gFailures;
}

void chkF(const char* name, float a, float b) {
    if (std::fabs(a - b) > 1e-3f) {
        std::printf("FAIL %-22s wrote %.4f  read %.4f\n", name, (double)a, (double)b);
        ++gFailures;
    }
}

void chkI(const char* name, int a, int b) {
    if (a != b) {
        std::printf("FAIL %-22s wrote %d  read %d\n", name, a, b);
        ++gFailures;
    }
}

void chkB(const char* name, bool a, bool b) {
    if (a != b) {
        std::printf("FAIL %-22s wrote %s  read %s\n", name,
                    a ? "true" : "false", b ? "true" : "false");
        ++gFailures;
    }
}

// Every field pushed away from its default: comparing defaults would pass even
// when the wrong variable is written.
Params probeParams() {
    Params p;
    p.separation = 1.11f;  p.alignment = 2.22f;  p.cohesion = 3.33f;
    p.obstacle   = 4.44f;  p.predator  = 5.55f;  p.target   = 6.66f;
    p.follow     = 7.77f;
    p.separationRadius = 11.0f;  p.alignmentRadius = 22.0f;
    p.cohesionRadius   = 33.0f;  p.followRadius    = 44.0f;
    p.obstacleLook     = 55.0f;  p.predatorRadius  = 66.0f;
    p.targetRadius     = 77.0f;  p.boundsMargin    = 88.0f;
    p.boundsWeight     = 9.9f;
    p.maxSpeed    = 101.0f;  p.minSpeed    = 12.0f;
    p.maxForce    = 131.0f;  p.forceBudget = 1.75f;
    p.predatorSpeed = 141.0f;  p.predatorForce = 151.0f;
    p.predatorHunt  = 2.5f;    p.predatorRest  = 3.5f;
    p.catchRadius   = 17.0f;
    p.energyDrain = 0.11f;  p.energyRecover = 0.22f;
    p.energyPanicDrain = 0.33f;
    p.mouseRadius = 99.0f;  p.mousePush = 4.25f;  p.fovCos = -0.42f;
    p.periodicBounds = true;  p.hardWalls = false;
    p.topological = true;  p.topoK = 11;  p.topoKf = 11.0f;
    p.distanceWeighted = true;  p.energyEnabled = true;
    p.occlusion = true;
    p.multiSpecies = true;  p.speciesCount = 4;  p.crossSpecies = 0.37f;
    p.windStrength = 123.0f;  p.windAngleDeg = 45.0f;
    p.windTurbulence = 0.66f; p.windGust = 1.25f;
    p.foodDrain = 0.13f;  p.foodRadius = 95.0f;  p.foodPull = 0.61f;
    p.huntPressure = 0.44f;
    return p;
}

}  // namespace

int runSelfTest() {
    gFailures = 0;

    const Params p = probeParams();
    Params q;
    paramsFromJson(paramsToJson(p), q);

    chkF("separation", p.separation, q.separation);
    chkF("alignment", p.alignment, q.alignment);
    chkF("cohesion", p.cohesion, q.cohesion);
    chkF("obstacle", p.obstacle, q.obstacle);
    chkF("predator", p.predator, q.predator);
    chkF("target", p.target, q.target);
    chkF("follow", p.follow, q.follow);
    chkF("separationRadius", p.separationRadius, q.separationRadius);
    chkF("alignmentRadius", p.alignmentRadius, q.alignmentRadius);
    chkF("cohesionRadius", p.cohesionRadius, q.cohesionRadius);
    chkF("followRadius", p.followRadius, q.followRadius);
    chkF("obstacleLook", p.obstacleLook, q.obstacleLook);
    chkF("predatorRadius", p.predatorRadius, q.predatorRadius);
    chkF("targetRadius", p.targetRadius, q.targetRadius);
    chkF("boundsMargin", p.boundsMargin, q.boundsMargin);
    chkF("boundsWeight", p.boundsWeight, q.boundsWeight);
    chkF("maxSpeed", p.maxSpeed, q.maxSpeed);
    chkF("minSpeed", p.minSpeed, q.minSpeed);
    chkF("maxForce", p.maxForce, q.maxForce);
    chkF("forceBudget", p.forceBudget, q.forceBudget);
    chkF("predatorSpeed", p.predatorSpeed, q.predatorSpeed);
    chkF("predatorForce", p.predatorForce, q.predatorForce);
    chkF("predatorHunt", p.predatorHunt, q.predatorHunt);
    chkF("predatorRest", p.predatorRest, q.predatorRest);
    chkF("catchRadius", p.catchRadius, q.catchRadius);
    chkF("energyDrain", p.energyDrain, q.energyDrain);
    chkF("energyRecover", p.energyRecover, q.energyRecover);
    chkF("energyPanicDrain", p.energyPanicDrain, q.energyPanicDrain);
    chkF("mouseRadius", p.mouseRadius, q.mouseRadius);
    chkF("mousePush", p.mousePush, q.mousePush);
    chkF("fovCos", p.fovCos, q.fovCos);
    chkB("periodicBounds", p.periodicBounds, q.periodicBounds);
    chkB("hardWalls", p.hardWalls, q.hardWalls);
    chkB("topological", p.topological, q.topological);
    chkI("topoK", p.topoK, q.topoK);
    chkB("distanceWeighted", p.distanceWeighted, q.distanceWeighted);
    chkB("energyEnabled", p.energyEnabled, q.energyEnabled);
    chkB("occlusion", p.occlusion, q.occlusion);
    chkB("multiSpecies", p.multiSpecies, q.multiSpecies);
    chkI("speciesCount", p.speciesCount, q.speciesCount);
    chkF("crossSpecies", p.crossSpecies, q.crossSpecies);
    chkF("windStrength", p.windStrength, q.windStrength);
    chkF("windAngleDeg", p.windAngleDeg, q.windAngleDeg);
    chkF("windTurbulence", p.windTurbulence, q.windTurbulence);
    chkF("windGust", p.windGust, q.windGust);
    chkF("foodDrain", p.foodDrain, q.foodDrain);
    chkF("foodRadius", p.foodRadius, q.foodRadius);
    chkF("foodPull", p.foodPull, q.foodPull);
    chkF("huntPressure", p.huntPressure, q.huntPressure);

    // ---- full AppState through the real on-disk format
    AppState s;
    s.params      = p;
    s.panelOpen   = false;
    s.boidCount   = 432;
    s.leaderCount = 17;
    s.trails      = true;
    s.autoLeaders = false;
    s.kills       = true;
    s.windEnabled = true;
    s.foodEnabled = true;
    s.tool        = 2;
    s.brushRadius = 133.0f;

    const char* path = "boids_selftest.json";
    if (!saveAppState(path, s)) fail("saveAppState returned false");

    AppState r;
    if (!loadAppState(path, r)) fail("loadAppState returned false");

    chkB("panelOpen", s.panelOpen, r.panelOpen);
    chkI("boidCount", s.boidCount, r.boidCount);
    chkI("leaderCount", s.leaderCount, r.leaderCount);
    chkB("trails", s.trails, r.trails);
    chkB("autoLeaders", s.autoLeaders, r.autoLeaders);
    chkB("kills", s.kills, r.kills);
    chkB("windEnabled", s.windEnabled, r.windEnabled);
    chkB("foodEnabled", s.foodEnabled, r.foodEnabled);
    chkI("tool", s.tool, r.tool);
    chkF("brushRadius", s.brushRadius, r.brushRadius);

    // Spot-check params surviving the file; paramsToJson alone is covered above.
    chkB("file.periodicBounds", s.params.periodicBounds, r.params.periodicBounds);
    chkB("file.multiSpecies", s.params.multiSpecies, r.params.multiSpecies);
    chkI("file.speciesCount", s.params.speciesCount, r.params.speciesCount);
    chkF("file.crossSpecies", s.params.crossSpecies, r.params.crossSpecies);
    chkF("file.windStrength", s.params.windStrength, r.params.windStrength);
    chkF("file.foodRadius", s.params.foodRadius, r.params.foodRadius);
    chkF("file.catchRadius", s.params.catchRadius, r.params.catchRadius);

    std::remove(path);

    // ---- Recorder v1 backwards compatibility: hand-write an old-format file
    // and confirm it still loads, with the v2 extras simply absent.
    {
        const char* v1Path = "boids_selftest_v1.bin";
        FILE* fp = std::fopen(v1Path, "wb");
        if (!fp) {
            fail("could not create v1 recording");
        } else {
            const uint32_t magic = 0x4b4c4642u;
            const uint32_t version = 1;
            const uint32_t count = 1;
            const uint32_t nb = 2, np = 0;
            const Vector2 pos[2] = {{1.0f, 2.0f}, {3.0f, 4.0f}};
            const Vector2 vel[2] = {{5.0f, 6.0f}, {7.0f, 8.0f}};
            std::fwrite(&magic, 4, 1, fp);
            std::fwrite(&version, 4, 1, fp);
            std::fwrite(&count, 4, 1, fp);
            std::fwrite(&nb, 4, 1, fp);
            std::fwrite(&np, 4, 1, fp);
            std::fwrite(pos, sizeof(Vector2), nb, fp);
            std::fwrite(vel, sizeof(Vector2), nb, fp);
            std::fclose(fp);

            Recorder old;
            const bool ok = old.load(v1Path);
            std::remove(v1Path);
            if (!ok || old.frames.empty()) {
                fail("v1 recording failed to load");
            } else {
                const Frame& f = old.frames[0];
                chkI("v1.boids", (int)nb, (int)f.pos.size());
                chkF("v1.pos1.x", pos[1].x, f.pos[1].x);
                chkI("v1.species.empty", 0, (int)f.species.size());
                chkI("v1.targets.empty", 0, (int)f.targets.size());
            }
        }
    }

    // ---- Recorder v2 round-trip (species / flags / energy / targets)
    {
        Recorder rec;
        Frame a;
        a.pos     = {{1.0f, 2.0f}, {3.0f, 4.0f}};
        a.vel     = {{5.0f, 6.0f}, {7.0f, 8.0f}};
        a.species = {0, 2};
        a.flags   = {0, 1};
        a.energy  = {0.25f, 0.75f};
        Target t;
        t.pos = {10.0f, 20.0f};
        t.amount = 0.5f;
        t.active = true;
        a.targets = {t};
        a.multiSpecies  = true;
        a.energyEnabled = true;
        rec.frames.push_back(a);

        // v3 header: params of the recorded run must survive the file.
        rec.params.predatorRadius = 321.0f;
        rec.params.targetRadius   = 87.0f;
        rec.params.huntPressure   = 0.42f;

        const char* recPath = "boids_selftest.bin";
        if (!rec.save(recPath)) fail("Recorder::save returned false");
        Recorder back;
        if (!back.load(recPath)) fail("Recorder::load returned false");
        std::remove(recPath);

        if (back.frames.empty()) {
            fail("recorder: no frames loaded");
        } else {
            const Frame& b = back.frames[0];
            chkI("rec.boids", (int)a.pos.size(), (int)b.pos.size());
            chkB("rec.multiSpecies", a.multiSpecies, b.multiSpecies);
            chkF("rec.params.predatorRadius", rec.params.predatorRadius,
                 back.params.predatorRadius);
            chkF("rec.params.targetRadius", rec.params.targetRadius,
                 back.params.targetRadius);
            chkF("rec.params.huntPressure", rec.params.huntPressure,
                 back.params.huntPressure);
            chkB("rec.energyEnabled", a.energyEnabled, b.energyEnabled);
            chkI("rec.targets", (int)a.targets.size(), (int)b.targets.size());
            if (b.species.size() == 2) {
                chkI("rec.species0", a.species[0], b.species[0]);
                chkI("rec.species1", a.species[1], b.species[1]);
            } else fail("recorder: species missing");
            if (b.flags.size() == 2) {
                chkI("rec.leader1", a.flags[1], b.flags[1]);
            } else fail("recorder: flags missing");
            if (b.energy.size() == 2) {
                chkF("rec.energy1", a.energy[1], b.energy[1]);
            } else fail("recorder: energy missing");
            if (!b.targets.empty()) {
                chkF("rec.target.amount", a.targets[0].amount, b.targets[0].amount);
                chkF("rec.target.x", a.targets[0].pos.x, b.targets[0].pos.x);
                chkB("rec.target.active", a.targets[0].active, b.targets[0].active);
            }
        }
    }

    if (gFailures == 0) {
        std::printf("selftest OK: params + 17 state fields + recorder v3 round-tripped\n");
        return 0;
    }
    std::printf("selftest FAILED: %d problem(s)\n", gFailures);
    return 1;
}

}  // namespace boids
