// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Bimin Kiran Koju
#pragma once

namespace boids {

// Headless tournament selection over a subset of Params. Writes the winning
// parameter set as JSON to outPath and prints a fitness curve. Returns 0.
int runEvolve(int generations, const char* outPath);

}  // namespace boids
