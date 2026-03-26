#pragma once

// Compile-time constants that genuinely cannot change at runtime.
// All tunable values (IPs, ports, thresholds, paths, gains) have moved
// to RuntimeConfig (runtime_config.h) and are loaded from config.json.

static constexpr bool DEFAULT_HEADLESS = false;
