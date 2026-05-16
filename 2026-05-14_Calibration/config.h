// config.h
// Readers for calibration input files
// All readers throw std::runtime_error on file-not-found, malformed lines, 
//		or unrecognized keys/values, with messages describing the problem.

#pragma once

#include <string>
#include <vector>

#include "types.h"

namespace config {

// Key-value readers.
// Each parses a file with `key = value` lines (whitespace-flexible).
// Lines beginning with '#' and blank lines are ignored.
IOPs readIOPs(const std::string& path);
APFlags readAPFlags(const std::string& path);
BundleConfig readBundleConfig(const std::string& path);

// Tabular readers.
// Each parses a whitespace-separated table with one record per line.
// Lines beginning with '#' and blank lines are ignored.
std::vector<EOP> readEOPs(const std::string& path);
std::vector<ControlPoint> readControlPoints(const std::string& path);
std::vector<ImageObservation> readImageObservations(const std::string& path);

}  // namespace config
