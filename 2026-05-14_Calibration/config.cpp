// config.cpp
// Implementations of the readers declared in config.h

#include "config.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

	// Strip everything from '#' to end-of-line (inline comments)
	// Then trim leading and trailing whitespace
	std::string cleanLine(const std::string& line) {

		// Remove inline comment
		auto hash_pos = line.find('#');
		std::string s = (hash_pos == std::string::npos) ? line : line.substr(0, hash_pos);

		// Trim leading whitespace
		auto start = s.find_first_not_of(" \t\r\n");
		if (start == std::string::npos) return "";
		s = s.substr(start);

		// Trim trailing whitespace
		auto end = s.find_last_not_of(" \t\r\n");
		s = s.substr(0, end + 1);

		return s;
	}

	// Parse a line of the form "key = value" into a (key, value) pair
	// Throws if the line doesn't contain '='
	std::pair<std::string, std::string> parseKeyValue(const std::string& line) {
		auto eq_pos = line.find('=');
		if (eq_pos == std::string::npos) {
			throw std::runtime_error("Expected 'key = value' but found: " + line);
		}

		std::string key = line.substr(0, eq_pos);
		std::string value = line.substr(eq_pos + 1);

		// Trim whitespace from key and value
		auto trim = [](std::string& s) {
			auto start = s.find_first_not_of(" \t\r\n");
			if (start == std::string::npos) { s.clear(); return; }
			auto end = s.find_last_not_of(" \t\r\n");
			s = s.substr(start, end - start + 1);
			};
		trim(key);
		trim(value);

		return { key, value };
	}

	// Parse a string to bool. Accepts "true"/"false" (case-insensitive)
	// Throws on anything else
	bool parseBool(const std::string& value, const std::string& key) {
		std::string v = value;
		for (auto& c : v) c = static_cast<char>(std::tolower(c));
		if (v == "true") return true;
		if (v == "false") return false;
		throw std::runtime_error("Expected true/false for key '" + key + "', got: " + value);
	}

	// Parse a string to double
	// Throws if the string isn't a valid number
	double parseDouble(const std::string& value, const std::string& key) {
		try {
			size_t consumed = 0;
			double result = std::stod(value, &consumed);
			if (consumed != value.size()) {
				throw std::runtime_error("Trailing characters after number for key '" + key + "': " + value);
			}
			return result;
		}
		catch (const std::invalid_argument&) {
			throw std::runtime_error("Expected number for key '" + key + "', got: " + value);
		}
		catch (const std::out_of_range&) {
			throw std::runtime_error("Number out of range for key '" + key + "': " + value);
		}
	}

	// Parse a string to int
	// Throws if the string isn't a valid integer
	int parseInt(const std::string& value, const std::string& key) {
		try {
			size_t consumed = 0;
			int result = std::stoi(value, &consumed);
			if (consumed != value.size()) {
				throw std::runtime_error("Trailing characters after integer for key '" + key + "': " + value);
			}
			return result;
		}
		catch (const std::invalid_argument&) {
			throw std::runtime_error("Expected integer for key '" + key + "', got: " + value);
		}
		catch (const std::out_of_range&) {
			throw std::runtime_error("Integer out of range for key '" + key + "': " + value);
		}
	}

}  // anonymous namespace

namespace config {

	BundleConfig readBundleConfig(const std::string& path) {
		std::ifstream file(path);
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + path);
		}

		BundleConfig cfg;
		std::string line;
		while (std::getline(file, line)) {
			std::string cleaned = cleanLine(line);
			if (cleaned.empty()) continue;

			auto [key, value] = parseKeyValue(cleaned);

			if (key == "image_width") {
				cfg.image_width = parseInt(value, key);
			}
			else if (key == "image_height") {
				cfg.image_height = parseInt(value, key);
			}
			else if (key == "max_iterations") {
				cfg.max_iterations = parseInt(value, key);
			}
			else if (key == "epsilon") {
				cfg.epsilon = parseDouble(value, key);
			}
			else if (key == "verbose") {
				cfg.verbose = parseBool(value, key);
			}
			else if (key == "print_summary") {
				cfg.print_summary = parseBool(value, key);
			}
			else {
				throw std::runtime_error("Unknown key in " + path + ": " + key);
			}
		}

		return cfg;
	}

	APFlags readAPFlags(const std::string& path) {
		std::ifstream file(path);
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + path);
		}

		APFlags flags;
		std::string line;
		while (std::getline(file, line)) {
			std::string cleaned = cleanLine(line);
			if (cleaned.empty()) continue;

			auto [key, value] = parseKeyValue(cleaned);

			if (key == "use_rational_model") {
				flags.use_rational_model = parseBool(value, key);
			}
			else if (key == "use_thin_prism_model") {
				flags.use_thin_prism_model = parseBool(value, key);
			}
			else if (key == "use_tilted_model") {
				flags.use_tilted_model = parseBool(value, key);
			}
			else if (key == "fix_aspect_ratio") {
				flags.fix_aspect_ratio = parseBool(value, key);
			}
			else if (key == "fix_focal_length") {
				flags.fix_focal_length = parseBool(value, key);
			}
			else if (key == "fix_principal_point") {
				flags.fix_principal_point = parseBool(value, key);
			}
			else if (key == "fix_k1") {
				flags.fix_k1 = parseBool(value, key);
			}
			else if (key == "fix_k2") {
				flags.fix_k2 = parseBool(value, key);
			}
			else if (key == "fix_k3") {
				flags.fix_k3 = parseBool(value, key);
			}
			else if (key == "zero_tangent_dist") {
				flags.zero_tangent_dist = parseBool(value, key);
			}
			else if (key == "use_intrinsic_guess") {
				flags.use_intrinsic_guess = parseBool(value, key);
			}
			else {
				throw std::runtime_error("Unknown key in " + path + ": " + key);
			}
		}

		return flags;
	}

	IOPs readIOPs(const std::string& path) {
		std::ifstream file(path);
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + path);
		}

		IOPs iops;
		std::string line;
		while (std::getline(file, line)) {
			std::string cleaned = cleanLine(line);
			if (cleaned.empty()) continue;

			auto [key, value] = parseKeyValue(cleaned);

			if (key == "fx") {
				iops.fx = parseDouble(value, key);
			}
			else if (key == "fy") {
				iops.fy = parseDouble(value, key);
			}
			else if (key == "cx") {
				iops.cx = parseDouble(value, key);
			}
			else if (key == "cy") {
				iops.cy = parseDouble(value, key);
			}
			else if (key == "k1") {
				iops.k1 = parseDouble(value, key);
			}
			else if (key == "k2") {
				iops.k2 = parseDouble(value, key);
			}
			else if (key == "k3") {
				iops.k3 = parseDouble(value, key);
			}
			else if (key == "k4") {
				iops.k4 = parseDouble(value, key);
			}
			else if (key == "k5") {
				iops.k5 = parseDouble(value, key);
			}
			else if (key == "k6") {
				iops.k6 = parseDouble(value, key);
			}
			else if (key == "p1") {
				iops.p1 = parseDouble(value, key);
			}
			else if (key == "p2") {
				iops.p2 = parseDouble(value, key);
			}
			else if (key == "s1") {
				iops.s1 = parseDouble(value, key);
			}
			else if (key == "s2") {
				iops.s2 = parseDouble(value, key);
			}
			else if (key == "s3") {
				iops.s3 = parseDouble(value, key);
			}
			else if (key == "s4") {
				iops.s4 = parseDouble(value, key);
			}
			else if (key == "taux") {
				iops.taux = parseDouble(value, key);
			}
			else if (key == "tauy") {
				iops.tauy = parseDouble(value, key);
			}
			else {
				throw std::runtime_error("Unknown key in " + path + ": " + key);
			}
		}

		return iops;
	}

	std::vector<ControlPoint> readControlPoints(const std::string& path) {
		std::ifstream file(path);
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + path);
		}

		std::vector<ControlPoint> points;
		std::string line;
		int line_number = 0;
		while (std::getline(file, line)) {
			++line_number;
			std::string cleaned = cleanLine(line);
			if (cleaned.empty()) continue;

			std::istringstream iss(cleaned);
			ControlPoint pt;
			if (!(iss >> pt.point_id >> pt.X >> pt.Y >> pt.Z)) {
				throw std::runtime_error(
					"Malformed control point on line " + std::to_string(line_number) +
					" of " + path + ": " + cleaned);
			}

			points.push_back(pt);
		}

		return points;
	}

	std::vector<EOP> readEOPs(const std::string& path) {
		std::ifstream file(path);
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + path);
		}

		std::vector<EOP> eops;
		std::string line;
		int line_number = 0;
		while (std::getline(file, line)) {
			++line_number;
			std::string cleaned = cleanLine(line);
			if (cleaned.empty()) continue;

			std::istringstream iss(cleaned);
			EOP eop;
			if (!(iss >> eop.image_name >> eop.X >> eop.Y >> eop.Z
				>> eop.omega_deg >> eop.phi_deg >> eop.kappa_deg)) {
				throw std::runtime_error(
					"Malformed EOP on line " + std::to_string(line_number) +
					" of " + path + ": " + cleaned);
			}

			eops.push_back(eop);
		}

		return eops;
	}

	std::vector<ImageObservation> readImageObservations(const std::string& path) {
		std::ifstream file(path);
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + path);
		}

		std::vector<ImageObservation> observations;
		std::string line;
		int line_number = 0;
		while (std::getline(file, line)) {
			++line_number;
			std::string cleaned = cleanLine(line);
			if (cleaned.empty()) continue;

			std::istringstream iss(cleaned);
			ImageObservation obs;
			if (!(iss >> obs.point_id >> obs.image_name >> obs.x >> obs.y)) {
				throw std::runtime_error(
					"Malformed image observation on line " + std::to_string(line_number) +
					" of " + path + ": " + cleaned);
			}

			observations.push_back(obs);
		}

		return observations;
	}

}  // namespace config