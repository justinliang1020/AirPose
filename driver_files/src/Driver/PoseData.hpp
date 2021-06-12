#pragma once

#include <string>
#include <unordered_map>
#include <vector>

// make pose struct?
namespace PoseVRDriver {
	class PoseData {
	public:
		// parses buffer string into time and positions
		PoseData();

		void add_data(const std::string& buffer);
		bool contains(const std::string& position);

		double get_position(const std::string& position, int dimension);

		std::vector<double> get_position_vector(const std::string& position);
		std::vector<double> get_calibrated_position_vector(const std::string& position);
		std::vector<double> get_desired_tracker_position(const std::string& tracker_name); // implement later

		bool hands_up();

		void calibrate();
		void calculate_scale();
		void calculate_depth();

		void clear_calibration_data();

	public:
		double cur_time = 0;

		bool calibrating1 = false;
		bool calibrating_bridge = false; // bridges calibrating 1 and 2
		int calibrating_bridge_count = 0;	// counts how many times calibrating_bridge is reached
		bool calibrating2 = false;
		bool controller_ids_found = false;

		// current positions
		std::unordered_map<std::string, std::vector<double>> cur_positions; // hashmap key(position) : value(vector xyz)
		std::vector<double> cur_hmd = { 0, 0, 0, 0, 0, 0, 0 }; // contains position(xyz) and rotation (wxyz)
		std::vector<double> cur_left_controller = { 0, 0, 0 };
		std::vector<double> cur_right_controller = { 0, 0, 0 };

		// calibration origins
		std::unordered_map<std::string, std::vector<double>> calibrated1_origins;
		std::vector<double> calibrated1_hmd = { 0, 0, 0 };
		std::vector<double> calibrated1_left_controller = { 0, 0, 0 };
		std::vector<double> calibrated1_right_controller = { 0, 0, 0 };

		std::unordered_map<std::string, std::vector<double>> calibrated2_origins;
		std::vector<double> calibrated2_hmd = { 0, 0, 0 };
		std::vector<double> calibrated2_left_controller = { 0, 0, 0 };
		std::vector<double> calibrated2_right_controller = { 0, 0, 0 };

		// calibration data (only used in calibration process)
		std::vector<std::unordered_map<std::string, std::vector<double>>> calibration1_data;
		std::vector<double> calibration1_data_hmd = { 0, 0, 0 };
		std::vector<double> calibration1_data_left_controller = { 0, 0, 0 };
		std::vector<double> calibration1_data_right_controller = { 0, 0, 0 };

		std::vector<std::unordered_map<std::string, std::vector<double>>> calibration2_data;
		std::vector<double> calibration2_data_hmd = { 0, 0, 0 };
		std::vector<double> calibration2_data_left_controller = { 0, 0, 0 };
		std::vector<double> calibration2_data_right_controller = { 0, 0, 0 };
		
		// set in calculate_scale()
		std::vector<double> offset_waist = { 0, 0, 0 };
		std::vector<double> offset_left_foot = { 0, 0, 0 };
		std::vector<double> offset_right_foot = { 0, 0, 0 };

		std::vector<double> scale_multiplier = { 1, 1, 0.5 };	//z hardcoded fix later

		// depth offsets, calculated int calculate_depth()
		std::vector<double> depth_multiplier_hip = { 1, 1, 1 };
		std::vector<double> depth_multiplier_left_foot = { 1, 1, 1 };
		std::vector<double> depth_multiplier_right_foot = { 1, 1, 1 };

		double picture_plane_depth = 1;	//default before it getes set
	private:
		const int calibration_limit = 150;
		const int calibrating_bridge_count_threshold = 50;

		const double smoothing_a = 0.5;
		const double pi = 3.14159;
		const double body_radius = 0.05; // estimation, z-based so it's fine
		const double headset_to_hip_z = 0.1;
		const double calibration_step_threshold = 0.3;

		const double feet_rotation_sensitivity = 2;
		const double depth_hard_multiplier = 0.5;	//idk random
	};
}