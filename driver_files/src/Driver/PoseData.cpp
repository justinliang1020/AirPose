#include "PoseData.hpp"
#include <cmath>

PoseVRDriver::PoseData::PoseData()
{
}

// will it ever get incomplete buffer data?
// make prettier later
void PoseVRDriver::PoseData::add_data(const std::string &s)
{
	// parses buffer and adds to cur_positions
	bool cur_positions_empty = cur_positions.empty();
	auto prev_positions = cur_positions;	//testing smoothing
	cur_positions.clear();
	std::string delim = ";";

	bool time_parsed = false;
	std::string cur_position = "";
	int cur_dimension = 3;
	auto start = 0U;
	auto end = s.find(delim);
	while (end != std::string::npos)
	{
		std::string cur = s.substr(start, end - start);
		if (!time_parsed)
		{
			time_parsed = true;
			cur_time = std::stod(cur);
		}
		else if (cur_dimension == 3)
		{
			cur_position = cur;
			cur_positions[cur_position] = std::vector<double>();
			cur_dimension = 0;
		}
		else
		{
			//this->wantedPose[0] = (1 - smoothing) * this->wantedPose[0] + smoothing * a;
			if (cur_positions_empty)
				cur_positions[cur_position].push_back(std::stod(cur));
			else
			{
				//cur_positions[cur_position].push_back(smoothing_a * std::stod(cur) + (1 - smoothing_a) * prev_positions[cur_position][cur_dimension]);
				cur_positions[cur_position].push_back(std::stod(cur));
			}
				
			cur_dimension += 1;
		}
		start = end + delim.length();
		end = s.find(delim, start);
	}

	calibrate();
}

bool PoseVRDriver::PoseData::contains(const std::string& position)
{
	return !(cur_positions.find(position) == cur_positions.end());
}

double PoseVRDriver::PoseData::get_position(const std::string& position, int dimension)
{
	// dimensions: 0 = x, 1 = y, 2 = z. returns void if doesn't contain position
	// TODO: PROPER ERROR HANDLING WHEN POSITION DOESN't EXIST
	if (!contains(position))
		return 1000000;
	return cur_positions[position][dimension];	// FIX THIS STUFF
}

std::vector<double> PoseVRDriver::PoseData::get_position_vector(const std::string& position)
{
	// TODO: PROPER ERROR HANDLING WHEN POSITION DOESN't EXIST
	if (!contains(position))
		return std::vector<double>();
	return cur_positions[position];
}

std::vector<double> PoseVRDriver::PoseData::get_calibrated_position_vector(const std::string& position)
{
	// if not calibrated, returns default position vector
	if (calibrated1_origins.size() == 0)
		return get_position_vector(position);
	std::vector<double> calibrated;
	auto uncalibrated = get_position_vector(position);
	for (int i = 0; i < uncalibrated.size(); ++i)
	{
		calibrated.push_back(calibrated1_origins[position][i] - uncalibrated[i]);
	}
	return calibrated;
}

std::vector<double> PoseVRDriver::PoseData::get_desired_tracker_position(const std::string& tracker_name)
{
	// tracker_name: waist, left_foot, right_foot
	std::vector<double> v;
	if (cur_positions.empty())
	{
		v = { 0, 0, 0 };
		return v;
	}
	
	if (tracker_name == "waist")
	{
		auto left_hip = get_calibrated_position_vector("LEFT_HIP");
		auto right_hip = get_calibrated_position_vector("RIGHT_HIP");
		double offset_hip_x = offset_waist[0];	//changing
		double offset_hip_y = offset_waist[1];	//hardcoding
		double mult_hip_x = scale_multiplier[0];
		double mult_hip_y = scale_multiplier[1];
		double mult_hip_z = 0.2;	//doesn't do anything rn
		auto average_hip_x = -(left_hip[0] + right_hip[0]) / 2; // invert for some reason
		auto average_hip_y = (left_hip[1] + right_hip[1]) / 2;

		// testing out depth correction here
		//x = x' * z / d


		auto hip_x = offset_hip_x + (mult_hip_x * average_hip_x);
		auto hip_y = offset_hip_y + (mult_hip_y * average_hip_y);
		//auto hip_z = cur_hmd[2] + left_hip[2] * 5; // testing
		auto hip_z = cur_hmd[2];

		

		// do rotation stuff here, make into vector of length 7
		
		// ROTATION RADIANS ROTATION RADIANS USE RADIANS YOU DUMBASS
		// rotation about y-axis where a = angle
		// qw = cos(a/2)
		// qx = 0
		// qy = sin(a/2)
		// qz = 0
		// assume that turning 90 degrees means hip.z = 0.5
		double a = ((left_hip[2] - right_hip[2]) / 2) * 3 * pi;	// modify this sensitivity some more
		double qw = std::cos(a/2);
		double qy = std::sin(a/2);	

		// add rotation offset
		// use unit circle to estimate rotation offset
		double x_rotation_offset = - body_radius * cos(pi / 2 - a);	// inverse?
		double z_rotation_offset = -(body_radius * sin(pi / 2 - a) - headset_to_hip_z);

		hip_x += x_rotation_offset;
		hip_z += z_rotation_offset;

		v = { hip_x, hip_y, hip_z, qw, 0, qy, 0 };
		
	}
	else if (tracker_name == "left_foot")
	{
		auto left_ankle = get_calibrated_position_vector("RIGHT_ANKLE");			// inversed because webcam mirrored
		auto left_foot_index = get_calibrated_position_vector("RIGHT_FOOT_INDEX");
		double offset_left_foot_x = offset_left_foot[0];
		double offset_left_foot_y = offset_left_foot[1];
		double mult_left_foot_x = scale_multiplier[0];
		double mult_left_foot_y = scale_multiplier[1];
		double mult_left_foot_z = 0.5;
		depth_multiplier_left_foot[1] = (calibrated1_hmd[2] - cur_hmd[2]) * depth_hard_multiplier;	//rename later
		// testing out depth correction here
		// eg. x = x' * z / d
			//doesn't realyl work
		//double left_foot_x = offset_left_foot_x + (mult_left_foot_x * (-left_ankle[0])); // invert
		//double left_foot_y = offset_left_foot_y + (mult_left_foot_y * left_ankle[1]) - depth_multiplier_left_foot[1] * (cur_hmd[2] - calibrated1_hmd[2]);
		//double left_foot_z = cur_hmd[2] - (mult_left_foot_z * left_ankle[2]);   //invert

		double left_foot_x = offset_left_foot_x + (mult_left_foot_x * (-left_ankle[0])); // invert
		double left_foot_y = offset_left_foot_y + (mult_left_foot_y * left_ankle[1]) + depth_multiplier_left_foot[1];
		//double left_foot_y = offset_left_foot_y + (mult_left_foot_y * left_ankle[1]);
		double left_foot_z = cur_hmd[2] - (mult_left_foot_z * left_ankle[2]);   //invert


		// test end
		double a = (left_foot_index[2] - left_ankle[2]) * feet_rotation_sensitivity * pi;	// guessing hardcode

		if (left_foot_index[0] > left_ankle[0])
			a = -a;

		double qw = std::cos(a / 2);
		double qy = std::sin(a / 2);

		double x_rotation_offset = -body_radius * cos(pi / 2 - a);	// inverse?
		double z_rotation_offset = -(body_radius * sin(pi / 2 - a) - headset_to_hip_z);

		left_foot_x += x_rotation_offset;
		left_foot_z += z_rotation_offset;

		v = { left_foot_x, left_foot_y, left_foot_z, qw, 0, qy, 0 };
	}
	else if (tracker_name == "right_foot")
	{
		auto right_ankle = get_calibrated_position_vector("LEFT_ANKLE");			// inversed because webcam mirrored
		auto right_foot_index = get_calibrated_position_vector("LEFT_FOOT_INDEX");
		double offset_right_foot_x = offset_right_foot[0];
		double offset_right_foot_y = offset_right_foot[1];
		double mult_right_foot_x = scale_multiplier[0];
		double mult_right_foot_y = scale_multiplier[1];
		double mult_right_foot_z = 0.5;
		depth_multiplier_right_foot[1] = (calibrated1_hmd[2] - cur_hmd[2]) * depth_hard_multiplier;	//rename later

		double right_foot_x = offset_right_foot_x + (mult_right_foot_x * (-right_ankle[0]));    //invert
		double right_foot_y = offset_right_foot_y + (mult_right_foot_y * right_ankle[1]) + depth_multiplier_right_foot[1];
		//double right_foot_y = offset_right_foot_y + (mult_right_foot_y * right_ankle[1]);
		double right_foot_z = cur_hmd[2] - (mult_right_foot_z * right_ankle[2]); //invert
		

		double a = (right_foot_index[2] - right_ankle[2]) * feet_rotation_sensitivity * pi;	//guessing hardcode

		if (right_foot_index[0] > right_ankle[0])
			a = -a;

		double qw = std::cos(a / 2);
		double qy = std::sin(a / 2);

		double x_rotation_offset = -body_radius * cos(pi / 2 - a);	// inverse?
		double z_rotation_offset = -(body_radius * sin(pi / 2 - a) - headset_to_hip_z);

		right_foot_x += x_rotation_offset;
		right_foot_z += z_rotation_offset;

		v = { right_foot_x, right_foot_y, right_foot_z, qw, 0, qy, 0 };
	}
	return v;
}

bool PoseVRDriver::PoseData::hands_up()
{
	if (!contains("LEFT_WRIST") || !contains("RIGHT_WRIST"))
		return false;

	double left_wrist_y = get_position("LEFT_WRIST", 1);
	double right_wrist_y = get_position("RIGHT_WIRST", 1);

	if (left_wrist_y < .3 || right_wrist_y < .3)
		return true;
	return false;
}

// TODO: method very ugly, fix later
void PoseVRDriver::PoseData::calibrate() {
	// finds average positions for pose tracked feet and hips, hmd, and controllers.
	// calibration_data is cleared when calibration starts in VRDriver::PipeThread()
	if (!controller_ids_found)
	{
		return;
	}


	// Calibration 1
	if (calibrating1)
	{
		if (calibration1_data.size() != calibration_limit)
		{
			std::unordered_map<std::string, std::vector<double>> m;
			for (auto p : cur_positions)
			{
				m[p.first] = p.second;
			}
			calibration1_data.push_back(m);

			//store data for hmd, controllers
			for (int i = 0; i < 3; ++i)
			{
				calibration1_data_hmd[i] += cur_hmd[i];
				calibration1_data_left_controller[i] += cur_left_controller[i];
				calibration1_data_right_controller[i] += cur_right_controller[i];
			}
		}

		// calibration1 complete
		else {
			calibrated1_origins.clear();

			// calculate calibrated positions
			// iterate through each position and xyz to find sum
			for (auto p : cur_positions)
			{
				std::vector<double> v;
				for (int i = 0; i < 3; ++i)
				{
					double cur_sum = 0;
					for (auto data : calibration1_data)
					{
						cur_sum += data[p.first][i];
					}
					cur_sum /= calibration_limit;
					v.push_back(cur_sum);
				}
				calibrated1_origins[p.first] = v;
			}

			// clear calibration_data
			calibration1_data.clear();

			//calculate averages for hmd, controllers
			for (int i = 0; i < 3; ++i)
			{
				calibrated1_hmd[i] = calibration1_data_hmd[i] / calibration_limit;
				calibrated1_left_controller[i] = calibration1_data_left_controller[i] / calibration_limit;
				calibrated1_right_controller[i] = calibration1_data_right_controller[i] / calibration_limit;

				// clear calibration data for future calibrations
				calibration1_data_hmd[i] = 0;
				calibration1_data_left_controller[i] = 0;
				calibration1_data_right_controller[i] = 0;

			}

			calculate_scale();
			calibrating1 = false;
			// TODO: fix, only starting calibrating2 when plyaer hmd moves depth wise enough
			calibrating_bridge = true;
		}
	}
	else if (calibrating_bridge)
	{
		// fix to differentiate between stepping bakcwards/forwards
		// maybe add a count and only do calibrating2 after cur_hmd crosses threshold given number of tiems
		if (std::abs(cur_hmd[2] - calibrated1_hmd[2]) > calibration_step_threshold)
		{
			calibrating_bridge_count += 1;
		}
		if (calibrating_bridge_count > calibrating_bridge_count_threshold)
		{
			calibrating_bridge = false;
			//calibrating2 = true;
			calibrating_bridge_count = 0;
		}
	}

	// Calibration 2
	else if (calibrating2)
	{
		// calibration stuff
		if (calibration2_data.size() != calibration_limit)
		{
			std::unordered_map<std::string, std::vector<double>> m;
			for (auto p : cur_positions)
			{
				m[p.first] = p.second;
			}
			calibration2_data.push_back(m);

			//store data for hmd, controllers
			for (int i = 0; i < 3; ++i)
			{
				calibration2_data_hmd[i] += cur_hmd[i];
				calibration2_data_left_controller[i] += cur_left_controller[i];
				calibration2_data_right_controller[i] += cur_right_controller[i];
			}
		}

		// calibration complete
		else {
			calibrated2_origins.clear();

			// calculate calibrated positions
			// iterate through each position and xyz to find sum
			for (auto p : cur_positions)
			{
				std::vector<double> v;
				for (int i = 0; i < 3; ++i)
				{
					double cur_sum = 0;
					for (auto data : calibration2_data)
					{
						cur_sum += data[p.first][i];
					}
					cur_sum /= calibration_limit;
					v.push_back(cur_sum);
				}
				calibrated2_origins[p.first] = v;
			}

			// clear calibration_data
			calibration2_data.clear();

			//calculate averages for hmd, controllers
			for (int i = 0; i < 3; ++i)
			{
				calibrated2_hmd[i] = calibration2_data_hmd[i] / calibration_limit;
				calibrated2_left_controller[i] = calibration2_data_left_controller[i] / calibration_limit;
				calibrated2_right_controller[i] = calibration2_data_right_controller[i] / calibration_limit;

				// clear calibration data for future calibrations
				calibration2_data_hmd[i] = 0;
				calibration2_data_left_controller[i] = 0;
				calibration2_data_right_controller[i] = 0;

			}

			calculate_depth();
			calibrating2 = false;
		}
	}
}

void PoseVRDriver::PoseData::calculate_scale()
{
	// only calculates using calibration1 data
	// x scales
	double left_wrist_x = calibrated1_origins["LEFT_WRIST"][0];
	double right_wrist_x = calibrated1_origins["RIGHT_WRIST"][0];
	double left_ankle_x = calibrated1_origins["LEFT_ANKLE"][0];
	double right_ankle_x = calibrated1_origins["RIGHT_ANKLE"][0];
	double nose_x = calibrated1_origins["NOSE"][0];
	double left_hip_x = calibrated1_origins["LEFT_HIP"][0];
	double right_hip_x = calibrated1_origins["RIGHT_HIP"][0];

	double left_wrist_to_right_wrist = std::abs(right_wrist_x - left_wrist_x);

	double left_controller_to_right_controller_x = std::abs(calibrated1_right_controller[0] - calibrated1_left_controller[0]);
	
	double ratio_x = left_controller_to_right_controller_x / left_wrist_to_right_wrist; // vr to pose ratio
	scale_multiplier[0] = ratio_x;

	offset_waist[0] = calibrated1_hmd[0];
	offset_left_foot[0] = calibrated1_hmd[0] + ratio_x * (right_ankle_x - nose_x);	//flip left and right
	offset_right_foot[0] = calibrated1_hmd[0] + ratio_x * (left_ankle_x - nose_x);
	

	// y scales
	double nose_y = calibrated1_origins["NOSE"][1];
	double left_ankle_y = calibrated1_origins["LEFT_ANKLE"][1];
	double right_ankle_y = calibrated1_origins["RIGHT_ANKLE"][1];
	double left_hip_y = calibrated1_origins["LEFT_HIP"][1];
	double right_hip_y = calibrated1_origins["RIGHT_HIP"][1];

	double nose_to_ground_y = ((left_ankle_y + right_ankle_y) / 2) - nose_y;	//add some hardcoded value to find true ground

	double hmd_to_ground_y = calibrated1_hmd[1] + 0.2; // guessing hardcoded offset here, nose to ankle instead of eyes to ground

	double ratio_y = hmd_to_ground_y / nose_to_ground_y;
	scale_multiplier[1] = ratio_y;

	offset_waist[1] = ratio_y * (((left_ankle_y + right_ankle_y) / 2) - ((left_hip_y + right_hip_y) / 2));
	offset_left_foot[1] = 0.1;
	offset_right_foot[1] = 0.1;
}

void PoseVRDriver::PoseData::calculate_depth()
{
	// calculates depth offsets
	
	// x', y': camera 2d-plane points
	// d: picture plane location (on z-axis)
	// x' = x * d / z
	// y' = y * d / z
	// 1) calculate z2, given x1' = x1 * d / z1, x2' = x2 * d / z2; x1', x2', x1, x2, (z1 - z2) = diff
	// x1' * z1 / x1 = x2' * z2 / x2
	// z1 = z2 + diff
	// z1 / z2 = (x2' * x1) / (x1' * x2)
	// z2 + diff / z2 = (x2' * x1) / (x1' * x2)
	// 1 + (diff / z2) = (x2' * x1) / (x1' * x2)
	// diff / z2 = 1 + ((x2' * x1) / (x1' * x2))
	// z2 = diff / (1 + ((x2' * x1) / (x1' * x2))) // inverse the 1s with 2s to find z1
	// 2) calculate d, given x2', x2, z2 (or could calculate with x1', x1, z2)
	// d = x2' * z2 / x2
	// eg. x = x' * z / d, i have x'(pose estimation), z(cur_hmd + pose estimation), and d(calculated constant)
	/*double x1 = calibrated1_hmd[0];
	double x2 = calibrated2_hmd[0];
	double y1 = calibrated1_hmd[1];
	double y2 = calibrated2_hmd[1];
	double diff = calibrated1_hmd[2] - calibrated2_hmd[2];

	double x1p = calibrated1_origins["NOSE"][0];
	double x2p = calibrated2_origins["NOSE"][0];
	double y1p = calibrated1_origins["NOSE"][1];
	double y2p = calibrated2_origins["NOSE"][1];

	double z2 = diff / (1 + ((y2p * y1) / (y1p * y2)));
	double d = y2p * z2 / y2;
	picture_plane_depth = d;*/

	// y2 + depth_multiplier_left_foot * diff = y1, z is calibrated
	double y1 = calibrated1_origins["LEFT_ANKLE"][1];
	double y2 = calibrated2_origins["LEFT_ANKLE"][1];
	double diff = calibrated2_hmd[2] - calibrated1_hmd[2];

	depth_multiplier_left_foot[1] = (y1 - y2) / diff;
}

void PoseVRDriver::PoseData::clear_calibration_data()
{
	// clears calibration1 and calibration2 data
	calibration1_data.clear();
	calibration1_data_hmd = { 0, 0, 0 };
	calibration1_data_left_controller = { 0, 0, 0 };
	calibration1_data_right_controller = { 0, 0, 0 };
}

