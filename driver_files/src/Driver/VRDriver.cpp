#include "VRDriver.hpp"
#include <Driver/HMDDevice.hpp>
#include <Driver/TrackerDevice.hpp>
#include <Driver/ControllerDevice.hpp>
#include <Driver/TrackingReferenceDevice.hpp>

std::mutex mymutex;

vr::EVRInitError PoseVRDriver::VRDriver::Init(vr::IVRDriverContext* pDriverContext)
{

    // Perform driver context initialisation
    if (vr::EVRInitError init_error = vr::InitServerDriverContext(pDriverContext); init_error != vr::EVRInitError::VRInitError_None) {
        return init_error;
    }

    Log("Activating ExampleDriver...");

    Log("Connecting to pipe...");
    LPTSTR lpszPipename = TEXT("\\\\.\\pipe\\posevr_pipe");

    // Open the named pipe
    // Most of these parameters aren't very relevant for pipes.
    pipe = CreateFile(
        lpszPipename,
        GENERIC_READ, // only need read access
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    // trackers not created if pipe client isn't connected to server
    if (pipe == INVALID_HANDLE_VALUE) {
        Log("Failed to connect to pipe. PoseVR not creating trackers.");
        //system("pause");
        pipe_connected = false;
        CloseHandle(pipe);
        return vr::VRInitError_None;
    }

    // Initialize pipe thread
    Log("Starting pipe thread");
    std::thread pipeThread(&PoseVRDriver::VRDriver::PipeThread, this);
    pipeThread.detach();

    // Add trackers
    auto waist_tracker = std::make_shared<TrackerDevice>("0Waist_TrackerDevice");
    this->AddDevice(waist_tracker);
    this->trackers_.push_back(waist_tracker);

    auto left_foot_tracker = std::make_shared<TrackerDevice>("1LeftFoot_TrackerDevice");
    this->AddDevice(left_foot_tracker);
    this->trackers_.push_back(left_foot_tracker);

    auto right_foot_tracker = std::make_shared<TrackerDevice>("2RightFoot_TrackerDevice");
    this->AddDevice(right_foot_tracker);
    this->trackers_.push_back(right_foot_tracker);

    
    

    Log("ExampleDriver Loaded Successfully");

	return vr::VRInitError_None;
}

void PoseVRDriver::VRDriver::Cleanup()
{
}

void PoseVRDriver::VRDriver::RunFrame()
{
    if (pipe_connected)
    {
        // Collect events
        vr::VREvent_t event;
        std::vector<vr::VREvent_t> events;
        while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event)))
        {
            events.push_back(event);
        }
        this->openvr_events_ = events;

        // Update frame timing
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        this->frame_timing_ = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_frame_time_);
        this->last_frame_time_ = now;

        // lock thread to for thread safety
        std::lock_guard<std::mutex> g(mymutex);

        // update HMD and controllers position
        GetCurHMDControllersPose();

        std::vector<double> waist_pos = poseData.get_desired_tracker_position("waist");
        std::vector<double> left_foot_pos = poseData.get_desired_tracker_position("left_foot");
        std::vector<double> right_foot_pos = poseData.get_desired_tracker_position("right_foot");

        // vertical correction for depth when player moves forwards or backwards
        // fix depth correction later
        /*if (poseData.calibrated1_hmd[1] != 0 && poseData.cur_hmd[1] - poseData.calibrated1_hmd[1] < jump_threshold)
        {
            if ((left_foot_pos[1] > 0.1 && right_foot_pos[1] > 0.1))
            {
                if (left_foot_pos[1] < right_foot_pos[1])
                {
                    left_foot_pos[1] += 0.1 - right_foot_pos[1];
                    right_foot_pos[1] = 0.1;
                }
                else
                {
                    left_foot_pos[1] = 0.1;
                    right_foot_pos[1] += 0.1 - left_foot_pos[1];
                }
            }
            else if (left_foot_pos[1] < 0.1 && right_foot_pos[1] < 0.1)
            {
                if (left_foot_pos[1] < right_foot_pos[1])
                {
                    left_foot_pos[1] = 0.1;
                    right_foot_pos[1] += 0.1 - left_foot_pos[1];
                }
                else
                {
                    left_foot_pos[1] += 0.1 - right_foot_pos[1];
                    right_foot_pos[1] = 0.1;
                }
            }
        }*/

        // "sitting" detection (elongates legs when player sits down)
        // TODO: needs to be able to work even if player doesn't directly face forward
        if (std::abs(waist_pos[1] - left_foot_pos[1]) < sit_threshold
            && std::abs(waist_pos[1] - right_foot_pos[1]) < sit_threshold)
        {
            left_foot_pos[2] += (left_foot_pos[2] - waist_pos[2]) * sit_foot_depth_multiplier;
            right_foot_pos[2] += (left_foot_pos[2] - waist_pos[2]) * sit_foot_depth_multiplier;
        }

        this->trackers_[0]->UpdatePos(waist_pos[0], waist_pos[1], waist_pos[2]);
        this->trackers_[0]->UpdateRot(waist_pos[3], waist_pos[4], waist_pos[5], waist_pos[6]);
        this->trackers_[1]->UpdatePos(left_foot_pos[0], left_foot_pos[1], left_foot_pos[2]);
        this->trackers_[1]->UpdateRot(left_foot_pos[3], left_foot_pos[4], left_foot_pos[5], left_foot_pos[6]);
        this->trackers_[2]->UpdatePos(right_foot_pos[0], right_foot_pos[1], right_foot_pos[2]);
        this->trackers_[2]->UpdateRot(right_foot_pos[3], right_foot_pos[4], right_foot_pos[5], right_foot_pos[6]);


        for (auto& device : this->devices_)
            device->Update();
    }
    
}

bool PoseVRDriver::VRDriver::ShouldBlockStandbyMode()
{
    return false;
}

void PoseVRDriver::VRDriver::EnterStandby()
{
}

void PoseVRDriver::VRDriver::LeaveStandby()
{
}

std::vector<std::shared_ptr<PoseVRDriver::IVRDevice>> PoseVRDriver::VRDriver::GetDevices()
{
    return this->devices_;
}

std::vector<vr::VREvent_t> PoseVRDriver::VRDriver::GetOpenVREvents()
{
    return this->openvr_events_;
}

std::chrono::milliseconds PoseVRDriver::VRDriver::GetLastFrameTime()
{
    return this->frame_timing_;
}

bool PoseVRDriver::VRDriver::AddDevice(std::shared_ptr<IVRDevice> device)
{
    vr::ETrackedDeviceClass openvr_device_class;
    // Remember to update this switch when new device types are added
    switch (device->GetDeviceType()) {
        case DeviceType::CONTROLLER:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_Controller;
            break;
        case DeviceType::HMD:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_HMD;
            break;
        case DeviceType::TRACKER:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker;
            break;
        case DeviceType::TRACKING_REFERENCE:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference;
            break;
        default:
            return false;
    }
    Log("tracker device added");
    bool result = vr::VRServerDriverHost()->TrackedDeviceAdded(device->GetSerial().c_str(), openvr_device_class, device.get());
    if (result)
        this->devices_.push_back(device);
        
    return result;
}

PoseVRDriver::SettingsValue PoseVRDriver::VRDriver::GetSettingsValue(std::string key)
{
    vr::EVRSettingsError err = vr::EVRSettingsError::VRSettingsError_None;
    int int_value = vr::VRSettings()->GetInt32(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return int_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;
    float float_value = vr::VRSettings()->GetFloat(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return float_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;
    bool bool_value = vr::VRSettings()->GetBool(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return bool_value;
    }
    std::string str_value;
    str_value.reserve(1024);
    vr::VRSettings()->GetString(settings_key_.c_str(), key.c_str(), str_value.data(), 1024, &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return str_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;

    return SettingsValue();
}

void PoseVRDriver::VRDriver::Log(std::string message)
{
    std::string message_endl = message + "\n";
    vr::VRDriverLog()->Log(message_endl.c_str());
}

vr::IVRDriverInput* PoseVRDriver::VRDriver::GetInput()
{
    return vr::VRDriverInput();
}

vr::CVRPropertyHelpers* PoseVRDriver::VRDriver::GetProperties()
{
    return vr::VRProperties();
}

vr::IVRServerDriverHost* PoseVRDriver::VRDriver::GetDriverHost()
{
    return vr::VRServerDriverHost();
}

void PoseVRDriver::VRDriver::PipeThread()
{
    Log("Reading data from pipe...");


    // The read operation will block until there is data to read
    char buffer[512];
    DWORD numBytesRead = 0;
    while (true)
    {

        BOOL result = ReadFile(
            pipe,
            buffer, // the data from the pipe will be put here
            511 * sizeof(char), // number of bytes allocated
            &numBytesRead, // this will store number of bytes actually read
            NULL // not using overlapped IO
        );

        if (result) {
            buffer[numBytesRead / sizeof(char)] = '\0'; // null terminate the string
            /*wcout << "Number of bytes read: " << numBytesRead << endl;
            wcout << "Message: " << buffer << endl;*/
            switch (buffer[0])
            {
            case 'n':   //no data
                break;
            case 'c':   //calibrate
                poseData.calibrating1 = true;
                poseData.clear_calibration_data();
                // CLEAR calibration2_data
                break;
            default:    //
                std::lock_guard<std::mutex> guard(mymutex);
                //GetCurHMDControllersPose();
                poseData.add_data(buffer);
                
                break;
            }
        }
        else {
            //wcout << "Failed to read data from the pipe." << endl;
            break; // maybe not break here? not sure
        }
    }
    // Close our pipe handle
    CloseHandle(pipe);

    Log("Closing pipe PoseVR.");

    //system("pause");
}


void PoseVRDriver::VRDriver::GetCurHMDControllersPose()
{
    int hmd_idx = 0;

    vr::TrackedDevicePose_t hmd_pose[10];
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, hmd_pose, 10);

    vr::HmdQuaternion_t q_hmd = GetRotation(hmd_pose[hmd_idx].mDeviceToAbsoluteTracking);
    vr::HmdVector3_t pos_hmd = GetPosition(hmd_pose[hmd_idx].mDeviceToAbsoluteTracking);

    if (pos_hmd.v != NULL)
    {
        poseData.cur_hmd.clear();
        poseData.cur_hmd.push_back(pos_hmd.v[0]);
        poseData.cur_hmd.push_back(pos_hmd.v[1]);
        poseData.cur_hmd.push_back(pos_hmd.v[2]);
        poseData.cur_hmd.push_back(q_hmd.w);
        poseData.cur_hmd.push_back(q_hmd.x);
        poseData.cur_hmd.push_back(q_hmd.y);
        poseData.cur_hmd.push_back(q_hmd.z);
    }

    if (!poseData.controller_ids_found)
    {
        int num_devices = 0;    //TODO: temporary fix. may break later if using base stations
        for (int i = 0; i < 10; i++)
        {
            auto props = vr::VRProperties()->TrackedDeviceToPropertyContainer(i);

            std::string modelNumber = vr::VRProperties()->GetStringProperty(props, vr::Prop_ModelNumber_String);

            if (modelNumber != "")
            {
                num_devices += 1;
            }
        }
        if (prev_num_devices != num_devices)
        {
            for (int i = 0; i < 10; i++)
            {
                auto props = vr::VRProperties()->TrackedDeviceToPropertyContainer(i);

                std::string modelNumber = vr::VRProperties()->GetStringProperty(props, vr::Prop_ModelNumber_String);

                transform(modelNumber.begin(), modelNumber.end(), modelNumber.begin(), std::tolower);   // make string lower case

                if (modelNumber.find("left") != std::string::npos)
                    left_controller_idx = i;
                if (modelNumber.find("right") != std::string::npos)
                    right_controller_idx = i;
            }

            if (left_controller_idx != -1 && right_controller_idx != -1)
                poseData.controller_ids_found = true;

            prev_num_devices = num_devices;
        }
    }
    else
    {
        if (left_controller_idx == 0 && right_controller_idx == 0)
        {
            Log("error: left_controller_idx == 0 or right_controller_idx == 0");
            return;
        }
            
        //send controller data to posedata
        vr::HmdQuaternion_t q_left_controller = GetRotation(hmd_pose[left_controller_idx].mDeviceToAbsoluteTracking);
        vr::HmdVector3_t pos_left_controller = GetPosition(hmd_pose[left_controller_idx].mDeviceToAbsoluteTracking);
        poseData.cur_left_controller.clear();
        poseData.cur_left_controller.push_back(pos_left_controller.v[0]);
        poseData.cur_left_controller.push_back(pos_left_controller.v[1]);
        poseData.cur_left_controller.push_back(pos_left_controller.v[2]);

        vr::HmdQuaternion_t q_right_controller = GetRotation(hmd_pose[right_controller_idx].mDeviceToAbsoluteTracking);
        vr::HmdVector3_t pos_right_controller = GetPosition(hmd_pose[right_controller_idx].mDeviceToAbsoluteTracking);
        poseData.cur_right_controller.clear();
        poseData.cur_right_controller.push_back(pos_right_controller.v[0]);
        poseData.cur_right_controller.push_back(pos_right_controller.v[1]);
        poseData.cur_right_controller.push_back(pos_right_controller.v[2]);
    }
}


//-----------------------------------------------------------------------------
// Purpose: Calculates quaternion (qw,qx,qy,qz) representing the rotation
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

vr::HmdQuaternion_t PoseVRDriver::VRDriver::GetRotation(vr::HmdMatrix34_t matrix) {
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}
//-----------------------------------------------------------------------------
// Purpose: Extracts position (x,y,z).
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

vr::HmdVector3_t PoseVRDriver::VRDriver::GetPosition(vr::HmdMatrix34_t matrix) {
    vr::HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}