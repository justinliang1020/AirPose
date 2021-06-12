#include "TrackerDevice.hpp"
#include <Windows.h>

PoseVRDriver::TrackerDevice::TrackerDevice(std::string serial):
    serial_(serial)
{
}

std::string PoseVRDriver::TrackerDevice::GetSerial()
{
    return this->serial_;
}

void PoseVRDriver::TrackerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    // Check if this device was asked to be identified
    auto events = GetDriver()->GetOpenVREvents();
    for (auto event : events) {
        // Note here, event.trackedDeviceIndex does not necissarily equal this->device_index_, not sure why, but the component handle will match so we can just use that instead
        //if (event.trackedDeviceIndex == this->device_index_) {
        if (event.eventType == vr::EVREventType::VREvent_Input_HapticVibration) {
            if (event.data.hapticVibration.componentHandle == this->haptic_component_) {
                this->did_vibrate_ = true;
            }
        }
        //}
    }

    // Check if we need to keep vibrating
    if (this->did_vibrate_) {
        this->vibrate_anim_state_ += (GetDriver()->GetLastFrameTime().count()/1000.f);
        if (this->vibrate_anim_state_ > 1.0f) {
            this->did_vibrate_ = false;
            this->vibrate_anim_state_ = 0.0f;
        }
    }
    // REWRITE SO I KNOW WHAT GOING ON
    // velocity interplotation does help a lot, just need some motion smoothign now
    // Update time delta (for working out velocity)
    std::chrono::milliseconds time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    double cur_time = time_since_epoch.count() / 1000.0;
    double delta_time = (time_since_epoch - prev_time).count() / 1000.0;

    // Update pose timestamp

    prev_time = time_since_epoch;

    // Setup pose for this frame
    auto pose = this->wanted_pose_;
    

    // TODO: velocity for linear interpolation?
    pose.vecVelocity[0] = velocity_smoothing * ((wanted_pose_.vecPosition[0] - last_pose_.vecPosition[0]) / delta_time) + (1 - velocity_smoothing) * pose.vecVelocity[0];
    pose.vecVelocity[1] = velocity_smoothing * ((wanted_pose_.vecPosition[1] - last_pose_.vecPosition[1]) / delta_time) + (1 - velocity_smoothing) * pose.vecVelocity[1];
    pose.vecVelocity[2] = velocity_smoothing * ((wanted_pose_.vecPosition[2] - last_pose_.vecPosition[2]) / delta_time) + (1 - velocity_smoothing) * pose.vecVelocity[2];

    // Post pose
    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
}



DeviceType PoseVRDriver::TrackerDevice::GetDeviceType()
{
    return DeviceType::TRACKER;
}

vr::TrackedDeviceIndex_t PoseVRDriver::TrackerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

vr::EVRInitError PoseVRDriver::TrackerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating tracker " + this->serial_);

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    // Setup inputs and outputs
    /*GetDriver()->GetInput()->CreateHapticComponent(props, "/output/haptic", &this->haptic_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/click", &this->system_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/touch", &this->system_touch_component_);*/

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 2);
    
    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "example_tracker");

    // Opt out of hand selection
    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);

    // Set up a render model path
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "{htc}/rendermodels/vr_tracker_vive_1_0");

    // Set controller profile
    //GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{example}/input/example_tracker_bindings.json");

    // Set the icon
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, "{example}/icons/tracker_ready.png");

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, "{example}/icons/tracker_not_ready.png");
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, "{example}/icons/tracker_not_ready.png");

    // create switch statement based on serial id to create hip and feet trackers
    std::string role;
    switch (this->serial_[0])
    {
    case 0:
        role = "vive_tracker_waist";
        // set starting pose
        this->wanted_pose_.vecPosition[1] = 1;
        break;
    case '1':
        role = "vive_tracker_left_foot";
        // set starting pose
        this->wanted_pose_.vecPosition[0] = -0.3;
        break;
    case '2':
        role = "vive_tracker_right_foot";
        // set starting pose
        this->wanted_pose_.vecPosition[0] = 0.3;
        break;
    }

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, role.c_str());

    

    return vr::EVRInitError::VRInitError_None;
}

void PoseVRDriver::TrackerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void PoseVRDriver::TrackerDevice::EnterStandby()
{
}

void* PoseVRDriver::TrackerDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void PoseVRDriver::TrackerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t PoseVRDriver::TrackerDevice::GetPose()
{
    return last_pose_;
}

// PoseVR methods
void PoseVRDriver::TrackerDevice::UpdatePos(double x, double y, double z)
{
    
    this->wanted_pose_.vecPosition[0] = position_smoothing * x + (1 - position_smoothing) * this->wanted_pose_.vecPosition[0];  //can do some motion smoothing here?
    this->wanted_pose_.vecPosition[1] = position_smoothing * y + (1 - position_smoothing) * this->wanted_pose_.vecPosition[1];
    this->wanted_pose_.vecPosition[2] = position_smoothing * z + (1 - position_smoothing) * this->wanted_pose_.vecPosition[2];
}

void PoseVRDriver::TrackerDevice::UpdateRot(double w, double x, double y, double z)
{
    struct vr::HmdQuaternion_t q = { w, x, y, z };

    // lerp smoothing JUICE
    double dot = w * this->wanted_pose_.qRotation.w + x * this->wanted_pose_.qRotation.x + y * this->wanted_pose_.qRotation.y + z * this->wanted_pose_.qRotation.z;

    if (dot < 0)
    {
        q.w = rotation_smoothing * w - (1 - rotation_smoothing) * this->wanted_pose_.qRotation.w;
        q.x = rotation_smoothing * x - (1 - rotation_smoothing) * this->wanted_pose_.qRotation.x;
        q.y = rotation_smoothing * y - (1 - rotation_smoothing) * this->wanted_pose_.qRotation.y;
        q.z = rotation_smoothing * z - (1 - rotation_smoothing) * this->wanted_pose_.qRotation.z;
    }
    else
    {
        q.w = rotation_smoothing * w + (1 - rotation_smoothing) * this->wanted_pose_.qRotation.w;
        q.x = rotation_smoothing * x + (1 - rotation_smoothing) * this->wanted_pose_.qRotation.x;
        q.y = rotation_smoothing * y + (1 - rotation_smoothing) * this->wanted_pose_.qRotation.y;
        q.z = rotation_smoothing * z + (1 - rotation_smoothing) * this->wanted_pose_.qRotation.z;
    }
    //normalize
    double mag = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    
    q.w /= mag;
    q.x /= mag;
    q.y /= mag;
    q.z /= mag;

    this->wanted_pose_.qRotation = q;   // probablty can do some smoothing/slerp stuff here
}