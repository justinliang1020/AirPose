#pragma once

#include <chrono>
#include <cmath>

#include <linalg.h>

#include <Driver/IVRDevice.hpp>
#include <Native/DriverFactory.hpp>

#include <windows.h>    // used in VRDriver but can only be included here???
#include <thread>
#include <string>
#include <mutex>
#include <algorithm>

namespace PoseVRDriver {
    class TrackerDevice : public IVRDevice {
        public:

            TrackerDevice(std::string serial);
            ~TrackerDevice() = default;

            // Inherited via IVRDevice
            virtual std::string GetSerial() override;
            virtual void Update() override;
            virtual vr::TrackedDeviceIndex_t GetDeviceIndex() override;
            virtual DeviceType GetDeviceType() override;

            virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
            virtual void Deactivate() override;
            virtual void EnterStandby() override;
            virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
            virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
            virtual vr::DriverPose_t GetPose() override;

            // PoseVR methods
            void UpdatePos(double x, double y, double z);
            void UpdateRot(double w, double x, double y, double z);

            vr::DriverPose_t wanted_pose_ = IVRDevice::MakeDefaultPose();

    private:
        vr::TrackedDeviceIndex_t device_index_ = vr::k_unTrackedDeviceIndexInvalid;
        std::string serial_;

        vr::DriverPose_t last_pose_ = IVRDevice::MakeDefaultPose();

        bool did_vibrate_ = false;
        float vibrate_anim_state_ = 0.f;

        vr::VRInputComponentHandle_t haptic_component_ = 0;

        vr::VRInputComponentHandle_t system_click_component_ = 0;
        vr::VRInputComponentHandle_t system_touch_component_ = 0;

        // PoseVR variables
        std::chrono::milliseconds prev_time;
        const double velocity_smoothing = 0.2;  // lower number, more smoothing (0,1)
        const double position_smoothing = 0.2;  
        const double rotation_smoothing = 0.2;
    };
};