#pragma once

#include <vector>
#include <memory>
//#include <windows.h>
//#include <thread>     // can't include these or build breaks!

#include <openvr_driver.h>

#include <Driver/IVRDriver.hpp>
#include <Driver/IVRDevice.hpp>
#include <Driver/TrackerDevice.hpp>

#include "PoseData.hpp"

namespace PoseVRDriver {
    class VRDriver : public IVRDriver {
    public:


        // Inherited via IVRDriver
        virtual std::vector<std::shared_ptr<IVRDevice>> GetDevices() override;
        virtual std::vector<vr::VREvent_t> GetOpenVREvents() override;
        virtual std::chrono::milliseconds GetLastFrameTime() override;
        virtual bool AddDevice(std::shared_ptr<IVRDevice> device) override;
        virtual SettingsValue GetSettingsValue(std::string key) override;
        virtual void Log(std::string message) override;

        virtual vr::IVRDriverInput* GetInput() override;
        virtual vr::CVRPropertyHelpers* GetProperties() override;
        virtual vr::IVRServerDriverHost* GetDriverHost() override;

        // Inherited via IServerTrackedDeviceProvider
        virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;
        virtual void Cleanup() override;
        virtual void RunFrame() override;
        virtual bool ShouldBlockStandbyMode() override;
        virtual void EnterStandby() override;
        virtual void LeaveStandby() override;
        virtual ~VRDriver() = default;



    private:
        std::vector<std::shared_ptr<IVRDevice>> devices_;
        std::vector<vr::VREvent_t> openvr_events_;
        std::chrono::milliseconds frame_timing_ = std::chrono::milliseconds(16);
        std::chrono::system_clock::time_point last_frame_time_ = std::chrono::system_clock::now();
        std::string settings_key_ = "driver_example";

        // PoseVR methods
        void PipeThread();  //creates multithreaded named pipe to communicate with PoseVR python application
        std::vector<std::shared_ptr<TrackerDevice>> trackers_;
        vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);
        vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix);
        void GetCurHMDControllersPose();
        

        // PoseVR variables
        PoseData poseData = PoseData();
        HANDLE pipe;
        bool pipe_connected = true;
        bool tested = false; //only for debugging
        int prev_num_devices = 0;
        int left_controller_idx = -1;
        int right_controller_idx = -1;
        

        double jump_threshold = 0.1;
        double sit_threshold = 0.2;
        double sit_foot_depth_multiplier = 0.4;
        
    };
};