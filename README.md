# AirPose
 Computer vision based full body tracking for VR.
 Does XY-tracking quite well, Z-tracking (depth) is still experimental.
 
[![Airpose Demo Video](https://img.youtube.com/vi/b5pe5vCh3a8/0.jpg)](https://www.youtube.com/watch?v=b5pe5vCh3a8)


# Install instructions

1. [Download "airpose.7z"](https://github.com/justinliang1020/AirPose/releases/download/v0.1/AirPose.7z) and unzip the file.
2. Copy-paste the airpose folder into "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers"
3. Make sure you have a webcam connected to your computer pointed at your playspace.
4. Run Airpose App
5. Start SteamVR
6. If your controllers don't show up, reassign trackers from hand to respective positions (waist, left foot, right foot) in SteamVR on your desktop from the "Manage Vive Trackers" settings tab. Restart SteamVR after you do this.
![Vive trackers settings](/trackers.png)

Note: Airpose will only run if you run the Airpose App before starting SteamVR. Otherwise, it won't impact your VR gameplay.

# Calibration instructions

1. Run airpose app
2. Start SteamVR
3. While SteamVR is starting up, face the camera and stay still
4. Count for about 5 seconds after SteamVR starts up, and after that it should be calibrated
5. If you need to recalibrate, hit the calibrate button and stand still for 5 seconds facing the camera

# Dev stuff

I still need to do some dev stuff with cmake to make the driver files buildable.

The OpenVR driver is built on [terminal29's openvr sample driver](https://github.com/terminal29/Simple-OpenVR-Driver-Tutorial)
