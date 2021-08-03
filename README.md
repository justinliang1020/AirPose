# AirPose
 Computer vision based full body tracking for VR.
 Does XY-tracking quite well, Z-tracking (depth) is still experimental.
 
 [Demo video:](https://www.youtube.com/watch?v=b5pe5vCh3a8)
 
[![Airpose Demo Video](https://img.youtube.com/vi/b5pe5vCh3a8/0.jpg)](https://www.youtube.com/watch?v=b5pe5vCh3a8)


# Install instructions

1. [Download "airpose.release.7z"](https://github.com/justinliang1020/AirPose/releases/download/v0.1/AirPose.7z) and unzip the file.
2. Open up the airpose.release folder. You should see "airpose" folder and and Airpose App
![image](https://user-images.githubusercontent.com/54543035/123564974-f28c5180-d780-11eb-9914-3f0d2d29f856.png)
3. Copy-paste the airpose folder into "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers"
4. Make sure you have a webcam connected to your computer pointed at your playspace.
5. Run Airpose App
6. Start SteamVR
7. If your controllers don't show up, reassign trackers from hand to respective positions (waist, left foot, right foot) in SteamVR on your desktop from the "Manage Vive Trackers" settings tab. Restart SteamVR after you do this.
![Vive trackers settings](/trackers.png)

Note: Airpose will only run if you run the Airpose App before starting SteamVR. Otherwise, it won't impact your VR gameplay.

# Calibration instructions

1. Run airpose app
2. Start SteamVR
3. While SteamVR is starting up, face the camera and stay still
4. Count for about 5 seconds after SteamVR starts up, and after that it should be calibrated
5. If you need to recalibrate, hit the calibrate button and stand still for 5 seconds facing the camera

# Tips

1. Use a camera with a higher resolution for better tracking. I use my phone's camera by connecting it with Iriun Webcam.
2. Having a background with low contrast to your body may decrease performance

# Dev stuff

The OpenVR driver is built on [terminal29's openvr sample driver](https://github.com/terminal29/Simple-OpenVR-Driver-Tutorial)
