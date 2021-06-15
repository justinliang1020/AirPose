# AirPose
 Computer vision based full body tracking for VR
 
[![Airpose Demo Video](https://img.youtube.com/vi/b5pe5vCh3a8/0.jpg)](https://www.youtube.com/watch?v=b5pe5vCh3a8)


# Install instructions

[Install here](https://github.com/justinliang1020/AirPose/releases/tag/v0.1)
1. Download "airpose.7z" and unzip the file.
2. Copy-paste the airpose folder into "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers"
3. Make sure you have a webcam connected to your computer pointed at your playspace.
4. Run Airpose App
5. Start SteamVR
6. If your controllers don't show up, reassign trackers from hand to respective positions (waist, left foot, right foot) in SteamVR on your desktop from the "Manage Vive Trackers" settings tab. Restart SteamVR after you do this.

Note: Airpose will only run if you run the Airpose App before starting SteamVR. Otherwise, it won't impact your VR gameplay.

# Dev stuff

I still need to do some dev stuff to make the driver files buildable.

The OpenVR driver is built on [terminal29's openvr sample driver](https://github.com/terminal29/Simple-OpenVR-Driver-Tutorial)
