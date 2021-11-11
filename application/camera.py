from tkinter import Image
from typing import Dict, List

from cv2 import VideoCapture, flip

def get_camera_image(cap: VideoCapture) -> Image:
    _, image = cap.read()

    # Flip the image horizontally for a later selfie-view display, and convert
    # the BGR image to RGB.
    image = flip(image, 1)

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    return image

def initialize_camera() -> Dict[str, VideoCapture]:
    cameras = {}
    for i in range(100):
        cap =  VideoCapture(i)
        if cap.isOpened():
            cameras[str(i)] = cap
        else:
            cap.release() 
    return cameras

def get_cameras() -> Dict[str, VideoCapture]:
    return cameras

selected_camera = "0"

def get_camera() -> VideoCapture:
    global selected_camera
    return cameras[selected_camera]

def close_camera():
    cap.release()

cameras = initialize_camera()
# setup webcam
cap = get_camera()
_, image = cap.read()
image_height, image_width, _ = image.shape


def set_selected_camera(selected: str):
    global selected_camera
    selected_camera = selected