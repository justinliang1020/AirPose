from tkinter import Image

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