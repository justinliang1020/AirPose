# tkinter app stuff
import os
import sys
from tkinter import Button, Frame, Label, Tk
from typing import Callable


def init_tkinter_app() -> Tk:
    Logo = resource_path("favicon.ico")
    root = Tk()
    root.title('Airpose')
    root.iconbitmap(Logo)
    # Create a frame
    app = Frame(root, bg="white")
    app.grid()
    return app

def init_calibrate_button(app: Tk, command: Callable):
    # Create calibration button
    calibration_button = Button(app, text="Calibrate", command=command, width = 50, height = 5, bg = 'green')
    calibration_button.grid(row=1,column=0)

def init_video_output(app: Tk) -> Label:
    # Create a label for video stream
    video_label = Label(app)
    video_label.grid(row=0,column=0,columnspan=1)
    return video_label

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)
    