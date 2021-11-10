# external modules
from cv2 import VideoCapture, VideoWriter, VideoWriter_fourcc, FONT_HERSHEY_SIMPLEX, flip, transpose, cvtColor, COLOR_RGB2BGR
import time
import mediapipe as mp
import win32pipe
import win32file
import pywintypes
import os
import sys
from threading import Thread
from tkinter import Tk, Frame, Label, Button #fix later
from PIL import ImageTk, Image

# posevr modules
import posevr_client

# setup pipe
pipe_name = 'posevr_pipe'
pipe_connected = False 

# setup webcam
cap = VideoCapture(0)
success, image = cap.read()
image_height, image_width, _ = image.shape

# function for calibration
def calibrate():
    if pipe_connected:
        global calibrating
        calibrating = True

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)

Logo = resource_path("favicon.ico")

# tkinter app stuff
root = Tk()
root.title('Airpose')
root.iconbitmap(Logo)
# Create a frame
app = Frame(root, bg="white")
app.grid()
# Create a label for video stream
video_label = Label(app)
video_label.grid(row=0,column=0,columnspan=1)
# Create calibration button
calibration_button = Button(root, text="Calibrate", command=calibrate, width = 50, height = 5, bg = 'green')
calibration_button.grid(row=1,column=0)
calibrating = False
initial_calibrating = False

# setup mediapipe
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.8,min_tracking_confidence=0.8,smooth_landmarks=True) 

# recorded positions
positions = ['LEFT_HIP','RIGHT_HIP','LEFT_ANKLE','RIGHT_ANKLE','LEFT_WRIST','RIGHT_WRIST', 'NOSE', 'LEFT_FOOT_INDEX', 'RIGHT_FOOT_INDEX']
heel_height = .8
hand_height = .3
prev_time = 0
prev_landmarks = {}
min_visibility = 0.7


def get_landmarks(positions, results):
    # returns a dict of {position : (position.x, position.y, position.z)}
    res = {}
    for position in positions:
        r = results.pose_landmarks.landmark[getattr(mp_pose.PoseLandmark, position)]
        res[position] = (r.x, r.y, r.z)
    return res

def start_pipe(pipe):
    """Waits until pipe server connects to pipe client

    Args:
        pipe ([type]): [description]
    """
    global pipe_connected
    global initial_calibrating
    print('Waiting for client to connect...')
    win32pipe.ConnectNamedPipe(pipe, None) 
    print('Client is connected')
    if posevr_client.pipe_ended:
        return
    pipe_connected = True
    time.sleep(8) # wait for driver to get set up TODO: fix
    initial_calibrating = True

def video_stream():
    try:    #try is attempt to cleanup app after steamvr disconnects
        image = get_camera_image(cap)
        results = pose.process(image)        
        
        send = get_data(results)
                
        send_data_to_pipe(send)
        
        # Draw the pose annotation on the image.
        image.flags.writeable = True
        mp_drawing.draw_landmarks(
            image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        image = cvtColor(image, COLOR_RGB2BGR)

        # send image to tkinter app
        img = Image.fromarray(image)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)

        
        #loops back
        video_label.after(1, video_stream)
    except:
        root.quit()
        return

def get_data(results):
    global prev_landmarks
    global prev_time

    send = 'n'

    if results.pose_landmarks:
        landmarks = {}
        for position in positions:
            r = results.pose_landmarks.landmark[getattr(mp_pose.PoseLandmark, position)]
            if not r or (r.visibility) < min_visibility:
                if position in prev_landmarks:
                    landmarks[position] = prev_landmarks[position]
                else:
                    landmarks[position] = (0, 0, 0) # only happens at very beginning if landmarks occluded
            else:
                landmarks[position] = (r.x, r.y, r.z)
        
        cur_time = time.time() - prev_time
        prev_time = time.time()

        send = f'{round(cur_time, 5)};'
        for landmark, coord in landmarks.items():
            send += f'{landmark};{round(coord[0], 5)};{round(coord[1], 5)};{round(coord[2], 5)};'

        prev_landmarks = landmarks
    return send

def get_camera_image(cap: VideoCapture):
    success, image = cap.read()
        
    if not success:
        print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
        video_label.after(1, video_stream)

        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
    image = flip(image, 1)

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
    image.flags.writeable = False
    return image

def send_data_to_pipe(send: str):
    global calibrating
    global initial_calibrating

    if pipe_connected:
        if calibrating or initial_calibrating:
            send = 'c'
            calibrating = False
            initial_calibrating = False
        some_data = str.encode(str(send), encoding="ascii") # about 150 chars for 4 positions, rounded 5 decimal points
            # Send the encoded string to client
        err, bytes_written = win32file.WriteFile(
                pipe,
                some_data,
                pywintypes.OVERLAPPED()
            )


if __name__ == '__main__':
    # starts pipe in seperate thread
    pipe = win32pipe.CreateNamedPipe(f'\\\\.\\pipe\\{pipe_name}', win32pipe.PIPE_ACCESS_DUPLEX | win32file.FILE_FLAG_OVERLAPPED,
                                     win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_READMODE_MESSAGE | win32pipe.PIPE_WAIT,
                                     1, 65536, 65536, 0, None)
    print(f'Pipe {pipe_name} created.')
    t1 = Thread(target=start_pipe, args=(pipe,))
    t1.start()
    
    prev_time = time.time()

    root.after(0, video_stream)
    root.mainloop()

    # ends pipe server by connecting a dummy client pipe and closing
    if not pipe_connected:
        posevr_client.named_pipe_client()
    
    cap.release()
