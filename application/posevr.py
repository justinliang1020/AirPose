# external modules
from cv2 import VideoCapture, cvtColor, COLOR_RGB2BGR
import time
import mediapipe as mp

from threading import Thread
from PIL import ImageTk, Image

from camera import get_camera_image
from ui import init_calibrate_button, init_tkinter_app, init_video_output
from pipe import calibrate, close_pipe, create_pipe, send_data_to_pipe, start_pipe

# setup webcam
cap = VideoCapture(0)
_, image = cap.read()
image_height, image_width, _ = image.shape

# setup mediapipe
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.8,min_tracking_confidence=0.8,smooth_landmarks=True) 

# recorded positions
POSITIONS = ['LEFT_HIP','RIGHT_HIP','LEFT_ANKLE','RIGHT_ANKLE','LEFT_WRIST','RIGHT_WRIST', 'NOSE', 'LEFT_FOOT_INDEX', 'RIGHT_FOOT_INDEX']
HEEL_HEIGHT = .8
HAND_HEIGHT = .3
MIN_VISIBILITY = 0.7

prev_time = 0
prev_landmarks = {}

def video_stream_loop():
    global video_label
    global root
    try:    #try is attempt to cleanup app after steamvr disconnects
        process_camera_image()
  
        #loops back
        video_label.after(1, video_stream_loop)
    except:
        root.quit()
        return

def process_camera_image() -> Image:
    image = get_camera_image(cap)
    results = pose.process(image)        
    data = convert_to_pipe_data(results)
    send_data_to_pipe(data)
    update_video_output(image, results.pose_landmarks)

def update_video_output(image: Image, landmarks):
    # Draw the pose annotation on the image.
    image.flags.writeable = True
    mp_drawing.draw_landmarks(image, landmarks, mp_pose.POSE_CONNECTIONS)
    image = cvtColor(image, COLOR_RGB2BGR)

    # send image to tkinter app
    img = Image.fromarray(image)
    imgtk = ImageTk.PhotoImage(image=img)
    video_label.imgtk = imgtk
    video_label.configure(image=imgtk)

def convert_to_pipe_data(results):
    global prev_landmarks
    global prev_time

    send = 'n'

    if results.pose_landmarks:
        landmarks = {}
        for position in POSITIONS:
            r = results.pose_landmarks.landmark[getattr(mp_pose.PoseLandmark, position)]
            if not r or (r.visibility) < MIN_VISIBILITY:
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

if __name__ == '__main__':
    # starts pipe in seperate thread
    pipe = create_pipe()
    t1 = Thread(target=start_pipe, args=(pipe,))
    t1.start()
    
    prev_time = time.time()

    root = init_tkinter_app()
    init_calibrate_button(root, calibrate)
    video_label = init_video_output(root)
    root.after(0, video_stream_loop)
    root.mainloop()

    close_pipe()
    
    cap.release()
