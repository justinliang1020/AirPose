import win32pipe
import win32file
import pywintypes

# posevr modules
import posevr_client

# setup pipe
PIPE_NAME = 'posevr_pipe'
pipe_connected = False 


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
        win32file.WriteFile(
            pipe,
            some_data,
            pywintypes.OVERLAPPED())

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

def create_pipe():
    pipe = win32pipe.CreateNamedPipe(f'\\\\.\\pipe\\{PIPE_NAME}', win32pipe.PIPE_ACCESS_DUPLEX | win32file.FILE_FLAG_OVERLAPPED,
                                     win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_READMODE_MESSAGE | win32pipe.PIPE_WAIT,
                                     1, 65536, 65536, 0, None)
    print(f'Pipe {PIPE_NAME} created.')
    return pipe

# function for calibration
def calibrate():
    if pipe_connected:
        global calibrating
        calibrating = True

def close_pipe():
    # ends pipe server by connecting a dummy client pipe and closing
    if not pipe_connected:
        posevr_client.named_pipe_client()