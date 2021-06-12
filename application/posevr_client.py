import win32file
import pywintypes

pipe_name = 'posevr_pipe'
pipe_ended = False

def named_pipe_client():
    global pipe_ended
    print('Connecting to server...')
    while True:
        try:
            handle = win32file.CreateFile(f'\\\\.\\pipe\\{pipe_name}', win32file.GENERIC_READ | win32file.GENERIC_WRITE,
                                        0, None, win32file.OPEN_EXISTING, win32file.FILE_ATTRIBUTE_NORMAL, None)
            print('Ending pipe...')
            pipe_ended = True
            return
        except pywintypes.error as e:
            print(e.args)
            if e.args[0] == 2:   # ERROR_FILE_NOT_FOUND
                print("No Named Pipe")
            elif e.args[0] == 109:   # ERROR_BROKEN_PIPE
                print("Named Pipe is broken")
            print('Could not connect to server')
            break
