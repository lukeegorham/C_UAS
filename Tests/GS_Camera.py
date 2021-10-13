import socket
import cv2
import threading
import numpy as np
import zmq
import base64

# Global Variables
host_gcam = '192.168.1.26'  # Address of receiver
port_gcam = 46554
host_loc = '192.168.1.50'
port_loc_video = 46554
global g_cam


# Initialize Networking for Sending Commands
def init_cmd_send():
    global g_cam
    g_cam = socket.socket()
    g_cam.connect((host_gcam, port_gcam))


# Listener for Camera Video Feed
def recv_video():
    # Initialize
    context = zmq.Context()
    v_feed = context.socket(zmq.SUB)
    v_feed.bind('tcp://*:46554')
    v_feed.setsockopt_string(zmq.SUBSCRIBE, np.compat.unicode(''))
    # v_feed.listen(10)
    # feed, addr = v_feed.accept()
    # Display Feed
    while True:
        frame = v_feed.recv_string()
        img = base64.b64decode(frame)
        npimg = np.fromstring(img, dtype=np.uint8)
        source = cv2.imdecode(npimg, 1)
        if source is None:
            print(f'Nothing To Display')
        cv2.imshow("Live Feed", source)
        cv2.waitKey(1)
        print(f'1')


def main():
    global g_cam
    # Start connection to send commands
    init_cmd_send()
    # Start receiving video feed
    thread_1 = threading.Thread(target=recv_video, args=(), daemon=True)
    thread_1.start()
    # Get and send commands until quit
    message = None
    while message != 'q':
        message = input("->")
        g_cam.send(bytes(message, 'utf8'))
        data = g_cam.recv(32)
    # Once told to quit, exit nicely
    g_cam.close()
    quit()


if __name__ == '__main__':
    main()
