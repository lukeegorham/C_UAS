import socket
import cv2
import threading
import numpy as np
import zmq
import base64

# Global Variables
self = '192.168.1.31'  # Address of receiver
port = 46554
host_loc = ''
g_cam = None
v_feed = None


# Initialize Networking for Sending Commands
def init_cmd_send():
    global g_cam
    g_cam = socket.socket()
    try:
        g_cam.connect((self, port))
    except:
        print("Connection Refused!")
        quit()


# Listener for Camera Video Feed
def recv_video():
    # Initialize
    global v_feed
    context = zmq.Context()
    v_feed = context.socket(zmq.SUB)
    v_feed.bind('tcp://*:' + str(port))
    v_feed.setsockopt_string(zmq.SUBSCRIBE, np.compat.unicode(''))
    # Display Feed
    while True:
        frame = v_feed.recv_string()                # Receive from connection
        img = base64.b64decode(frame)               # Decode raw frame
        npimg = np.fromstring(img, dtype=np.uint8)  # Convert frame to Numpy image
        source = cv2.imdecode(npimg, 1)             # Decode Numpy image to frame to display with cv2
        if source is None:
            print("Nothing To Display")
        cv2.imshow("Live Feed", source)
        cv2.waitKey(1)


def main():
    global g_cam
    global v_feed
    # Start connection to send commands
    init_cmd_send()
    # Start receiving video feed
    thread_1 = threading.Thread(target=recv_video, args=(), daemon=True)
    thread_1.start()
    # Get and send commands until quit
    message = None
    while message != 'q':
        message = input("->")  # Get message - NOTE: blocks execution (which is perfectly okay)
        g_cam.send(bytes(message, 'utf8'))
        data = g_cam.recv(32)
    # Once told to quit, exit nicely
    g_cam.shutdown(1)
    g_cam.close()
    v_feed.close()
    quit()


if __name__ == '__main__':
    main()
