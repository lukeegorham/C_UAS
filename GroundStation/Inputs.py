import socket
from time import sleep, time
import struct
import queue
import tkinter as tk
from tkinter import *
import threading
import GUI

def main():
    master = tk.Tk()
    receiver = C2RX(master)
    master.mainloop()

class C2RX():
    def __init__(self, master):
        self.rad_data = [-1, -1, -1, -1, -1, -1, -1]
        self.queue = queue.Queue()
        self.master = master
        statuses = [1, 1, 1, 1, 1, 1]
        info = [-1, -1, -1]
        self.gui = GUI.C2GUI(self.master, statuses, info, self.queue)
        self.running = 1
        self.thread1 = threading.Thread(target=self.start_server())
        self.thread1.start()

    def periodicCall(self):
        self.gui.processIncoming()
        if not self.running:
            self.running = 1
        self.master.after(200, self.periodicCall())


    def start_server(self):
        """
        Creates and starts the UDP server to grap all of our data. It binds to a port and continually listens.
        :param shared_memory: the shared memory object across the server
        :return: radar data message
        """

        # Defines our IP's
        radar_ip = '192.168.1.60'

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        port_num = 55565
        server_socket.bind(('', port_num))
        # radar_message_length = 72
        sleep(1)
        print("UDP SERVER: RUNNING")

        while self.running:
            data, addr = server_socket.recvfrom(100000)

            if addr[0] == radar_ip:
                length = len(data)
                form = '<' + str(int(length / 8)) + 'd'
                locations = struct.unpack(form, data)
                self.rad_data = locations
                self.queue.put(locations)
                self.running = 0
                break

            else:
                print("UNRECOGNIZED IP: ", addr)
                self.rad_data = [-1, -1, -1, -1, -1, -1, -1]
                self.queue.put(self.rad_data)
                break

    def get_rad(self):
        return self.rad_data